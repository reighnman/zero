#include "TeamVersusBehavior.h"

#include <zero/behavior/BehaviorBuilder.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/behavior/nodes/BlackboardNode.h>
#include <zero/behavior/nodes/InputActionNode.h>
#include <zero/behavior/nodes/MapNode.h>
#include <zero/behavior/nodes/MathNode.h>
#include <zero/behavior/nodes/MoveNode.h>
#include <zero/behavior/nodes/PlayerNode.h>
#include <zero/behavior/nodes/RenderNode.h>
#include <zero/behavior/nodes/ShipNode.h>
#include <zero/zones/teamversus/TeamVersus.h>
#include <zero/zones/teamversus/nodes/CombatMath.h>
#include <zero/zones/teamversus/nodes/DecoyDeceptionNode.h>
#include <zero/zones/teamversus/nodes/DriftCombatNode.h>
#include <zero/zones/teamversus/nodes/EngagementPhaseNode.h>
#include <zero/zones/teamversus/nodes/EvasiveManeuverNode.h>
#include <zero/zones/teamversus/nodes/FireCadenceNode.h>
#include <zero/zones/teamversus/nodes/InterceptAimNode.h>
#include <zero/zones/teamversus/nodes/ItemCooldownNode.h>
#include <zero/zones/teamversus/nodes/MatchStateNode.h>
#include <zero/zones/teamversus/nodes/MineLayNode.h>
#include <zero/zones/teamversus/nodes/PortalEscapeNode.h>
#include <zero/zones/teamversus/nodes/ReadyShotNode.h>
#include <zero/zones/teamversus/nodes/RepelDecisionNode.h>
#include <zero/zones/teamversus/nodes/ShotClearanceNode.h>
#include <zero/zones/teamversus/nodes/TargetSelectNode.h>
#include <zero/zones/teamversus/nodes/TeamAdvantageNode.h>
#include <zero/zones/teamversus/nodes/TeamSpacingNode.h>
#include <zero/zones/teamversus/nodes/TerrainAvoidNode.h>
#include <zero/zones/teamversus/nodes/ThreatAssessmentNode.h>

namespace zero {
namespace teamversus {

void TeamVersusBehavior::OnInitialize(behavior::ExecuteContext& ctx) {
  // Deliberately does not set "request_ship".
  //
  // The zone is what puts a bot into a knockout match, and it decides the ship. Seeding a default
  // here would have the bot immediately ask to swap out of whatever it was assigned, fighting the
  // very system that started the match. ZoneController still sets the key from config
  // (TeamVersus:RequestShip, or --ship) when an operator explicitly wants a fixed ship, and the
  // tree's ship-request branch harmlessly does nothing whenever the key is absent.
}

// =================================================================================================
// Shape of this tree
//
// Four stages run in order every tick, and the split between them is the main structural idea:
//
//   1. Lifecycle   - are we spectating, is the match live.
//   2. Perception  - one pass that writes every derived fact to the blackboard.
//   3. Reflexes    - defensive item usage, which must be allowed to happen no matter what else is
//                    going on.
//   4. Act         - movement and firing, running in parallel.
//
// Perception being a single pass matters more than it looks. The obvious alternative - each branch
// deriving what it needs where it needs it - has two failure modes that are hard to see and easy to
// hit. Nodes that carry tick-to-tick memory get run several times per tick against different
// targets and end up differencing samples that belong to different players; and independent
// branches testing overlapping conditions eventually disagree with each other about what situation
// the bot is in. Deriving everything once, up front, in a fixed order, removes both by
// construction.
//
// Posture (EngagementPhaseNode) is the other structural idea. Rather than each branch deciding for
// itself whether to advance or back off, one node classifies the situation into Press / Poke /
// Recover / Regroup and publishes both the posture and the standoff distance it implies. Every
// branch downstream reads that instead of re-deriving it.
//
// -------------------------------------------------------------------------------------------------
// Where the numbers come from
//
// The constants below are taken from measurements over 15 real 4v4 league matches (roughly 46,000
// bullets, 10,700 bombs, 758 repels, 187 kills), not from intuition. The findings that shaped the
// design most:
//
//  - Fights are decided by local head-count, not range. The damage-exchange ratio at 10-14 tiles
//    runs 0.58 / 1.11 / 2.33 / 3.09 at a local advantage of -2 / -1 / 0 / +1. Range moves that
//    number far less than one extra body does.
//  - People shoot from a long way out. Median bullet-firing range is 29 tiles, at a hit rate under
//    10%, while kills actually land at a median 11 tiles after a deliberate dive from 32 tiles
//    three seconds earlier. Constant long-range pressure plus occasional commitment - not one or
//    the other.
//  - Isolation is what kills. The median victim was 41 tiles from their nearest teammate two full
//    seconds before dying, against a 27 tile baseline for everyone alive.
//  - Energy is spent carefully. Bullets go out at a median 90% energy (p10 60%), bombs at 70%,
//    and repels at 30% - each item has a distinct and consistent context of use.
//  - Movement is an orbit, not a chase. Heading sits 75-95 degrees off the direction of travel,
//    thrust is held 84-93% of the time, and the hull turns about 1.5x faster than the velocity
//    vector does.
//
// Per-node comments carry the specific numbers behind each decision.
// =================================================================================================

std::unique_ptr<behavior::BehaviorNode> TeamVersusBehavior::CreateTree(behavior::ExecuteContext& ctx) {
  using namespace behavior;

  BehaviorBuilder builder;

  // --- perception ---------------------------------------------------------------------------
  // Radius the exchange-ratio table was measured at. Widening it would count teammates too far away
  // to affect the trade within the second the exchange is measured over.
  constexpr float kLocalRadius = 25.0f;
  // How far out to look for incoming fire. Comfortably past the range a bullet covers during the
  // time it takes us to react and turn.
  constexpr float kThreatScanDistance = 22.0f;

  // --- formation ----------------------------------------------------------------------------
  // Roughly the bomb blast radius, so teammates stop sharing every bomb aimed at either of them.
  constexpr float kMinTeamSpacing = 11.0f;
  // A little above the measured median spacing of 26 tiles.
  constexpr float kMaxTeamSpacing = 32.0f;
  // (The distance at which being cut off makes walking home the priority lives on
  // EngagementPhaseNode as regroup_support_distance, since posture owns that decision.)

  // --- approach -----------------------------------------------------------------------------
  // Past this we path toward the target instead of steering at it. Comfortably outside the widest
  // standoff posture (Recover, 42 tiles) plus the pump's swing, so the orbit is never interrupted by
  // the approach branch trying to reclaim it at the outer edge of a normal in-out cycle.
  constexpr float kApproachRange = 50.0f;

  // --- terrain ------------------------------------------------------------------------------
  // Small on purpose: this should fire when a wall is actually in the way, not merely nearby.
  // Walls bounce rather than stop, so proximity alone is not a problem.
  constexpr float kWallDistance = 4.5f;
  constexpr float kWallOpeningDistance = 35.0f;

  // --- firing -------------------------------------------------------------------------------
  // Humans essentially never fire bullets below 60% energy; p10 of firing energy is exactly 0.6.
  // Pressing bypasses this, because committing to a kill is the one time spending down to nothing
  // is correct.
  constexpr float kBulletEnergyFloor = 0.55f;
  // Bombs cost 300 against a 1700 tank - 17.6% of full health per shot - and measured bomb usage
  // sits at a median 70% energy.
  constexpr float kBombEnergyFloor = 0.62f;
  // Bomb range window. The p25/p75 of real bomb-firing range is 21-34 tiles; below the minimum the
  // blast catches us, and past the maximum a two-and-a-half second flight is pure hope.
  constexpr float kBombMinRange = 14.0f;
  constexpr float kBombMaxRange = 40.0f;
  // Bullets need a real hit, so the tolerance around the target hull is tight. Bombs deliver blast
  // damage on a near miss and are used as area denial, so theirs is deliberately loose.
  constexpr float kBulletHitTolerance = 1.6f;
  constexpr float kBombHitTolerance = 5.0f;
  // Target selection has no range limit any more - it picks who we should be fighting, not who we
  // can hit - so the firing block carries its own. A bullet expires at 68 tiles (BulletAliveTime 550
  // against a 12.5 tile/sec muzzle speed), and the corpus's own hit rate is at the noise floor well
  // before that; this sits above the p75 human firing range so it costs no real volume.
  constexpr float kBulletMaxRange = 45.0f;
  // Longest world-frame flight each weapon will accept - how much warning we are willing to give the
  // target. This is what makes our own velocity part of the fire decision: a shot leaves at our
  // velocity plus the muzzle velocity, so standing still means 12.5 tiles/sec, slower than the ship
  // it is chasing, and at 30 tiles that is a 2.4 second flight nobody sits still for. Bullets are
  // held tight because they need a real hit; bombs and thors are looser because a late bomb still
  // denies the ground it lands on.
  constexpr float kBulletMaxFlightTime = 1.5f;
  constexpr float kBombMaxFlightTime = 2.2f;
  constexpr float kThorMaxFlightTime = 2.2f;
  // Thors pass through walls, which is the entire reason to spend one: a target we cannot otherwise
  // reach. Rare in real play (150 uses against 46,000 bullets), used at a median 25 tiles.
  constexpr float kThorMaxRange = 30.0f;
  constexpr float kThorTargetEnergyPercent = 0.4f;
  constexpr float kThorHitTolerance = 4.0f;

  // --- defense ------------------------------------------------------------------------------
  // Give up aim for a dodge only when the incoming volley would actually kill us. Below that the
  // evasive node still nudges, but keeps the nose on target - damage taken per sample is lowest
  // nose-on (5.37) and highest broadside (8.50), so turning away is a trade, not free protection.
  constexpr float kLethalFraction = 1.0f;

  // --- items --------------------------------------------------------------------------------
  // One second between two presses of the same item key. Long enough for the item's effect to
  // actually happen and for the next perception pass to see the result, which is the whole point -
  // the alternative is re-deciding an unchanged situation every tick and emptying the item.
  constexpr u32 kItemDebounceTicks = 100;
  // Decoys get a much longer one on top of that. The ghost persists for a while and a second one
  // does not make the first more convincing, so this is an anti-stacking interval rather than a
  // debounce.
  constexpr u32 kDecoyCooldownTicks = 900;

  // Antiwarp is never up below 75% energy, because it suppresses our own recharge the entire time it
  // is running. Raise a little above the floor so recharge crossing the line does not toggle it.
  constexpr float kAntiwarpDropEnergy = 0.75f;
  constexpr float kAntiwarpRaiseEnergy = 0.80f;

  // Safety net if the "GO!" match-start message is ever missed. Generous, because reacting late is
  // far better than a bot that starts shooting during a ready check.
  constexpr u32 kMatchSafetyNetTicks = 3000;

  // clang-format off
  // The root is a Parallel, which is the only composite that genuinely runs every child every tick.
  //
  // A Selector would stop at the lifecycle node, since that always succeeds. A Sequence looks
  // correct but is not: SequenceNode remembers the index of any child that returns Running and
  // resumes there on the following tick, skipping everything before it. ShipRequestNode returns
  // Running while a ship swap is pending, so a Sequence root would stop updating the match
  // lifecycle for exactly the window between leaving spec and being in the right ship - which is
  // precisely when the lifecycle needs to arm its safety net.
  builder
    .Parallel()
      // =======================================================================================
      // 1. Lifecycle. Runs every tick, unconditionally.
      // =======================================================================================
      .Child<MatchStateNode>(kMatchSafetyNetTicks)

      // Ask for the configured ship, if one was configured at all.
      //
      // This sits in the root Parallel rather than in the mode Selector below, and that placement is
      // load-bearing. ShipRequestNode reports Running for as long as the ship does not match, so as
      // a sibling *before* the combat branch it would make a Selector return Running every tick and
      // silently suppress perception, reflexes and movement entirely. In an arena that refuses the
      // request - locked ships, a match system that assigns them, a typo in config - that is not a
      // brief pause, it is a bot that never fights again for the rest of the round.
      //
      // Success-decorated and parallel to everything else, the request becomes advisory: the bot
      // asks periodically (the node rate-limits itself internally) and fights with whatever it
      // actually has in the meantime. Harmlessly does nothing when no ship was configured.
      .Sequence(CompositeDecorator::Success)
          .InvertChild<BlackboardSetQueryNode>("spectating")
          .InvertChild<ShipQueryNode>("request_ship")
          .Child<ShipRequestNode>("request_ship")
          .End()

      .Selector()
        // In spec we do nothing at all except (optionally) ask to be queued. A spectator has no
        // ship to steer and no shots to take, and the zone is responsible for putting us in.
        .Sequence()
            .Child<BlackboardSetQueryNode>("spectating")
            .Sequence(CompositeDecorator::Success)
                .Child<QueueCommandNode>()
                .End()
            .End()

        // =====================================================================================
        // In a ship. Stages 2-4 all run, in order.
        // =====================================================================================
        .Sequence()

        // =====================================================================================
        // 2. Perception. One pass, fixed order, everything below reads its output.
        // =====================================================================================
        .Sequence(CompositeDecorator::Success)
            // Bombs, thors, mines, decoys and portals all share one cooldown slot, and a tick that
            // presses two of those keys gets one of them silently dropped by the game. The reflex
            // stage below claims the slot when it spends an item; this clears last tick's claim.
            .Child<BlackboardEraseNode>("bomb_slot_used")
            .Child<PlayerPositionQueryNode>("self_position")
            .Child<PlayerCurrentEnergyQueryNode>("self_energy")
            .Child<TeamAdvantageNode>(kLocalRadius)

            .Sequence(CompositeDecorator::Success)  // Having no reachable enemy is a normal state, not a failure.
                .Child<TargetSelectNode>("target")
                .Child<InterceptAimNode>(WeaponType::Bullet, "target", "aimshot", "bullet_predicted",
                                         "bullet_flight_time")
                // The bomb solution is computed every tick even though bombs fire at most every 1.5
                // seconds. It has to be: the aim node estimates the target's turn rate by
                // differencing consecutive samples and discards anything older than half a second,
                // so a node only executed at the moment of firing would find its own history stale
                // every single time and silently fall back to straight-line extrapolation.
                //
                // It writes to its own keys for the same reason. Bombs are slightly slower than
                // bullets once recoil is accounted for, so the two weapons genuinely disagree about
                // where the target will be, and sharing one key would leave whichever ran last
                // deciding both.
                .Child<InterceptAimNode>(WeaponType::Bomb, "target", "bomb_aimshot", "bomb_predicted",
                                         "bomb_flight_time")
                .End()

            .Child<EngagementPhaseNode>()
            // Success-wrapped because ThreatAssessmentNode reports Failure whenever nothing is
            // inbound - which is most ticks. It happens to be last in this sequence today, so a
            // bare Failure would be harmless, but a Sequence stops dead at its first failing
            // child and "last" is not a property that survives someone appending a node.
            .SuccessChild<ThreatAssessmentNode>(kThreatScanDistance)
            .End()

        // =====================================================================================
        // 3. Reflexes. Defensive items, evaluated regardless of what movement is doing.
        //    Success-decorated so spending (or not spending) an item never decides whether the
        //    bot moves and shoots this tick.
        // =====================================================================================
        //    Every item press is debounced by an ItemCooldownNode sitting immediately before its
        //    InputActionNode. None of these items produce an observable effect on the tick they are
        //    used - a repel takes time to push shots clear, a decoy takes time to be mis-targeted, a
        //    warp takes time to move us and for perception to see it - while the condition that
        //    triggered them is still true on the next tick and the tick after. Without a debounce
        //    the tree re-decides the same unchanged situation at 100Hz and empties the item down to
        //    zero against a single incoming bomb. The cooldown node claims its timer on success, so
        //    it must stay the last gate before the key press.
        .Sequence(CompositeDecorator::Success)
            .Selector()
                // --- dire: something inbound is going to kill us --------------------------------
                // Escalation order is portal, then decoy, then repel. Repel is last not because it
                // is the weakest - it is the only one of the three that actually stops the damage
                // and leaves us in position - but because it is the scarcest and the most
                // universally applicable. Anything the other two can solve should be solved by them,
                // so the repel is still there for the case nothing else covers.
                .Sequence()
                    .Child<BlackboardSetQueryNode>("threat_lethal")
                    .Selector()
                        // Instant and total: removes us from the fight rather than surviving it.
                        // The node checks the marker still exists and that the far end is an
                        // improvement, so this fails harmlessly when there is nowhere good to go.
                        .Sequence()
                            .Child<PortalEscapeNode>()
                            .Child<ItemCooldownNode>("warp", kItemDebounceTicks)
                            .Child<WarpNode>()
                            .Child<ScalarNode>(1.0f, "bomb_slot_used")
                            .End()
                        // Can't leave: make the next volley go somewhere else instead.
                        .Sequence()
                            .Child<DecoyDeceptionNode>()
                            .Child<ItemCooldownNode>("decoy", kDecoyCooldownTicks)
                            .Child<InputActionNode>(InputAction::Decoy)
                            .Child<ScalarNode>(1.0f, "bomb_slot_used")
                            .End()
                        // Last resort. RepelDecisionNode additionally requires that the damage is
                        // no longer dodgeable, so this does not fire on a lethal volley we still
                        // have time to thrust out of.
                        .Sequence()
                            .Child<RepelDecisionNode>()
                            .Child<ItemCooldownNode>("repel", kItemDebounceTicks)
                            .Child<InputActionNode>(InputAction::Repel)
                            .Child<ScalarNode>(1.0f, "bomb_slot_used")
                            .End()
                        .End()
                    .End()

                // --- housekeeping: nothing is currently trying to kill us -----------------------
                .Sequence()  // Lay a marker while things are calm, so one exists when they aren't.
                    .Child<PortalLayNode>()
                    .Child<ItemCooldownNode>("portal", kItemDebounceTicks)
                    .Child<InputActionNode>(InputAction::Portal)
                    .Child<ScalarNode>(1.0f, "bomb_slot_used")
                    .End()
                .Sequence()  // Mine the ground behind us while withdrawing from a pursuer.
                    .Child<MineLayNode>()
                    .Child<ItemCooldownNode>("mine", kItemDebounceTicks)
                    .Child<InputActionNode>(InputAction::Mine)
                    .Child<ScalarNode>(1.0f, "bomb_slot_used")
                    .End()
                .End()
            .End()

        // Antiwarp denies the portal escape to anyone caught inside it, which is worth exactly as
        // much as the kill we are trying to close out - so it goes on while pressing and comes off
        // the moment we can't spare it.
        //
        // "Can't spare it" is about recharge, not about the field. Antiwarp suppresses our own
        // energy recharge for as long as it is up, which makes it the one item whose cost is paid
        // continuously and invisibly: a bot that leaves it on is not merely wasting something, it is
        // fighting the whole engagement with its regeneration switched off, and it will lose trades
        // it should win without anything obviously going wrong. So it is only ever up while we are
        // comfortably healthy, and drops the instant we are not.
        .Selector(CompositeDecorator::Success)
            .Sequence()
                .Child<ShipCapabilityQueryNode>(ShipCapability_Antiwarp)
                .Child<BlackboardSetQueryNode>("phase_press")
                // Turn-on sits above the turn-off floor rather than on it, so a bot hovering at the
                // threshold does not toggle every time recharge crosses the line.
                .Child<PlayerEnergyPercentThresholdNode>(kAntiwarpRaiseEnergy)
                .InvertChild<PlayerStatusQueryNode>(Status_Antiwarp)
                .Child<ItemCooldownNode>("antiwarp", kItemDebounceTicks)
                .Child<InputActionNode>(InputAction::Antiwarp)
                .End()
            .Sequence()
                .Child<ShipCapabilityQueryNode>(ShipCapability_Antiwarp)
                .Child<PlayerStatusQueryNode>(Status_Antiwarp)
                .Selector()
                    .InvertChild<BlackboardSetQueryNode>("phase_press")
                    .InvertChild<PlayerEnergyPercentThresholdNode>(kAntiwarpDropEnergy)
                    .End()
                // Shares the "antiwarp" timer with the branch above, so an on-toggle and an
                // off-toggle cannot land inside the same window. Status arrives from the server a
                // round trip late, so without this the bot toggles again before it can see that the
                // first toggle took, and ends up flickering the field on and off.
                .Child<ItemCooldownNode>("antiwarp", kItemDebounceTicks)
                .Child<InputActionNode>(InputAction::Antiwarp)
                .End()
            .End()

        // =====================================================================================
        // 4. Act.
        // =====================================================================================
        .Selector()
            // --- before the match actually starts -------------------------------------------
            // Hold position, keep formation, stay off the walls - and fire exactly one bullet to
            // signal ready. That shot is part of the match protocol, not combat: the zone waits
            // for every player to confirm before it announces "GO!", so withholding it stalls the
            // match indefinitely. ReadyShotNode owns the "did it actually leave the ship"
            // bookkeeping; everything else stays quiet, since opening real fire during a ready
            // check is both wrong and conspicuous.
            .Sequence()
                .InvertChild<BlackboardSetQueryNode>("match_live")
                .Sequence(CompositeDecorator::Success)
                    .Child<ReadyShotNode>()
                    .Child<InputActionNode>(InputAction::Bullet)
                    .End()
                .Sequence(CompositeDecorator::Success)
                    .Child<TeamSpacingNode>(kMinTeamSpacing, kMaxTeamSpacing)
                    .End()
                .Selector(CompositeDecorator::Success)
                    .Child<TerrainAvoidNode>(kWallDistance, kWallOpeningDistance)
                    .Child<SeekZeroNode>()
                    .End()
                .End()

            // --- nobody to fight -----------------------------------------------------------
            // Since target selection has no range limit, reaching here means there is genuinely no
            // enemy we know anything about - all dead, in safe, in an unreachable region, or never
            // yet seen. Not "none in range": that used to land here too, and because the only thing
            // this branch did was stop dead, both teams would sit motionless for the whole round
            // waiting for someone else to close the distance.
            //
            // Close on the team rather than idling apart, since whatever happens next happens at a
            // head-count advantage for whoever is together when it does. The gate is formation
            // spacing rather than the regroup distance, because there is no fight to be given up by
            // tightening up now.
            .Sequence()
                .InvertChild<BlackboardSetQueryNode>("target")
                .Selector(CompositeDecorator::Success)
                    .Sequence()
                        .Child<BlackboardSetQueryNode>("nearest_teammate_position")
                        .Child<DistanceThresholdNode>("nearest_teammate_position", kMaxTeamSpacing)
                        .Child<GoToNode>("nearest_teammate_position")
                        .Child<RenderPathNode>(Vector3f(0.0f, 1.0f, 0.5f))
                        .End()
                    .Child<SeekZeroNode>()
                    .End()
                .End()

            // --- fighting ------------------------------------------------------------------
            // Movement and firing run in parallel rather than in sequence, because they are
            // genuinely independent: which way we thrust does not decide whether a shot is on, and
            // a shot being off should never stop us from moving.
            .Parallel()

                // ---- movement: a Selector, so exactly one node has primary authority --------
                //
                // Ordered as: survive, regroup, approach, then fight. The split that matters is
                // between the two *pathed* branches and the two *steered* ones below them. Steering
                // is a force toward where we want to be and knows nothing about the map; on a walled
                // arena that is only usable once we are already in the same pocket as the target.
                // Anything further than that has to be pathed, or the bot presses itself into
                // whatever wall happens to lie between it and the enemy.
                .Selector()
                    // Returns Success only when the incoming volley is actually lethal, in which
                    // case it takes the tick (and handles its own wall avoidance). Otherwise it
                    // fails, having already blended in a speed-changing nudge that keeps the nose on
                    // target - so the branches below still run and the dodge comes for free.
                    .Child<EvasiveManeuverNode>(kLethalFraction)

                    // Cut off from the team. Walk home; the firing block keeps working on the way.
                    .Sequence()
                        .Child<BlackboardSetQueryNode>("phase_regroup")
                        .Child<BlackboardSetQueryNode>("nearest_teammate_position")
                        .Child<GoToNode>("nearest_teammate_position")
                        .Child<RenderPathNode>(Vector3f(1.0f, 0.5f, 0.0f))
                        .End()

                    // Approach. Either the target is too far to fight from here, or terrain is in
                    // the way - and both are answered the same way, by pathing rather than steering.
                    //
                    // The line-of-sight test is what makes this self-limiting: it is a plain cast
                    // from us to the target, so the moment the path brings them into view this
                    // branch stops claiming the tick and the orbit below takes over. There is no
                    // separate "stop approaching" rule to get wrong.
                    .Sequence()
                        .Selector()
                            .Child<DistanceThresholdNode>("target_position", kApproachRange)
                            .InvertChild<VisibilityQueryNode>("target_position")
                            .End()
                        .Child<GoToNode>("target_position")
                        .Child<RenderPathNode>(Vector3f(0.0f, 0.5f, 1.0f))
                        .End()

                    // From here down movement is a raw steering force, so terrain has to be handled
                    // explicitly. This sits *below* the pathed branches on purpose: the pathfinder
                    // already routes around walls, and an open-direction shove layered on top of it
                    // fights the route it is following. Blending is no better - an avoidance force
                    // added to a force pushing into a corner is just a smaller push into the corner
                    // - so once terrain is genuinely in the way here, it wins the tick outright.
                    .Child<TerrainAvoidNode>(kWallDistance, kWallOpeningDistance, "target_position")

                    // The normal case: orbit at the standoff posture asked for, pumping in and out.
                    .Child<DriftCombatNode>("aimshot", "target_position", "standoff_distance")
                    .End()

                // ---- formation: a light continuous correction on top of whatever won above ---
                // Force magnitudes here are two to three orders of magnitude below a terrain escape
                // or a lethal break, so this leans without ever overriding.
                .Sequence(CompositeDecorator::Success)
                    .Child<TeamSpacingNode>(kMinTeamSpacing, kMaxTeamSpacing)
                    .End()

                // ---- firing -----------------------------------------------------------------
                .Sequence(CompositeDecorator::Success)
                    .Child<BlackboardSetQueryNode>("match_live")
                    .InvertChild<TileQueryNode>(kTileIdSafe)

                    .Selector(CompositeDecorator::Success)
                        // Thor: the only weapon that passes through walls, so it is worth spending
                        // exactly when nothing else can reach - a weakened target behind terrain.
                        // ShotClearanceNode skips the line-of-sight test for thors precisely
                        // because giving them one would veto every shot worth taking.
                        .Sequence()
                            .InvertChild<BlackboardSetQueryNode>("bomb_slot_used")
                            .Child<ShipWeaponCapabilityQueryNode>(WeaponType::Thor)
                            .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Thor)
                            .Child<PlayerEnergyPercentThresholdNode>(kBombEnergyFloor)
                            .InvertChild<DistanceThresholdNode>("target_position", kThorMaxRange)
                            .InvertChild<VisibilityQueryNode>("target_position")
                            .InvertChild<ScalarThresholdNode<float>>("target_energy_percent", kThorTargetEnergyPercent)
                            .Child<ShotClearanceNode>(WeaponType::Thor, "bomb_predicted", kThorHitTolerance, kThorMaxFlightTime)
                            .Child<InputActionNode>(InputAction::Thor)
                            .End()

                        // Bomb: area denial at mid range. The clearance node handles the two things
                        // that make bombs different from bullets - the fuse means terrain in the
                        // last few tiles cannot stop it, and the blast means a friendly near the
                        // detonation point matters while a friendly in the flight path does not.
                        .Sequence()
                            .InvertChild<BlackboardSetQueryNode>("bomb_slot_used")
                            .InvertChild<InputQueryNode>(InputAction::Thor)
                            .Child<ShipWeaponCapabilityQueryNode>(WeaponType::Bomb)
                            .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bomb)
                            .Child<PlayerEnergyPercentThresholdNode>(kBombEnergyFloor)
                            .Child<DistanceThresholdNode>("target_position", kBombMinRange)
                            .InvertChild<DistanceThresholdNode>("target_position", kBombMaxRange)
                            .Child<ShotClearanceNode>(WeaponType::Bomb, "bomb_predicted", kBombHitTolerance, kBombMaxFlightTime)
                            .Child<InputActionNode>(InputAction::Bomb)
                            .End()

                        // Bullets: the bread and butter, fired continuously at long range.
                        .Sequence()
                            .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bullet)
                            .InvertChild<InputQueryNode>(InputAction::Bomb)  // Never both in one tick.
                            .InvertChild<InputQueryNode>(InputAction::Thor)
                            .InvertChild<DistanceThresholdNode>("target_position", kBulletMaxRange)
                            .Selector()  // Energy discipline, waived while committing to a kill.
                                .Child<BlackboardSetQueryNode>("phase_press")
                                .Child<PlayerEnergyPercentThresholdNode>(kBulletEnergyFloor)
                                .End()
                            .Child<ShotClearanceNode>(WeaponType::Bullet, "bullet_predicted", kBulletHitTolerance,
                                                     kBulletMaxFlightTime)
                            .Child<FireCadenceNode>()  // Human trigger rhythm - runs last, so it only
                                                       // consumes its budget on shots we actually take.
                            .Child<InputActionNode>(InputAction::Bullet)
                            .End()
                        .End()
                    .End()
                .End()   // Parallel (fighting)
            .End()       // Selector (act)
          .End()         // Sequence (in a ship, in a match)
        .End()           // Selector (mode)
      .End();            // Parallel (root)
  // clang-format on

  return builder.Build();
}

}  // namespace teamversus
}  // namespace zero
