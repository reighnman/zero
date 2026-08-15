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
#include <zero/zones/teamversus/nodes/MatchStateNode.h>
#include <zero/zones/teamversus/nodes/MineLayNode.h>
#include <zero/zones/teamversus/nodes/PortalEscapeNode.h>
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
  // Far enough out that walking home is genuinely better than fighting where we are.
  constexpr float kRegroupDistance = 46.0f;

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
        .Sequence(CompositeDecorator::Success)
            .Selector()
                .Sequence()  // Repel first: it stops the damage AND leaves us in position.
                    .Child<RepelDecisionNode>()
                    .Child<InputActionNode>(InputAction::Repel)
                    .End()
                .Sequence()  // Portal is the fallback once repels are gone - instant, but one-way.
                    .Child<BlackboardSetQueryNode>("threat_lethal")
                    .Child<PortalEscapeNode>()
                    .Child<WarpNode>()
                    .End()
                .Sequence()  // Lay a marker while things are calm, so one exists when they aren't.
                    .Child<PortalLayNode>()
                    .Child<InputActionNode>(InputAction::Portal)
                    .Child<ScalarNode>(1.0f, "bomb_slot_used")
                    .End()
                .Sequence()  // Decoys are a long-range misdirect, not a panic button - see the node.
                    .Child<DecoyDeceptionNode>()
                    .Child<InputActionNode>(InputAction::Decoy)
                    .Child<ScalarNode>(1.0f, "bomb_slot_used")
                    .End()
                .Sequence()  // Mine the ground behind us while withdrawing from a pursuer.
                    .Child<MineLayNode>()
                    .Child<InputActionNode>(InputAction::Mine)
                    .Child<ScalarNode>(1.0f, "bomb_slot_used")
                    .End()
                .End()
            .End()

        // Antiwarp denies the portal escape to anyone inside it, which is worth exactly as much as
        // the kill we are trying to close out - so it goes on while pressing and comes off the
        // moment we can't spare the drain.
        .Selector(CompositeDecorator::Success)
            .Sequence()
                .Child<ShipCapabilityQueryNode>(ShipCapability_Antiwarp)
                .Child<BlackboardSetQueryNode>("phase_press")
                .Child<PlayerEnergyPercentThresholdNode>(0.7f)
                .InvertChild<PlayerStatusQueryNode>(Status_Antiwarp)
                .Child<InputActionNode>(InputAction::Antiwarp)
                .End()
            .Sequence()
                .Child<ShipCapabilityQueryNode>(ShipCapability_Antiwarp)
                .Child<PlayerStatusQueryNode>(Status_Antiwarp)
                .Selector()
                    .InvertChild<BlackboardSetQueryNode>("phase_press")
                    .InvertChild<PlayerEnergyPercentThresholdNode>(0.6f)
                    .End()
                .Child<InputActionNode>(InputAction::Antiwarp)
                .End()
            .End()

        // =====================================================================================
        // 4. Act.
        // =====================================================================================
        .Selector()
            // --- before the match actually starts -------------------------------------------
            // Keep formation and stay off the walls, but do not shoot and do not engage. The zone
            // takes us out of spec some unknown time before "GO!", and opening fire during a ready
            // check is both wrong and conspicuous.
            .Sequence()
                .InvertChild<BlackboardSetQueryNode>("match_live")
                .Sequence(CompositeDecorator::Success)
                    .Child<TeamSpacingNode>(kMinTeamSpacing, kMaxTeamSpacing)
                    .End()
                .Selector(CompositeDecorator::Success)
                    .Child<TerrainAvoidNode>(kWallDistance, kWallOpeningDistance)
                    .Child<SeekZeroNode>()
                    .End()
                .End()

            // --- nobody to fight -----------------------------------------------------------
            // Either everyone is dead, unreachable, or sitting in safe. Close on the team rather
            // than idling apart, since whatever happens next will happen at a head-count advantage
            // for whoever is together when it does.
            .Sequence()
                .InvertChild<BlackboardSetQueryNode>("target")
                .Selector(CompositeDecorator::Success)
                    .Sequence()
                        .Child<BlackboardSetQueryNode>("nearest_teammate_position")
                        .Child<DistanceThresholdNode>("nearest_teammate_position", kRegroupDistance)
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
                .Selector()
                    // Terrain wins outright. Blending avoidance into a force that is actively
                    // pushing into a corner only produces a smaller push into the corner.
                    .Child<TerrainAvoidNode>(kWallDistance, kWallOpeningDistance, "target_position")

                    // Returns Success only when the incoming volley is actually lethal, in which
                    // case it takes the tick. Otherwise it fails, having already blended in a
                    // speed-changing nudge that keeps the nose on target - so the branches below
                    // still run and the dodge comes for free.
                    .Child<EvasiveManeuverNode>(kLethalFraction)

                    // Cut off from the team. Walk home; the firing block keeps working on the way.
                    .Sequence()
                        .Child<BlackboardSetQueryNode>("phase_regroup")
                        .Child<BlackboardSetQueryNode>("nearest_teammate_position")
                        .Child<GoToNode>("nearest_teammate_position")
                        .Child<RenderPathNode>(Vector3f(1.0f, 0.5f, 0.0f))
                        .End()

                    // Target is behind terrain. Path to them rather than orbiting a wall.
                    .Sequence()
                        .InvertChild<VisibilityQueryNode>("target_position")
                        .Child<GoToNode>("target_position")
                        .Child<RenderPathNode>(Vector3f(0.0f, 0.5f, 1.0f))
                        .End()

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
                            .Child<ShotClearanceNode>(WeaponType::Thor, "bomb_predicted", kThorHitTolerance)
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
                            .Child<ShotClearanceNode>(WeaponType::Bomb, "bomb_predicted", kBombHitTolerance)
                            .Child<InputActionNode>(InputAction::Bomb)
                            .End()

                        // Bullets: the bread and butter, fired continuously at long range.
                        .Sequence()
                            .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bullet)
                            .InvertChild<InputQueryNode>(InputAction::Bomb)  // Never both in one tick.
                            .InvertChild<InputQueryNode>(InputAction::Thor)
                            .Selector()  // Energy discipline, waived while committing to a kill.
                                .Child<BlackboardSetQueryNode>("phase_press")
                                .Child<PlayerEnergyPercentThresholdNode>(kBulletEnergyFloor)
                                .End()
                            .Child<ShotClearanceNode>(WeaponType::Bullet, "bullet_predicted", kBulletHitTolerance)
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
