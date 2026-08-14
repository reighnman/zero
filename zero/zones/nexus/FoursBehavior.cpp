#include <zero/behavior/BehaviorBuilder.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/behavior/nodes/AimNode.h>
#include <zero/behavior/nodes/AttachNode.h>
#include <zero/behavior/nodes/BlackboardNode.h>
#include <zero/behavior/nodes/InputActionNode.h>
#include <zero/behavior/nodes/ChatNode.h>
#include <zero/behavior/nodes/MapNode.h>
#include <zero/behavior/nodes/MathNode.h>
#include <zero/behavior/nodes/MoveNode.h>
#include <zero/behavior/nodes/PlayerNode.h>
#include <zero/behavior/nodes/PowerballNode.h>
#include <zero/behavior/nodes/RegionNode.h>
#include <zero/behavior/nodes/RenderNode.h>
#include <zero/behavior/nodes/ShipNode.h>
#include <zero/behavior/nodes/ThreatNode.h>
#include <zero/behavior/nodes/TimerNode.h>
#include <zero/behavior/nodes/WaypointNode.h>
#include <zero/zones/svs/nodes/BurstAreaQueryNode.h>
#include <zero/zones/svs/nodes/DynamicPlayerBoundingBoxQueryNode.h>
#include <zero/zones/svs/nodes/FindNearestGreenNode.h>
#include <zero/zones/svs/nodes/IncomingDamageQueryNode.h>
#include <zero/zones/svs/nodes/MemoryTargetNode.h>
#include <zero/zones/svs/nodes/NearbyEnemyWeaponQueryNode.h>
#include <zero/zones/nexus/nodes/NearestTeammateNode.h>
#include <zero/zones/nexus/nodes/LowestTargetNode.h>
#include <zero/zones/nexus/nodes/FleeNode.h>
#include <zero/zones/nexus/nodes/FleeDistanceNode.h>
#include <zero/zones/nexus/nodes/OrbitNode.h>
#include <zero/zones/nexus/nodes/LocalAdvantageNode.h>
#include <zero/zones/nexus/nodes/EngagementRangeNode.h>
#include <zero/zones/nexus/nodes/BroadsideFaceNode.h>
#include <zero/zones/nexus/nodes/BombBlastSafetyNode.h>
#include <zero/zones/nexus/nodes/TeamCentroidNode.h>
#include <zero/zones/nexus/nodes/IncomingBlastDamageNode.h>
#include <zero/zones/nexus/nodes/RocketUsageNode.h>
#include <zero/zones/nexus/nodes/RushCommitmentNode.h>
#include <zero/zones/nexus/nodes/MineAvailableNode.h>
#include <zero/zones/nexus/nodes/EnemiesNearTargetNode.h>
#include <zero/zones/nexus/nodes/TeamFocusTargetNode.h>
#include <zero/zones/nexus/nodes/PursuedFromBehindNode.h>
#include <zero/zones/nexus/nodes/CruiseSpeedNode.h>
#include <zero/zones/nexus/nodes/TargetEnergyPercentThresholdNode.h>
#include <zero/zones/nexus/nodes/WallAvoidanceNode.h>
#include <zero/zones/nexus/nodes/DodgeIncomingDamage.h>
#include <zero/zones/nexus/nodes/DodgeJukeNode.h>
#include <zero/zones/nexus/nodes/EnergyDisadvantageNode.h>
#include <zero/zones/nexus/nodes/TargetEnergyDropNode.h>
#include <zero/zones/nexus/nodes/ShotLineOfSightNode.h>
#include <zero/zones/nexus/nodes/TargetOpeningRangeNode.h>
#include <zero/zones/nexus/nodes/FinishableTargetNode.h>
#include <zero/zones/trenchwars/nodes/AttachNode.h>
#include <zero/zones/nexus/nodes/PlayerByNameNode.h>
#include <zero/zones/nexus/nodes/PredictiveAimNode.h>
#include <zero/zones/nexus/nodes/TargetAccelerationNode.h>

#include <zero/zones/nexus/Nexus.h>
#include "FoursBehavior.h"


using namespace zero::svs;

namespace zero {
namespace nexus {

// TODO WIP
struct Placeholder : public behavior::BehaviorNode {
  Placeholder(const char* something) : something(something) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    float enter_delay = (ctx.bot->game->connection.settings.EnterDelay / 100.0f);
    Player* self = ctx.bot->game->player_manager.GetSelf();

    // Make sure we are in a ship and not dead.
    if (!self || self->ship == 8) return behavior::ExecuteResult::Success;
    if (self->enter_delay > 0.0f && self->enter_delay < enter_delay) return behavior::ExecuteResult::Success;

    auto opt_tw = ctx.blackboard.Value<Nexus*>("nex");
    float radius = ctx.bot->game->connection.settings.ShipSettings[self->ship].GetRadius();
    Nexus* nexus = *opt_tw;

    path::Path entrance_path = ctx.bot->bot_controller->pathfinder->FindPath(
        ctx.bot->game->GetMap(), self->position, nexus->entrance_position, radius, self->frequency);

    // if (entrance_path.GetRemainingDistance() < nearby_threshold) {
    //  return behavior::ExecuteResult::Success;
    // }
    return behavior::ExecuteResult::Success;
  }

  const char* something = nullptr;
};

std::unique_ptr<behavior::BehaviorNode> FoursBehavior::CreateTree(behavior::ExecuteContext& ctx) {
  using namespace behavior;

  BehaviorBuilder builder;

  const Vector2f center(512, 512);

  // Used for target prio
  constexpr float kLowEnergyThreshold = 800.0f;         // Energy threshold to prio targets
  constexpr float kLowEnergyDistanceThreshold = 20.0f;  // Distance threshold for prio targets

  // Don't dodge below this
  constexpr float kLowEnergyRushThreshold = 400.0f;  // Rush threshold
  // We will rush if someone is low energy within this range. Replay kills are landed off a
  // committed, accelerating dive that *starts* well outside knife range - median killer-to-victim
  // range runs 27t at 2s before the kill, 20t at 1s, 15.7t at 0.5s, 11.4t when it lands, with the
  // killer's speed climbing 14.7 -> 16.9 tiles/sec across that window. Only committing once already
  // inside 10 tiles means never reproducing that dive at all, since by then the kill has either
  // happened or the target has slipped away.
  constexpr float kRushDistanceThreshold = 20.0f;
  // A dive is judged once at commitment and then bounded, because every other condition on the rush
  // is an absolute test of the present instant and none of them can notice the fight decaying
  // underneath us. See RushCommitmentNode.
  //
  // Losing a net body since we committed is the abort. Not "outnumbered" - that is already checked
  // separately and absolutely - but the specific case of starting supported and arriving alone,
  // which is what a chase does to a formation: we leave at rush speed and our team does not.
  constexpr float kRushMaxAdvantageLoss = 1.0f;
  // ~2.5s. Long enough to cross the 20-tile rush range and land a kill, short enough that a target
  // outrunning us stops being chased before we are across the map from our own team.
  constexpr u32 kRushMaxTicks = 250;
  // ~2s of not re-arming. The head-count fluctuates as players drift through the radius, so without
  // a refractory period an abort lasts exactly one tick and we stall in place still deep in their half.
  constexpr u32 kRushAbortCooldownTicks = 200;
  constexpr u32 kRushRepelThreshold = 1;             // If we don't have this many reps dont rush targets
  // Only press a target we've spotted as low energy ourselves if we have enough energy left to
  // commit to closing the distance - otherwise we'd be diving in already weak.
  constexpr float kRushMinEnergyPercent = 0.5f;

  // Check for incoming damage within this range
  constexpr float kRepelDistance = 7.0f;

  // How much damage that is going towards an enemy before we start bombing. This is to limit the frequency of our
  // bombing so it overlaps bullets and is harder to dodge.
  constexpr float kBombRequiredDamageOverlap = 300.0f;

  // Extra clearance beyond the bomb's actual blast radius before we're willing to fire one, since
  // both we and the teammate keep moving during the bomb's flight and the detonation point is only
  // ever an estimate. With a typical BombExplodePixels this lands the effective keep-out at roughly
  // the 12 tiles the old (single-friendly) check used, while now applying to every teammate and to
  // early detonations along the flight path rather than to one player at the target.
  //
  // Humans are far more conservative than even this: their 25th-percentile distance from the target
  // to their own nearest teammate when bombing is 24 tiles, against 17 for the bot. This margin is
  // deliberately not raised to match, because much of that human gap is just teammates being spread
  // out rather than a deliberate hold - what the corpus actually shows them doing is *discriminating
  // by weapon*, firing bombs into an occupied lane less often than bullets, which is the behavior
  // this gate restores.
  constexpr float kBombFriendlyBlastMargin = 6.0f;

  // Closing speed toward the target required before we'll fire a bomb. This used to be 2.0, which
  // silently made bombing almost impossible: while orbiting, our velocity is tangential, so the
  // component along the aim line sits near zero and the gate never opened. Together with a flat
  // 12-tile minimum range that exactly matched the 12-tile orbit distance, it left the bot with
  // almost no window in which it was allowed to bomb at all - in the 1-human-vs-7-bots replays the
  // human fired 71 bombs while the bot he isolated fired 2.
  //
  // The requirement existed so the bomb would carry our forward momentum, but PredictiveAimNode now
  // solves the lead in our own reference frame and accounts for ship velocity properly, so a
  // tangential launch is aimed correctly rather than drifting. All that's left worth excluding is
  // lobbing one while actively reversing away from the target. The 12-tile floor is dropped
  // outright: BombBlastSafetyNode already keeps us outside our own blast using the real
  // BombExplodePixels radius, which is what that number was standing in for.
  constexpr float kBombMinForwardVelocity = 0.0f;

  // --- Rockets ---
  // A rocket is a short burst of extra thrust and a raised speed cap. It only converts into real
  // distance if we're already near our normal top speed - lit from slow, most of the burn is spent
  // reaching a speed we'd have reached anyway, and while chasing it also risks sailing straight past
  // the target. So the chase gate below requires us to already be moving.
  //
  // There is only ONE use of a rocket now: chasing down a target we have already judged finishable.
  // The escape rocket is deliberately gone, and this is worth stating so it isn't re-added as an
  // obvious-looking improvement. Lighting a rocket to break contact was self-defeating in a way the
  // gate could not fix, because the problem was not WHEN it fired but what firing it did to us:
  //   - It doubled our speed pointed away from the fight, so by the time the burn ended we were far
  //     enough out that the trip back arrived alone and late - and isolation has predicted deaths in
  //     every recording of this branch so far.
  //   - It spent the item we needed for the offensive gate, so the escape and the kill competed for
  //     the same resource and the escape, being unconditional on target state, always won first.
  //   - It needed a hold block (the burn had to be seen through, since nothing is worth turning back
  //     into at that speed) whose whole job was to suppress the rest of the tree. That block was the
  //     suspected path by which a defensive burn leaked back into offense mid-flight, and deleting
  //     the burn deletes the leak rather than adding a fifth condition to catch it.
  // Breaking contact is FleeNode's job, and FleeDistanceNode already scales how far we go by how
  // badly we're hurt. Ordinary reverse thrust gets us there under control and with the item intact.
  constexpr float kRocketMinSpeedPercent = 0.8f;
  // Chasing: only worth it if there's a real gap to close. Raised from 12 after the first live test
  // came out visibly rocket-happy - at that range we were spending a limited item to cover ground
  // ordinary thrust would have covered, and then overshooting the target. This branch already sits
  // inside the rush sequence, so a rocket now means specifically "we are pressing a target we know
  // is weak, and it is far enough away to be getting off the hook".
  // A rocket is for finishing something already nearly dead, on its own, while we can afford the
  // dive - not for closing any gap that happens to be open. Live testing showed them going out far
  // too freely, so the gate is now four conjunctive conditions rather than distance and speed alone.
  //
  // Target energy is a percent of that ship's own max rather than an absolute, for the same reason
  // TargetEnergyPercentThresholdNode exists at all: a flat number means something different on
  // every ship and bounty.
  // Lowered 20% from 0.20 after deaths were still being traced to offensive rockets, despite the
  // gate already requiring the target isolated, us up bodies, and the target actively running.
  //
  // There is a systematic reason this gate fires more readily than its number suggests, and it
  // argues for erring low: `target_energy` is a HeuristicEnergyTracker *estimate*, not a reading,
  // and it is capped below the ship's true maximum. Dividing a capped estimate by the true maximum
  // biases target_percent DOWNWARD, so enemies read as weaker than they are and the threshold is
  // effectively looser than written. Tightening the constant compensates without pretending we can
  // fix the estimate.
  constexpr float kRocketTargetEnergyPercent = 0.16f;
  constexpr float kRocketMinSelfEnergyPercent = 0.6f;
  // "Isolated" means nobody of theirs within this radius. The count includes the target itself, so
  // the gate fires only when the count is below 2.
  // Widened from 20. "Isolated" has to mean isolated at the scale we're about to travel, and a
  // rocket covers 16-35 tiles - an enemy 22 tiles from the target was well inside the dive and was
  // being ignored.
  constexpr float kRocketIsolationRadius = 30.0f;
  constexpr float kRocketMaxEnemiesNearTarget = 2.0f;
  // The gate that distance and speed alone could never express: only rocket at something that is
  // actually running away from us. A target holding station in its own team looks identical to a
  // fleeing one under a distance check, and rocketing at it is precisely the dive-into-a-crowd
  // behavior seen in play - we arrive at speed, in a group, with no thrust left to turn around.
  constexpr float kRocketMinOpeningSpeed = 3.0f;
  // Rockets also require a positive local head-count, not merely a non-negative one. At parity the
  // exchange is roughly even and there's nothing a limited item buys; committing one only makes
  // sense when we're up bodies and a kill actually converts into an advantage.
  constexpr float kRocketMinAdvantage = 1.0f;  // ScalarThresholdNode compares >=, so this is "+1 or better"

  constexpr float kRocketChaseMinDistance = 16.0f;
  constexpr float kRocketChaseMaxDistance = 35.0f;

  // --- Mines ---
  // Measured off the human in the bot-vs-human replays: he laid exactly one mine per match, both
  // times at 5-12% energy with a pursuer ~10 tiles back while running at ~20 tiles/sec with nearly
  // all of that speed pointed straight away. It's an escape tool - dropped to make a chaser break
  // off - not an area-denial one. Too close and we're still inside our own blast when it goes off;
  // too far and they simply steer around it.
  constexpr float kMineMinPursuerDistance = 7.0f;
  constexpr float kMineMaxPursuerDistance = 16.0f;
  // The chaser has to actually be behind us and running us down. Being at speed inside a distance
  // band - all the first cut checked - does not distinguish running away from running in, and the
  // bots duly used mines offensively: in rec11 four of seven laid them while closing on the enemy
  // at 17-23 tiles/sec (radial -16.7 to -23.0), against the human's +19.2/+19.5 flat-out retreats.
  constexpr float kMineRearConeDegrees = 120.0f;
  constexpr float kMinePursuitClosingSpeed = 6.0f;
  // Don't spend the one mine we get, plus its fire cost, unless we're healthy enough that surviving
  // the exchange is still the plan. Note this is a deliberately conservative rule rather than a
  // copy of the human, who laid his at 5-12% energy as a last resort.
  constexpr float kMineMinEnergyPercent = 0.75f;

  // --- Multifire ---
  // Multifire fans the shot instead of firing a single line: more energy per trigger, worse against
  // one target, better when several are bunched and a spread can catch more than one. Enemies
  // within this radius of the target count as a cluster worth fanning into.
  constexpr float kMultifireClusterRadius = 5.0f;
  constexpr float kMultifireMinEnemies = 2.0f;
  // Don't pay the extra cost per shot while we're short on energy.
  constexpr float kMultifireMinEnergyPercent = 0.5f;

  // Don't take bullet shots past this. Two things made long-range fire actively wasteful rather
  // than merely low-value: the aim solver was under-leading (fixed in PredictiveAimNode), and
  // beyond 40 tiles the tree was deliberately wobbling the aimpoint by up to 3 tiles via
  // ShotSpreadNode - so the bot was spraying randomized shots exactly where they were least likely
  // to land. Measured bullet hit rate is 7.4% at 35-39 tiles and below 7% past that, against
  // 13-16% at 15-19 and 33% at 10-14.
  //
  // This is set to trim the wasteful tail (the bot's 90th-percentile firing range was 53 tiles),
  // not to make the bot stingy. It deliberately isn't pulled in much further: volume of fire is the
  // one metric that separated winning from losing players in the replay corpus (24.0 vs 19.2
  // shots/min alive), so cutting deep into ordinary firing range would imitate the losing half.
  constexpr float kMaxBulletRange = 35.0f;

  //  If an enemy is near us and we're low energy thor if below this value
  constexpr float kThorEnemyThreshold = 200.0f;

  // How close a target has to be before we'll take a bullet shot whose direct lane is blocked by
  // terrain, betting on a bounce. Bullets bounce in this arena, so a blocked lane isn't automatically
  // a wasted shot - but a ricochet only has a real chance while the geometry is tight and the
  // remaining travel is short, so this sits just outside the engagement pump's inner edge (10.4).
  // Bombs get no equivalent allowance; see ShotLineOfSightNode.
  constexpr float kBulletBounceRange = 12.0f;

  // How far away from a teammate before we regroup (attach-to-safe-teammate check only).
  constexpr float kTeamRange = 40.0f;

  // How far we're willing to drift from the team's centre of mass before rejoining it.
  //
  // The old rule fired at 40 tiles from a single teammate, which is far too late: in the 12-0
  // bot-vs-human replay every death happened with no teammate at all inside 25 tiles, at a median
  // 58.6 tiles from the nearest one. By the time a 40-tile pairwise check trips, the bot is already
  // in the situation that kills it. The two sides of that match separated at roughly 24 tiles
  // (winners) against 43 (losers) of median support distance, so this sits just above the winning
  // side's spacing - close enough to hold a group, loose enough not to fire constantly while
  // fighting normally.
  constexpr float kTeamCohesionRange = 30.0f;

  // How far a retreat is allowed to bend toward the team instead of running dead away from the
  // chaser. Bots were ending a sustained retreat a median 3-7 tiles further from their nearest
  // teammate than they started it, over 33-62 retreats a match, which is precisely how they arrive
  // at the isolated 1-vs-2 that every death in these matches turns out to be. The human ends his
  // retreats at +1.4 tiles and points them far less away from his own side (median 84 degrees off
  // the bearing to it, against the bots' 101-140).
  //
  // 60 degrees still opens range - just on an arc back toward support rather than a straight line
  // into an empty corner of the map.
  constexpr float kFleeTeamBiasRadians = 1.05f;  // ~60 degrees

  // Cruising speed as a fraction of the ship's top speed. Teams mostly hold station and trade shots
  // until someone fails a dodge, and only then commits - so flat-out is the wrong default. A ship
  // already at maximum has no acceleration left to dodge with and carries momentum it cannot
  // cheaply reverse. Full speed is reserved for actually pressing a target ("rushing") and for
  // running away (recharge_timer), both exempted below.
  constexpr float kCruiseSpeedPercent = 0.8f;

  // Standoff to break off to, at full energy. This is a kiting leash rather than an escape -
  // FleeNode actively holds this band - which is correct while healthy and wrong while hurt, so it
  // is now the *healthy* end of a ramp rather than a fixed distance. See FleeDistanceNode.
  constexpr float kLeashDistance = 30.0f;

  // The hurt end of that ramp. Killers begin the run that lands a kill at a median 30.8 tiles and
  // close it in three seconds, so an injured bot holding the old fixed 30 was recharging inside the
  // kill funnel. 55 is outside effective bullet range (kMaxBulletRange 35) and outside the whole
  // observed approach, so a recharge there actually completes.
  constexpr float kLeashDistanceHurt = 55.0f;
  // Energy at which the ramp bottoms out. Roughly where the retreat triggers anyway, so the widening
  // is complete by the time the bot is committed to breaking off rather than still catching up to it.
  constexpr float kLeashHurtEnergyPercent = 0.35f;

  // Below this, FleeNode abandons the leash entirely and just opens distance. Raised from 0.20:
  // bots died at 7.5-9.2% energy, and at 20% the killer is already inside 13 tiles - half a second
  // from landing it - so the getaway was starting after the fight was decided. At 30% it starts
  // while there is still roughly 20 tiles of separation to work with.
  constexpr float kFleePanicEnergyPercent = 0.3f;

  // Once within this distance of the target, stop closing further and circle instead - close
  // enough that they'll eventually fail to dodge a lobbed shot and we can dive in, far enough to
  // have room to maneuver instead of colliding.
  //
  // Measured bullet hit rate falls off a cliff right where the old fixed 15.0f sat: 33.3% at 10-14
  // tiles against 16.3% at 15-19, which is why this was originally pulled in to 12.
  //
  // Raised 20% to 14.4 off live observation: bots were sitting too close during ordinary trading
  // fire, and paying for it - in rec15 they took 2.19-3.81 damage/sec against the human's 1.79 while
  // landing comparable bullet accuracy. Holding a little further out costs some hit rate and buys
  // back more than that in damage avoided.
  //
  // Note this puts the outer end of the pump (kPumpAmplitude, +/-4) at 18.4 tiles, past the
  // accuracy cliff, where 12 used to keep the whole cycle underneath it. That is a deliberate
  // trade rather than an oversight; if hit rate drops more than damage taken improves, trim the
  // amplitude rather than pulling this back.
  constexpr float kOrbitDistance = 14.4f;

  // An enemy inside this range is our problem regardless of what the rest of the team is doing -
  // we can't ignore someone shooting us in the face to go help elsewhere. Outside it, defer to the
  // team's focus target so four bots stop splitting into four separate duels.
  //
  // This was 15, which quietly disabled the whole team-focus override: we orbit at kOrbitDistance
  // (12) with a +/-4 pump, so the engaged target is almost always inside 15 tiles and the
  // self-defense exception fired essentially every tick. rec10 was the first match with team focus
  // enabled and the bot team's focus rate went *down* (52% against rec9's 61%), which is what that
  // looks like. The threshold has to be well inside normal fighting range to mean "on top of us"
  // rather than "engaged at all".
  // Raised 8 -> 10 alongside the engagement distance going to 14.4: this has to mean "closer than we
  // ever intend to be" relative to the pump's inner edge (10.4), not a fixed number. Too high and it
  // fires every tick and disables team focus entirely, which is what 15 did originally.
  constexpr float kSelfDefenseDistance = 10.0f;

  // Radius used for the local head-count that decides whether we're supported or outnumbered.
  // Matches the radius the replay analysis bucketed on, so the exchange table it came from applies.
  constexpr float kLocalAdvantageRadius = 25.0f;

  // Standoff to hold when the nearby head-count is against us. The exchange ratio while outnumbered
  // is bad at every range (0.57 at 5-9 tiles, 0.68 at 15-19), so this isn't a range that wins - it's
  // the range that loses least while we disengage and wait for a teammate.
  constexpr float kOutnumberedDistance = 28.0f;

  // In-and-out oscillation applied on top of whichever base range is active, timed off the observed
  // rhythm: closing runs of a median 1.7s and back-off runs of 1.4s, each sweeping a median ~9-10
  // tiles (p25 ~4). Amplitude is held to 4 rather than the full half-sweep so that the outer
  // extreme of the supported cycle lands at 16 tiles instead of pushing past the accuracy cliff at
  // 15 - the whole point of orbiting at 12 is to keep the cycle on the good side of it.
  constexpr float kPumpAmplitude = 4.0f;
  constexpr u32 kPumpHalfPeriodTicks = 150;  // ~1.5s per leg

  // Bomb hitbox tolerance multiplier while orbiting - bigger than the bullet/thor multiplier below
  // so bombs land as area denial off a near miss instead of needing a precise direct hit, like
  // lobbing them into blast range instead of sniping with them.
  //
  // This being range-blind is the point, not a defect. A dodge-likelihood gate was tried on top of
  // it (BombImpactLikelihoodNode, since removed) on the theory that a bomb the target can simply
  // thrust clear of is a wasted bomb. The physics was right and the conclusion was wrong: bombs
  // here are area denial and cover fire, so one that merely forces a dodge has already done its
  // job, and gating on expected direct hits cut bomb volume to almost nothing. Accuracy is not the
  // metric a cover-fire weapon should be tuned on.
  constexpr float kBombProximityMultiplier = 8.0f;

  // Burst-fire pacing (a 0.3s firing window followed by a forced ~1s pause) used to live here and
  // has been removed, because the corpus says it was modelling a habit that doesn't exist and
  // costing us the one thing that actually separates strong players from weak ones.
  //
  // Players do not self-throttle: only 2.4% of inter-shot gaps fall under 0.2s, so the weapon's own
  // cooldown is already the binding constraint and a synthetic pause on top of it is pure lost
  // output. And ranking the 28 players with enough data by K/D, nearly every positioning metric is
  // flat between the top and bottom thirds - median firing range 28.3 vs 28.7 tiles, hit rate 13.1%
  // vs 12.9%, held range 31.2 vs 31.4, support distance 26.5 vs 26.0, damage taken 1.73 vs 1.76
  // %max/sec. The metric that does separate them is volume of fire: 24.0 shots/min alive against
  // 19.2, a 25% edge. A throttle that cuts our rate of fire is therefore imitating the losing half
  // of the ladder.
  //
  // The between-volley BroadsideFaceNode branch went with it at the time, because it keyed off that
  // same burst timer. It is back, now keyed to the engagement pump's outbound leg instead - see the
  // Selector at the head of the aim-and-shoot Parallel. Tying it to the movement rhythm rather than
  // to a firing timer is what makes it safe: broadside stops the shot ray crossing the target, so
  // anything that entered broadside *because* we had not fired recently would prevent the firing
  // that releases it. Legs alternate on their own, so the window is always bounded.
  //
  // CAVEAT, and it is a real one: the corpus does not support broadside as a protective stance.
  // Damage taken *rises* sharply with heading offset - 5.2 per sample nose-on against 24.7 at
  // 90-119 degrees across rec20-23, a far starker split than the 5.3 -> 8.1 first measured. The
  // likeliest reading is still reverse causation, since you turn away once you are already being
  // hit, but it has not been demonstrated. This is in on the argument that a ship can only thrust
  // along its heading, so nose-on is the one attitude from which evading costs a full rotation
  // first. If damage taken climbs without hit rate improving, this is the branch to pull.

  // How long to keep pressing an advantage after the target loses energy (hit or spent shooting)
  // while we still have more than they do - a sustained window instead of a single-tick reaction,
  // since TargetEnergyDropNode only reports the drop on the one tick it actually happened.
  constexpr u32 kPressAdvantageTicks = 300;  // ~3s

  constexpr float kAvoidTeamDistance = 6.0f;

  // Minimum spacing we insist on from *any* enemy, not just the one we're shooting. Set inside the
  // inner edge of the engagement pump (kOrbitDistance - kPumpAmplitude = 10.4) so it only pushes
  // back when someone is closer than we ever intend to be, rather than fighting normal station
  // keeping. TwosBoxBehavior has carried this for a while; Fours never picked it up.
  constexpr float kAvoidEnemyDistance = 10.0f;

  // How close a wall needs to be before we override movement to steer clear of it while fleeing.
  // How far ahead, in seconds of travel, to look for terrain we're about to run into. A fixed
  // 5-tile radius is a quarter of a second of warning at fighting speed - far too late to turn a
  // ship carrying real momentum, which is how bots ended up wedged in pockets and then died on the
  // way out. Detection now scales with actual speed; kWallCheckDistance stays as the contact-range
  // backstop.
  constexpr float kWallLookaheadSeconds = 0.9f;

  constexpr float kWallCheckDistance = 5.0f;
  // How far out to search for an opening once a wall is too close.
  constexpr float kWallOpeningDistance = 35.0f;

  // How far ahead (in seconds worth of their smoothed acceleration) to bend predicted aim toward
  // where the target is actually trending, instead of assuming they hold their current velocity.
  constexpr float kAimLeadBiasSeconds = 0.2f;

  // Shot spread has been dropped from this tree entirely (the ShotSpreadNode header stays - other
  // nexus behaviors still use it). It scaled deliberate aim error by how hard the target was
  // maneuvering, on the theory that a dodging target needs to be hedged against rather than aimed
  // at precisely. The replay corpus doesn't support the premise: bullet hit rate is flat at ~13%
  // across every target lateral-speed bucket from 0-2 up to 18-20 tiles/sec, so accuracy is limited
  // by range, not by how much the target is jinking. Deliberate spread was therefore pure accuracy
  // loss layered on top of an aim solver that was already under-leading.

  // Enter a defensive (recharging) state once our energy drops below this fraction of the
  // target's estimated energy, and don't leave it again until we recover past the higher exit
  // ratio - the gap between the two is a hysteresis band so we don't flicker near parity.
  constexpr float kEnergyDisadvantageEnterRatio = 0.65f;
  constexpr float kEnergyDisadvantageExitRatio = 0.9f;
  // Always treat energy this low as a disadvantage regardless of the target's energy, since being
  // critically low is dangerous even against an equally weak target.
  // Raised from 0.094. Bots were dying at 7.9-15.3% energy while the human died at 2.9%, so a floor
  // at 9.4% was firing only once escape was no longer possible - by then a single bomb finishes you
  // and the retreat has nowhere to go. Leaving at 25% is what buys enough margin to actually get
  // out. This is the most likely constant here to need softening if bots turn out too skittish.
  // Walked back from 0.25 after rec17. At 0.25 this fired far too readily and, combined with a
  // retreat that used to suppress firing entirely, produced a 12-0 loss where the fleeing team got
  // off 49-78 bullets all match. 0.18 still leaves meaningfully more escape margin than the
  // original 0.094 - bots were dying at 7.9-15.3% before it was raised at all - without putting them
  // in permanent retreat.
  constexpr float kCriticalEnergyPercent = 0.18f;

  // Absolute floor on ENDING a retreat. The ratio test above only says whether we're still losing
  // the comparison, and while we're away recharging so is the enemy - so against an equally hurt
  // opponent it can clear with both sides near dead, sending us back into a fight we're still too
  // weak for. rec28 showed exactly that: retreating% rose to 51-64% but bots still died at
  // 5.9-13.2% energy and their retreats plateaued at 36 tiles against a ramp asking for 49-55,
  // because the retreat ended before they ever arrived.
  //
  // 0.5 rather than higher because a retreating bot is nearly silent: the return-fire branch is
  // capped at kMaxBulletRange (35) and the flee ramp holds us at 49-55 tiles while hurt, so time
  // spent recovering is time the team is effectively a body down. That tradeoff is what turned the
  // rec17 over-retreat into a 12-0 loss, and this is the constant that governs it - if damage dealt
  // falls or outnumbered% climbs because bots are away too long, lower this first.
  constexpr float kRetreatRecoveryEnergyPercent = 0.5f;

  // EnergyDisadvantageNode only ever compares us to the *current target's* energy, so it has no
  // notion of being outnumbered: in a 3v1 where the nearest enemy happens to be the hurt one, it
  // reports no disadvantage at all and the bot keeps fighting. That is the gap these two rules
  // close, and it is the one the replays point straight at - in rec15 Lalita spent 98.7% of her
  // hurt-and-close time outnumbered, retreated in only 45% of it, and died at 14.5% energy against
  // a -2 head-count. Bots overall spent about twice as long as the human in that state (27.7s and
  // 35.6s against his 15.5s) while retreating far less of it (38-68% against his 77.8%).
  //
  // At -2 or worse the exchange is a loss at every range (ratio 0.57-0.68 in the corpus table), so
  // that case leaves unconditionally. At -1 it's only worth breaking off if we're also not healthy.
  constexpr float kBadlyOutnumberedAdvantage = -1.0f;  // InvertChild fires below this, i.e. -2 or worse
  constexpr float kOutnumberedRetreatEnergy = 0.6f;

  // A hard floor on engaging at all. Below this we are one clean hit from dead and every exchange is
  // a losing one, so the only fight worth being in is one we can end this second.
  //
  // This sits below kCriticalEnergyPercent (0.18) rather than replacing it. That one is the general
  // "we're losing, disengage" trigger and is relative to the enemy; this is an absolute rule that
  // holds no matter how the comparison comes out, and - more to the point - it is what the finish
  // exemption below is measured against. Keeping them separate means the exemption can never widen
  // just because the retreat threshold gets retuned.
  constexpr float kEngageFloorEnergyPercent = 0.15f;

  // The exemption. Both conditions have to hold: they are well below us *and* nearly dead outright.
  // Relative alone is worthless down here - 14% against 13% is a coin flip, not a kill - and
  // absolute alone would have us trading with someone equally low for no reason.
  constexpr float kFinishRelativeEnergyPercent = 0.5f;
  constexpr float kFinishAbsoluteEnergyPercent = 0.15f;
  // And close enough that "finish them" is a real claim rather than an intention. Inside the
  // engagement pump's inner edge (10.4), so this is knife range, not a decision to go chase.
  constexpr float kFinishDistance = 10.0f;

  //.Child<ReadConfigIntNode<u16>>("queue_command1", "command1")
  //.Child<ReadConfigIntNode<u16>>("queue_command2", "command2")
  //.Child<ReadConfigIntNode<u16>>("queue_command3", "command3")
  //.Child<ChatMessageNode>(ChatMessageNode::PublicBlackboard("command1")) // Invert so this fails and freq is
  // reevaluated. .Child<ChatMessageNode>(ChatMessageNode::PublicBlackboard("command2")) // Invert so this fails and
  // freq is reevaluated. .Child<ChatMessageNode>(ChatMessageNode::PublicBlackboard("command3")) // Invert so this fails
  // and freq is reevaluated.
  // clang-format off
  builder
    .Selector()
        .Sequence() //Join the queue first thing and auto join TODO: add command or checks to requeue if something goes wrong later such as recycled arena
            //.InvertChild<BlackboardSetQueryNode>("queued") //Check if we have already joined the queue, if not join
            .Child<TimerExpiredNode>("queue")
            .Child<ChatMessageNode>(ChatMessageNode::Public("?next 4v4pub"))
            .Child<ChatMessageNode>(ChatMessageNode::Public("?return")) 
            .Child<TimerSetNode>("queue", 6000)
            //.Child<ScalarNode>(1.0f, "queued")  //was only joining queue on join then stopped working
            .End()
        .Sequence() // Don't do anything while in spec
            .Child<PlayerFrequencyQueryNode>("self_freq")
            .Child<EqualityNode<u16>>("self_freq", 8025)  //Check spec
            .Child<ScalarNode>(1.0f, "spectating")
            .End()
        .Sequence() // Match startup begins when we get taken out of spec (since we sit in spec when waiting)
            .Child<BlackboardSetQueryNode>("spectating")  //We just came out of spectating
            .Child<TimerSetNode>("match_startup", 3000)  //Safety net only - Nexus.cpp expires this immediately once it sees the "GO!" match start message over private chat.
            .Child<BlackboardEraseNode>("spectating")
            .End()
        .Sequence() // Enter the specified ship if not already in it and have been taken out of spec.
            .InvertChild<TimerExpiredNode>("match_startup")            
            .InvertChild<ShipQueryNode>("request_ship")
            .Child<ShipRequestNode>("request_ship")
            .End()
        .Sequence()  // Fire 1 shot startup shot and set targets position to monitor for when they move so we can get out of the ready check loop
            .InvertChild<TimerExpiredNode>("match_startup")      
            .Child<TimerExpiredNode>("pre_fire")  // just needs to be longer than match_start
            .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bullet)
            .Child<InputActionNode>(InputAction::Bullet)
            .Child<TimerSetNode>("pre_fire", 1500)
            .End()
        .Sequence() //Attach if someone is safe and we have full energy
            .Child<BlackboardSetQueryNode>("tchat_safe")
            .InvertChild<TimerExpiredNode>("tchat_safe_timer")
            .Child<PlayerEnergyPercentThresholdNode>(1.0f)
            .Child<TimerExpiredNode>("attach_cooldown")
            .InvertChild<AttachedQueryNode>("self")
            .Child<NearestTeammateNode>("nearest_teammate") 
            .Child<PlayerPositionQueryNode>("nearest_teammate", "nearest_teammate_position")
            .Child<DistanceThresholdNode>("nearest_teammate_position", kTeamRange) //If we're already near teammates dont run to them               
            .Child<PlayerByNameNode>("tchat_safe", "tchat_safe_player")
            .Child<AttachNode>("tchat_safe_player")
            .Child<TimerSetNode>("attach_cooldown", 100)
            .Child<BlackboardEraseNode>("tchat_safe")
            .Child<BlackboardEraseNode>("tchat_safe_timer")
            .End()
        .Sequence() // Detach if attached
            .Child<AttachedQueryNode>("self")
            .Child<DetachNode>()
            .End()
           .Selector() // Choose to fight the player or follow waypoints.
            .Sequence() // Find nearest target and either path to them or seek them directly.              
                .Sequence(CompositeDecorator::Success)
                    .Child<PlayerPositionQueryNode>("self_position")
                    .Sequence() //Base pick: the genuinely nearest enemy, kept under its own keys as well. "target" gets overridden by the team focus and low-energy rules below, and every defensive check needs the one actually on top of us rather than the one we've chosen to shoot.
                        .Child<NearestMemoryTargetNode>("target")
                        .Child<NearestMemoryTargetNode>("nearest_target")
                        .Child<PlayerEnergyQueryNode>("nearest_target", "nearest_target_energy")
                        .Child<PlayerPositionQueryNode>("nearest_target", "nearest_target_position")
                        .Child<TargetAccelerationNode>("nearest_target", "nearest_target_acceleration")
                        .Child<PredictiveAimNode>(WeaponType::Bullet, "nearest_target", "nearest_target_acceleration", "nearest_aimshot", kAimLeadBiasSeconds)
                        .End()
                     .Sequence(CompositeDecorator::Success) //Fight what the team is fighting, unless someone is already on top of us
                        .Child<DistanceThresholdNode>("nearest_target_position", "self_position", kSelfDefenseDistance) //If an enemy is right on us, deal with them instead
                        .Child<TeamFocusTargetNode>("target")
                        .End()
                     .Sequence(CompositeDecorator::Success) //If is someone low nearby override target
                        .Child<TimerExpiredNode>("recharge_timer") //Nearest target should be used when recharing
                        .Child<LowestTargetNode>("lowest_target")
                        .Child<PlayerPositionQueryNode>("lowest_target", "lowest_target_position")
                        .Child<PlayerEnergyQueryNode>("lowest_target", "lowest_target_energy")
                        .InvertChild<DistanceThresholdNode>("lowest_target_position", "self_position", kLowEnergyDistanceThreshold)
                        .InvertChild<ScalarThresholdNode<float>>("lowest_target_energy", kLowEnergyThreshold)
                        .Child<LowestTargetNode>("target")
                        .End()
                     .Sequence(CompositeDecorator::Success) //Derive everything else once, from whichever target actually survived the override chain above.
                        .Child<PlayerPositionQueryNode>("target", "target_position")
                        .Child<PlayerEnergyQueryNode>("target", "target_energy")
                        .Child<TargetAccelerationNode>("target", "target_acceleration")
                        .Child<PredictiveAimNode>(WeaponType::Bullet, "target", "target_acceleration", "aimshot", kAimLeadBiasSeconds)
                        .Child<PredictiveAimNode>(WeaponType::Bomb, "target", "target_acceleration", "bomb_aimshot", kAimLeadBiasSeconds) //Bombs fly slower than bullets, so they need their own (larger) lead
                        .Child<TargetEnergyDropNode>("target", "target_energy", "target_energy_dropped") //Did *this* target just lose energy - identity-checked, so a target switch is no longer misread as a hit
                        .End()
                .End()
                .Sequence(CompositeDecorator::Success) // If we have a portal but no location, lay one down.
                    .Child<ShipItemCountThresholdNode>(ShipItemType::Portal, 1)
                    .InvertChild<ShipPortalPositionQueryNode>()
                    .Child<InputActionNode>(InputAction::Portal)
                    .End()
                .Selector(CompositeDecorator::Success) // Multifire covers ground rather than a point, so run it only when a spread can catch more than one enemy.
                    .Sequence() // Turn it on for a cluster we can afford to fan into.
                        .Child<ShipCapabilityQueryNode>(ShipCapability_Multifire)
                        .Child<ScalarThresholdNode<float>>("enemies_near_target", kMultifireMinEnemies)
                        .Child<PlayerEnergyPercentThresholdNode>(kMultifireMinEnergyPercent)
                        .InvertChild<ShipMultifireQueryNode>()  //Check if multifire is off
                        .Child<InputActionNode>(InputAction::Multifire) //Turn on multifire
                        .End()
                    .Sequence() // Turn it back off for a lone target, or once we can't afford the per-shot premium.
                        .Child<ShipCapabilityQueryNode>(ShipCapability_Multifire)
                        .Child<ShipMultifireQueryNode>()  //Check if multifire is on
                        .Selector()
                            .InvertChild<ScalarThresholdNode<float>>("enemies_near_target", kMultifireMinEnemies)
                            .InvertChild<PlayerEnergyPercentThresholdNode>(kMultifireMinEnergyPercent)
                            .End()
                        .Child<InputActionNode>(InputAction::Multifire)  //Turn off multifire
                        .End()
                    .End()
                .Selector(CompositeDecorator::Success) // Toggle antiwarp based on energy
                    .Sequence() // Enable antiwarp if we are healthy
                        .Child<TimerExpiredNode>("tchat_safe_timer")  
                        .Child<ShipCapabilityQueryNode>(ShipCapability_Antiwarp)
                        .Child<PlayerEnergyPercentThresholdNode>(0.75f)
                        .InvertChild<PlayerStatusQueryNode>(Status_Antiwarp)
                        .Child<InputActionNode>(InputAction::Antiwarp)
                        .End()
                    .Sequence() // Disable antiwarp if we aren't healthy
                        .Child<ShipCapabilityQueryNode>(ShipCapability_Antiwarp)
                        .InvertChild<PlayerEnergyPercentThresholdNode>(0.75f)
                        .Child<PlayerStatusQueryNode>(Status_Antiwarp)
                        .Child<InputActionNode>(InputAction::Antiwarp)
                        .End()
                    .End()
                .Sequence(CompositeDecorator::Success) // Work out how close we should be fighting right now: the exchange favors closing hard at parity or better, and backing off while outnumbered.
                    .Child<LocalAdvantageNode>(kLocalAdvantageRadius, "local_advantage")
                    .Child<EngagementRangeNode>("local_advantage", "engagement_range", kOrbitDistance, kOutnumberedDistance, kPumpAmplitude, kPumpHalfPeriodTicks, "pump_outbound")
                    .Child<FleeDistanceNode>("flee_distance", kLeashDistance, kLeashDistanceHurt, kLeashHurtEnergyPercent) //How far to break off scales with how hurt we are - a fixed 30 tiles held injured bots in the exact band killers start their run from
                    .Child<EnemiesNearTargetNode>("target", kMultifireClusterRadius, "enemies_near_target") //Drives the multifire toggle below
                    .Child<EnemiesNearTargetNode>("target", kRocketIsolationRadius, "enemies_near_target_wide") //Wider count, for "is this target actually on its own" - drives the rocket gate
                    .Selector(CompositeDecorator::Success) // Keep the team's centre of mass fresh for the flee bias below - or clear it outright if we're the last one alive, so we don't retreat toward a dead teammate's last position.
                        .Child<TeamCentroidNode>("team_centroid")
                        .Child<BlackboardEraseNode>("team_centroid")
                        .End()
                    .Selector(CompositeDecorator::Success) // Hold something back unless we're committing to a kill or running for our life.
                        .Child<BlackboardSetQueryNode>("rushing")            //Pressing a target - commit everything
                        .InvertChild<TimerExpiredNode>("recharge_timer")     //Escaping - we want every bit of speed
                        .Child<CruiseSpeedNode>(kCruiseSpeedPercent)
                        .End()
                    .End()
                .Sequence(CompositeDecorator::Success) // Continuously reassess fight-vs-flee using energy relative to the target, instead of a fixed timer.
                    .Child<EnergyDisadvantageNode>("nearest_target", "nearest_target_energy", "energy_disadvantaged", kEnergyDisadvantageEnterRatio, kEnergyDisadvantageExitRatio, kCriticalEnergyPercent, kRetreatRecoveryEnergyPercent) //Judge fight-vs-flee against the enemy actually on top of us. Comparing against the team focus target meant a bot could be losing badly to someone at 3 tiles while reporting no disadvantage because the far target it had chosen to shoot was weaker.
                    .Child<TimerSetNode>("recharge_timer", 200)
                    .End()
                .Sequence(CompositeDecorator::Success) // Badly outnumbered - leave regardless of how the nearest duel happens to be going.
                    .InvertChild<ScalarThresholdNode<float>>("local_advantage", kBadlyOutnumberedAdvantage)
                    .Child<TimerSetNode>("recharge_timer", 200)
                    .End()
                .Sequence(CompositeDecorator::Success) // Down bodies and not healthy - stop trading and get out.
                    .InvertChild<ScalarThresholdNode<float>>("local_advantage", 0.0f)
                    .InvertChild<PlayerEnergyPercentThresholdNode>(kOutnumberedRetreatEnergy)
                    .Child<TimerSetNode>("recharge_timer", 200)
                    .End()
                .Sequence(CompositeDecorator::Success) // Hard floor: below this we don't engage anyone, full stop. EnergyDisadvantageNode's critical percent currently fires above this anyway, but that one is a tunable retreat threshold and this is a rule - stated separately so retuning the former can't quietly repeal the latter.
                    .InvertChild<PlayerEnergyPercentThresholdNode>(kEngageFloorEnergyPercent)
                    .Child<TimerSetNode>("recharge_timer", 200)
                    .End()
                .Selector(CompositeDecorator::Success) // Below the engage floor the only fight worth staying in is one we can end. Everything else above has already set recharge_timer by now; this is the single exception that takes it back off.
                    .Sequence()
                        .InvertChild<PlayerEnergyPercentThresholdNode>(kEngageFloorEnergyPercent) // we're under the floor
                        .Child<FinishableTargetNode>("target", "target_energy", kFinishRelativeEnergyPercent, kFinishAbsoluteEnergyPercent) // ...but they're clearly lower and nearly dead
                        .InvertChild<DistanceThresholdNode>("target_position", "self_position", kFinishDistance) // close enough to actually land it
                        .Child<VisibilityQueryNode>("target_position") // no pathing across the map at this energy
                        .Child<ScalarThresholdNode<float>>("local_advantage", 0.0f) // and not while their friends are the ones nearby
                        .Selector() // A dive without a repel in reserve has no way out if it goes wrong. Last one alive there's nobody left to wait for, so take the chance.
                            .Child<ShipItemCountThresholdNode>(ShipItemType::Repel, kRushRepelThreshold)
                            .InvertChild<BlackboardSetQueryNode>("team_centroid")
                            .End()
                        .Child<ScalarNode>(1.0f, "finishing")
                        .Child<BlackboardEraseNode>("recharge_timer")
                        .End()
                    .Child<BlackboardEraseNode>("finishing")
                    .End()
                .Selector()
                    .Sequence() // Attempt to dodge and use defensive items.
                        .Sequence(CompositeDecorator::Success) // Always check incoming damage so we can use it in repel and portal sequences.
                            .Child<IncomingBlastDamageNode>(kRepelDistance, "incoming_damage")  //Blast-falloff aware, so a bomb clipping our edge isn't scored as a lethal direct hit and doesn't burn a repel
                            .Child<PlayerCurrentEnergyQueryNode>("self_energy")
                            .End()
                        .Sequence(CompositeDecorator::Success) // If we are in danger but can't repel, use our portal.
                            .InvertChild<ShipItemCountThresholdNode>(ShipItemType::Repel)
                            .Child<ShipPortalPositionQueryNode>() // Check if we have a portal down.
                            .Child<ScalarThresholdNode<float>>("incoming_damage", "self_energy")  //If incoming damage is > than our current energy
                            .Child<TimerExpiredNode>("defense_timer")
                            .Child<InputActionNode>(InputAction::Warp)
                            .Child<TimerSetNode>("defense_timer", 100)
                            .End()
                        .Sequence(CompositeDecorator::Success) // Use repel when in danger.
                            .Child<ShipWeaponCapabilityQueryNode>(WeaponType::Repel)
                            .Child<TimerExpiredNode>("defense_timer")
                            .Child<ScalarThresholdNode<float>>("incoming_damage", "self_energy")  //If incoming damage is > than our current energy
                            .Child<InputActionNode>(InputAction::Repel)
                            .Child<TimerSetNode>("defense_timer", 100)
                            .End()
                        .Sequence(CompositeDecorator::Invert) // Check if enemy is very low energy and close to use. Don't bother dodging if they are rushing us with low energy.
                            .InvertChild<ScalarThresholdNode<float>>("target_energy", kLowEnergyRushThreshold)
                            .InvertChild<DistanceThresholdNode>("target_position", "self_position", kRushDistanceThreshold)
                            .End()
                        .Child<DodgeIncomingDamage>(0.2f, 30.0f)
                        .End()
                    .Sequence() // Keep distance from the target during ready-check instead of sitting still until the match officially starts.
                        .InvertChild<TimerExpiredNode>("match_startup")
                        .Selector() // Steer clear of nearby walls before fleeing so we don't get pinned in a corner.
                            .Child<WallAvoidanceNode>(kWallCheckDistance, kWallOpeningDistance, kWallLookaheadSeconds, "team_centroid") //Additive now - returns Failure so the flee below still runs and both forces sum
                            .Child<FleeNode>("nearest_target_position", kLeashDistance, 5.0f, 0.2f, "nearest_target_energy")
                            .End()
                        .End()
                    .Sequence()  //Keep enemy distance while reacharging
                        .InvertChild<TimerExpiredNode>("recharge_timer")
                        .Child<BlackboardEraseNode>("rushing") //We're breaking off, so we are no longer pressing. Without this "rushing" is only ever cleared inside the aim-and-shoot Parallel below, which this branch skips entirely - so it stayed set for the whole retreat, keeping the commitment posture alive, bypassing the cruise-speed cap and the firing energy gate long after we stopped shooting.
                        .Sequence(CompositeDecorator::Success) // Drop a mine into our wake, only for a chaser actually running us down from behind.
                            .Child<PlayerEnergyPercentThresholdNode>(kMineMinEnergyPercent)
                            .Child<AtMaxSpeedNode>(kRocketMinSpeedPercent)
                            .Child<PursuedFromBehindNode>("nearest_target", kMineRearConeDegrees, kMinePursuitClosingSpeed) //Must be the enemy actually chasing us, not the team's focus target somewhere else - checking "target" here is how mines started getting laid offensively again
                            .Child<DistanceThresholdNode>("nearest_target_position", "self_position", kMineMinPursuerDistance)
                            .InvertChild<DistanceThresholdNode>("nearest_target_position", "self_position", kMineMaxPursuerDistance)
                            .Child<TimerExpiredNode>("mine_timer")
                            .Child<MineAvailableNode>()
                            .Child<InputActionNode>(InputAction::Mine)
                            .Child<TimerSetNode>("mine_timer", 500)
                            .End()
                        .Selector() // Steer clear of nearby walls before fleeing so we don't get pinned in a corner.
                            .Child<WallAvoidanceNode>(kWallCheckDistance, kWallOpeningDistance, kWallLookaheadSeconds, "team_centroid") //Additive now - returns Failure so the flee below still runs and both forces sum
                            .Child<FleeNode>("nearest_aimshot", "flee_distance", 5.0f, kFleePanicEnergyPercent, "nearest_target_energy", "team_centroid", kFleeTeamBiasRadians) //Distance now scales with injury instead of being a fixed leash, and the panic threshold is raised - see FleeDistanceNode. The low-energy panic override is judged against whoever is chasing us; pointing it at "target_energy" meant a bot fleeing a healthy enemy at 3 tiles could suppress its own panic because the distant focus target it happened to be shooting was weaker.
                            .End()
                        .Sequence(CompositeDecorator::Success) // Keep shooting at whoever is chasing us. Backing off must not mean going silent - this branch takes the whole Selector, so the aim-and-shoot block below never runs while it is active, and without this a retreating bot fired nothing at all. In rec17 that produced a death spiral: outnumbered -> permanent retreat -> no return fire -> still outnumbered. The losing team fired 49-78 bullets all match against the winners' 136-239 and lost 12-0. FleeNode already faces the threat while retreating, so the heading is right and this only needs permission to pull the trigger.
                            .Child<TimerExpiredNode>("match_startup")
                            .InvertChild<DistanceThresholdNode>("nearest_target_position", kMaxBulletRange)
                            .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bullet)
                            .InvertChild<InputQueryNode>(InputAction::Bomb)
                            .InvertChild<TileQueryNode>(kTileIdSafe)
                            .Child<ShotVelocityQueryNode>(WeaponType::Bullet, "bullet_fire_velocity")
                            .Child<RayNode>("self_position", "bullet_fire_velocity", "bullet_fire_ray")
                            .Child<DynamicPlayerBoundingBoxQueryNode>("nearest_target", "nearest_target_bounds", 4.0f)
                            .Child<MoveRectangleNode>("nearest_target_bounds", "nearest_aimshot", "nearest_target_bounds")
                            .Child<RayRectangleInterceptNode>("bullet_fire_ray", "nearest_target_bounds")
                            .Child<ShotLineOfSightNode>("nearest_aimshot", kBulletBounceRange)  //Same terrain gate as the main fire check - retreating is when we're most likely to have terrain between us and whoever is chasing
                            .Child<InputActionNode>(InputAction::Bullet)
                            .End()
                        .End()
                    .Sequence() // Path to target if they aren't immediately visible.
                        .InvertChild<VisibilityQueryNode>("target_position")
                        .Child<ScalarThresholdNode<float>>("local_advantage", 0.0f) //Never cross the map into a fight we're already losing. This branch pathfinds straight at the target and sits ahead of the orbit/regroup movement in this Selector, so without a head-count check it happily routed the bot through the middle of the enemy team to reach a focus target picked for being near the team centroid - which is exactly the 3-on-1 dive. Failing here falls through to the regroup below instead.
                        .Child<GoToNode>("target_position")
                        .Child<AvoidTeamNode>(kAvoidTeamDistance)
                        .Child<AvoidEnemyNode>(kAvoidEnemyDistance) //Path around anyone we pass rather than straight over them
                        .Child<RenderPathNode>(Vector3f(0.0f, 1.0f, 0.5f))
                        .End()
                    .Sequence() // Aim at target and shoot while seeking them.
                        .Child<TimerExpiredNode>("match_startup")
                        .Parallel()
                            .Selector() // Face the target to line up a shot, or go broadside between waves.
                                .Sequence() // The pump's outbound leg IS the gap between waves - we're opening the range, not pressing. Turning side-on through it puts our thrust axis across their line of fire, so a dodge costs no rotation first and we're already moving laterally when their shot arrives. The leg flips inward on its own, so this window is always bounded and always followed by a facing window; gating on "haven't fired lately" instead would latch, since broadside stops the shot ray crossing the target in the first place.
                                    .Child<BlackboardSetQueryNode>("pump_outbound")
                                    .InvertChild<BlackboardSetQueryNode>("rushing") //A committed dive stays nose-on
                                    .InvertChild<BlackboardSetQueryNode>("finishing")
                                    .Child<BroadsideFaceNode>("target_position")
                                    .End()
                                .Child<FaceNode>("aimshot")
                                .End()
                            .Child<BlackboardEraseNode>("rushing") // Clear rushing status
                            .Sequence(CompositeDecorator::Success) // Juke away from moderate incoming threats without breaking aim off the target.
                                .Child<DodgeJukeNode>(30.0f)
                                .End()
                            .Selector()
                               .Sequence() // Committed to ending a fight we're otherwise too weak to be in. Gated hard by the finish exemption above, so reaching here already means they're nearly dead, close, visible, and we aren't outnumbered - all that's left is to actually go in.
                                    .Child<BlackboardSetQueryNode>("finishing")
                                    .Child<SeekNode>("aimshot", 0.0f, SeekNode::DistanceResolveType::Static)
                                    .End()
                               .Sequence() // If there is any low target with in this range prioritize
                                    .Selector() // Diving with no repel left has no way out if it goes wrong, so normally don't. Last one alive, there's no teammate left to fall back to and nothing to preserve the life for - take the fight.
                                        .Child<ShipItemCountThresholdNode>(ShipItemType::Repel, kRushRepelThreshold)
                                        .InvertChild<BlackboardSetQueryNode>("team_centroid") //TeamCentroidNode fails and the centroid is erased above when no teammate is alive
                                        .End()
                                    .Child<PlayerEnergyPercentThresholdNode>(kRushMinEnergyPercent) //only press if we have enough energy ourselves
                                    .Child<ScalarThresholdNode<float>>("local_advantage", 0.0f) //diving while outnumbered loses the exchange ~2:1 no matter how weak the target looks
                                    .InvertChild<DistanceThresholdNode>("target_position", "self_position", kRushDistanceThreshold)
                                    .InvertChild<ScalarThresholdNode<float>>("target_energy", kLowEnergyRushThreshold)
                                    .Child<RushCommitmentNode>("target", "local_advantage", kRushMaxTicks, kRushMaxAdvantageLoss, kRushAbortCooldownTicks) // Last gate before we actually commit, so it only counts ticks where everything above already held. Every condition above is an absolute test of this instant; this is the only one that can see the fight getting worse than the one we chose to dive into.
                                    .Child<SeekNode>("aimshot", 0.0f, SeekNode::DistanceResolveType::Static)
                                    .Child<ScalarNode>(1.0f, "rushing") // set rushing status
                                    .Sequence(CompositeDecorator::Success) //Rocket down a fleeing kill, but only once we're already moving - lit from slow it mostly buys back speed we'd have reached anyway, and it overshoots.
                                        .Child<ShipItemCountThresholdNode>(ShipItemType::Rocket) // check we have rocket items
                                        .Child<PlayerEnergyPercentThresholdNode>(kRocketMinSelfEnergyPercent) // only commit a limited item while we can still afford the dive
                                        .Child<TargetEnergyPercentThresholdNode>("target", "target_energy", kRocketTargetEnergyPercent) // percent of the target's own max, not an absolute - a rocket is for finishing something already nearly dead
                                        .InvertChild<ScalarThresholdNode<float>>("enemies_near_target_wide", kRocketMaxEnemiesNearTarget) // and only if it's on its own - diving a target with friends around just delivers us into them
                                        .Child<TimerExpiredNode>("recharge_timer") // never light one while we're supposed to be breaking off
                                        .Child<TargetOpeningRangeNode>("target", kRocketMinOpeningSpeed) // only chase something that is actually running. A target holding station in its own team reads identically to a fleeing one under a distance check, and rocketing at it is the dive-into-a-crowd behavior - we arrive fast, outnumbered, with no thrust left to turn around.
                                        .Child<ScalarThresholdNode<float>>("local_advantage", kRocketMinAdvantage) // stricter than the rush around it: spend a limited item only when we're up bodies and the kill actually converts
                                        .InvertChild<RocketActiveQueryNode>() // don't stack one on top of a burn already running
                                        .Child<AtMaxSpeedNode>(kRocketMinSpeedPercent)
                                        .InvertChild<DistanceThresholdNode>("target_position", kRocketChaseMaxDistance)  //dont rocket if too far away
                                        .Child<DistanceThresholdNode>("target_position", kRocketChaseMinDistance)  //dont rocket if right on them you'll overshoot
                                        .Child<TimerExpiredNode>("rocket_timer") // check cooldown period
                                        .Child<InputActionNode>(InputAction::Rocket) // use rockets
                                        .Child<TimerSetNode>("rocket_timer", 3000) // cooldown doubled from 1500 - even when every condition above holds, a second rocket 15s into the same engagement is almost always the tail of one commitment rather than a fresh decision
                                        .End()
                                    .Child<BlackboardEraseNode>("recharge_timer") // remove recharge status as we're going in for the kill
                                    .Child<BlackboardEraseNode>("orbit_direction") // pick a fresh orbit direction next time we're back to circling
                                    .End()
                                .Sequence() // Press the advantage for a while after the target loses energy (hit or spent shooting) and now has meaningfully less than we do.
                                    .InvertChild<DistanceThresholdNode>("target_position", "self_position", kOrbitDistance * 2.0f) //still needs to be a fight we're actually in, not clear across the map
                                    .Child<ScalarThresholdNode<float>>("local_advantage", 0.0f) //same reason as the rush above - committing forward only pays while we're not outnumbered
                                    .Selector() // Same repel reserve rule as the rush above - this branch also commits us forward.
                                        .Child<ShipItemCountThresholdNode>(ShipItemType::Repel, kRushRepelThreshold)
                                        .InvertChild<BlackboardSetQueryNode>("team_centroid")
                                        .End()
                                    .Child<PlayerCurrentEnergyQueryNode>("self_energy")
                                    .Sequence(CompositeDecorator::Success) // (Re)arm the window on a fresh drop - it doesn't need to still be dropping for the window to hold.
                                        .Child<BlackboardSetQueryNode>("target_energy_dropped") //Set by TargetEnergyDropNode, which checks the drop belongs to *this* target. The old "target_energy < target_energy_prev" compared bare numbers with no identity, so switching to a weaker target read as a hit and armed a 3s commit-forward window against someone we'd never touched.
                                        .Child<GreaterThanNode<float>>("self_energy", "target_energy")
                                        .Child<TimerSetNode>("press_advantage_until", kPressAdvantageTicks)
                                        .End()
                                    .InvertChild<TimerExpiredNode>("press_advantage_until") // still inside the window from a recent drop
                                    .Child<SeekNode>("aimshot", 0.0f, SeekNode::DistanceResolveType::Static)
                                    .Child<ScalarNode>(1.0f, "rushing")
                                    .Child<BlackboardEraseNode>("recharge_timer")
                                    .Child<BlackboardEraseNode>("orbit_direction")
                                    .End()
                                .Sequence()
                                    .Child<BlackboardSetQueryNode>("energy_disadvantaged")  // Set by EnergyDisadvantageNode above, relative to the target instead of a flat self-only threshold.
                                    .Sequence(CompositeDecorator::Success)
                                        .InvertChild<ShipItemCountThresholdNode>(ShipItemType::Repel)
                                        .Child<ShipWeaponCapabilityQueryNode>(WeaponType::Decoy)
                                        .Child<TimerExpiredNode>("decoy_timer")
                                        .Child<InputActionNode>(InputAction::Decoy)
                                        .Child<TimerSetNode>("decoy_timer", 850)
                                        .End()
                                    .End()
                                .Sequence(CompositeDecorator::Success)
                                    .InvertChild<BlackboardSetQueryNode>("rushing")
                                    .Sequence(CompositeDecorator::Success) // Bake terrain into the attack movement too - additive, so it steers us around walls while still closing/orbiting rather than replacing the attack. Wrapped so its Failure doesn't abort this sequence.
                                        .Child<WallAvoidanceNode>(kWallCheckDistance, kWallOpeningDistance, kWallLookaheadSeconds, "target_position")
                                        .End()
                                    .Selector()
                                        .Sequence() // Rejoin the team when we've drifted off it - still faces/fires/juke-dodges via this Parallel instead of running blind.
                                            .Child<TeamCentroidNode>("team_centroid") //Anchor on where the team actually is, not on one teammate who is themselves running somewhere else
                                            .Child<DistanceThresholdNode>("team_centroid", kTeamCohesionRange)
                                            .InvertChild<ScalarThresholdNode<float>>("local_advantage", 1.0f) //Only stay out on our own while we're actually up bodies locally
                                            .Child<GoToNode>("team_centroid")
                                            .Child<RenderPathNode>(Vector3f(0.0f, 1.0f, 0.5f))
                                            .End()
                                        .Selector() // Close the gap while still far out, then circle instead of closing all the way to melee range.
                                            .Sequence()
                                                .Child<DistanceThresholdNode>("target_position", "self_position", "engagement_range")
                                                .Child<SeekNode>("aimshot", 0.0f, SeekNode::DistanceResolveType::Zero)
                                                .End()
                                            .Child<OrbitNode>("aimshot", "engagement_range", "orbit_direction")
                                            .End()
                                        .End()
                                    .Child<AvoidTeamNode>(kAvoidTeamDistance)
                                    .Child<AvoidEnemyNode>(kAvoidEnemyDistance) //Don't let anyone sit on top of us regardless of who we've picked to shoot. Orbit distance is held to the *target*, so before this a second enemy could close to point blank and be ignored entirely while we kept station on someone else. Guarded by the "rushing" invert above, so it never fights a committed dive.
                                    .End()
                                .End()
                            .Sequence(CompositeDecorator::Success) // Bomb fire check.
                                .Child<TimerExpiredNode>("match_startup") // Ensure match countdown timer has expired
                                .Child<TimerExpiredNode>("recharge_timer")  // Ensure we're not still in a fleeing state
                                .Child<VectorSubtractNode>("bomb_aimshot", "self_position", "target_direction", true) //check target aim
                                .Child<PlayerVelocityQueryNode>("self_velocity") // get our current velocity
                                .Child<VectorDotNode>("self_velocity", "target_direction", "forward_velocity")  // compare our velocity to target
                                .Child<ScalarThresholdNode<float>>("forward_velocity", kBombMinForwardVelocity) // don't lob one while actively backing away from the target
                                .Child<PlayerEnergyPercentThresholdNode>(0.45f) // ensure we have enough energy to fire
                                .Child<ShipWeaponCapabilityQueryNode>(WeaponType::Bomb) // ensure bombs are ready to fire
                                .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bomb) // ensure bombs are off cooldown
                                .InvertChild<InputQueryNode>(InputAction::Thor) 
                                .Child<IncomingDamageQueryNode>("target", kRepelDistance * 2.5f, 2.75f, "outgoing_damage") // check outgoing damage to target
                                .Child<ScalarThresholdNode<float>>("outgoing_damage", kBombRequiredDamageOverlap) // Check if we have enough bullets overlapping outgoing damage to fire a bomb into.
                                .InvertChild<DistanceThresholdNode>("nearest_target_position", 50.0f)  //dont bomb from too far
                                .Child<BombBlastSafetyNode>("bomb_aimshot", kBombFriendlyBlastMargin)  //never bomb when the blast would catch us or a teammate - fall through to bullets instead
                                .Child<ShotLineOfSightNode>("bomb_aimshot", 0.0f, true)  //Bombs don't pass through walls, so the lane has to be clear - but only up to proximity range of the target, since the fuse trips on the ship first and terrain inside that last stretch can't stop the shot. Walls never trip a proximity fuse; only a direct projectile hit stops a bomb. No bounce allowance either - BombBounceCount is commonly 0 and a bounced bomb does reduced damage.
                                .Child<ShotVelocityQueryNode>(WeaponType::Bomb, "bomb_fire_velocity") // check bomb velocity
                                .Child<RayNode>("self_position", "bomb_fire_velocity", "bomb_fire_ray") // check collision ray
                                .Child<DynamicPlayerBoundingBoxQueryNode>("target", "target_bounds", kBombProximityMultiplier) // lob range, not a precise hit
                                .Child<MoveRectangleNode>("target_bounds", "bomb_aimshot", "target_bounds")
                                .Child<RenderRectNode>("world_camera", "target_bounds", Vector3f(1.0f, 0.0f, 0.0f))
                                .Child<RenderRayNode>("world_camera", "bomb_fire_ray", 50.0f, Vector3f(1.0f, 1.0f, 0.0f))
                                .Child<RayRectangleInterceptNode>("bomb_fire_ray", "target_bounds")
                                .Child<InputActionNode>(InputAction::Bomb) // fire bomb
                                .End()
                            .Sequence(CompositeDecorator::Success) // PB thor fire check.
                                .Child<TimerExpiredNode>("match_startup")
                                .InvertChild<PlayerEnergyPercentThresholdNode>(0.25f)  //If we are low energy            
                                .Child<ShipWeaponCapabilityQueryNode>(WeaponType::Thor)  //If we can thor
                                .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Thor)  //If its not on cd
                                .InvertChild<InputQueryNode>(InputAction::Bomb)  //If we're not bombing
                                 .InvertChild<ScalarThresholdNode<float>>("target_energy", kThorEnemyThreshold)  //If the enemy is low health
                                .InvertChild<DistanceThresholdNode>("target_position", 8.0f) //If the enemy within pb range
                                .Child<ShotVelocityQueryNode>(WeaponType::Thor, "thor_fire_velocity")
                                .Child<RayNode>("self_position", "thor_fire_velocity", "thor_fire_ray")
                                .Child<DynamicPlayerBoundingBoxQueryNode>("target", "target_bounds", 4.0f)
                                .Child<MoveRectangleNode>("target_bounds", "aimshot", "target_bounds")
                                .Child<RenderRectNode>("world_camera", "target_bounds", Vector3f(1.0f, 0.0f, 0.0f))
                                .Child<RenderRayNode>("world_camera", "thor_fire_ray", 50.0f, Vector3f(1.0f, 1.0f, 0.0f))
                                .Child<RayRectangleInterceptNode>("thor_fire_ray", "target_bounds")
                                .Child<InputActionNode>(InputAction::Thor) //Thor
                                .End()
                            .Sequence(CompositeDecorator::Success) // Determine if a shot should be fired by using weapon trajectory and bounding boxes.
                                .Child<TimerExpiredNode>("match_startup") // Ensure match countdown timer has expired            
                                .Child<TimerExpiredNode>("recharge_timer") // Ensure we're not still in a fleeing state
                                .InvertChild<DistanceThresholdNode>("target_position", kMaxBulletRange) // Don't spray at ranges where bullets essentially never connect
                                .Child<DynamicPlayerBoundingBoxQueryNode>("target", "target_bounds", 4.0f)
                                .Child<MoveRectangleNode>("target_bounds", "aimshot", "target_bounds")
                                .Child<RenderRectNode>("world_camera", "target_bounds", Vector3f(1.0f, 0.0f, 0.0f))
                                .Selector() // Energy gate on ordinary fire, bypassed while committed - a bot that has decided to end a fight has to be allowed to shoot, and by definition it is under every energy threshold here.
                                    .Child<BlackboardSetQueryNode>("rushing")
                                    .Child<BlackboardSetQueryNode>("finishing")
                                    .Child<PlayerEnergyPercentThresholdNode>(0.35f)
                                    .End()
                                .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bullet)
                                .InvertChild<InputQueryNode>(InputAction::Bomb) // Don't try to shoot a bullet when shooting a bomb.
                                .InvertChild<TileQueryNode>(kTileIdSafe)
                                .Child<ShotVelocityQueryNode>(WeaponType::Bullet, "bullet_fire_velocity")
                                .Child<RayNode>("self_position", "bullet_fire_velocity", "bullet_fire_ray")
                                .Child<DynamicPlayerBoundingBoxQueryNode>("target", "target_bounds", 4.0f)
                                .Child<MoveRectangleNode>("target_bounds", "aimshot", "target_bounds")
                                .Child<RayRectangleInterceptNode>("bullet_fire_ray", "target_bounds")
                                .Child<ShotLineOfSightNode>("aimshot", kBulletBounceRange)  //The intercept test above knows nothing about terrain, so a target behind a wall still produces a valid-looking shot. Bounce allowance kept for tight corners.
                                .Child<InputActionNode>(InputAction::Bullet)
                                .End()
                            .End()
                        .End()
                    .End()
                .End()
            .Sequence() // Follow set waypoints.
                .Child<WaypointNode>("waypoints", "waypoint_index", "waypoint_position", 15.0f)
                .Selector()
                    .Sequence()
                        .InvertChild<VisibilityQueryNode>("waypoint_position")
                        .Child<GoToNode>("waypoint_position")
                        .Child<RenderPathNode>(Vector3f(0.0f, 0.5f, 1.0f))
                        .End()
                    .Parallel()
                        .Child<FaceNode>("waypoint_position")
                        .Child<ArriveNode>("waypoint_position", 1.25f)
                        .End()
                    .End()
                .End()
            .End()
        .End();
  // clang-format on

  return builder.Build();
}

}  // namespace nexus
}  // namespace zero