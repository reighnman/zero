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
#include <zero/zones/nexus/nodes/OrbitNode.h>
#include <zero/zones/nexus/nodes/LocalAdvantageNode.h>
#include <zero/zones/nexus/nodes/EngagementRangeNode.h>
#include <zero/zones/nexus/nodes/BombBlastSafetyNode.h>
#include <zero/zones/nexus/nodes/TeamCentroidNode.h>
#include <zero/zones/nexus/nodes/IncomingBlastDamageNode.h>
#include <zero/zones/nexus/nodes/RocketUsageNode.h>
#include <zero/zones/nexus/nodes/MineAvailableNode.h>
#include <zero/zones/nexus/nodes/EnemiesNearTargetNode.h>
#include <zero/zones/nexus/nodes/WallAvoidanceNode.h>
#include <zero/zones/nexus/nodes/DodgeIncomingDamage.h>
#include <zero/zones/nexus/nodes/DodgeJukeNode.h>
#include <zero/zones/nexus/nodes/EnergyDisadvantageNode.h>
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
  // the target. So both uses below require us to already be moving.
  constexpr float kRocketMinSpeedPercent = 0.8f;
  // Chasing: only worth it if there's real ground to make up. Inside this we'd overshoot.
  constexpr float kRocketChaseMinDistance = 12.0f;
  constexpr float kRocketChaseMaxDistance = 35.0f;
  // Escaping: light it when whoever is chasing us is this close and still coming.
  constexpr float kRocketEscapeDistance = 18.0f;

  // --- Mines ---
  // Measured off the human in the bot-vs-human replays: he laid exactly one mine per match, both
  // times at 5-12% energy with a pursuer ~10 tiles back while running at ~20 tiles/sec with nearly
  // all of that speed pointed straight away. It's an escape tool - dropped to make a chaser break
  // off - not an area-denial one. Too close and we're still inside our own blast when it goes off;
  // too far and they simply steer around it.
  constexpr float kMineMinPursuerDistance = 7.0f;
  constexpr float kMineMaxPursuerDistance = 16.0f;

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

  constexpr float kLeashDistance = 30.0f;

  // Once within this distance of the target, stop closing further and circle instead - close
  // enough that they'll eventually fail to dodge a lobbed shot and we can dive in, far enough to
  // have room to maneuver instead of colliding.
  //
  // Measured bullet hit rate falls off a cliff right where the old fixed 15.0f sat: 33.3% at 10-14
  // tiles against 16.3% at 15-19. Orbiting at 12 puts the whole pump cycle below that cliff instead
  // of straddling it, and matches the 11.4t median range at which replay kills actually land.
  constexpr float kOrbitDistance = 12.0f;

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
  // The between-volley BroadsideFaceNode branch went with it: it keyed off the burst timer, so with
  // no bursts it would have fired on every orbiting tick instead of only during lulls. The data
  // doesn't support broadside as a protective stance anyway - damage taken *rises* with heading
  // offset, from 5.31 per sample nose-on to 8.10 at 90 degrees.

  // How long to keep pressing an advantage after the target loses energy (hit or spent shooting)
  // while we still have more than they do - a sustained window instead of a single-tick reaction,
  // since target_energy_prev only differs from target_energy for the one tick the drop happened.
  constexpr u32 kPressAdvantageTicks = 300;  // ~3s

  constexpr float kAvoidTeamDistance = 6.0f;

  // How close a wall needs to be before we override movement to steer clear of it while fleeing.
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
  constexpr float kCriticalEnergyPercent = 0.094f;

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
                    .Sequence() 
                        .Child<NearestMemoryTargetNode>("target")
                        .Child<PlayerPositionQueryNode>("target", "target_position")
                        .Child<PlayerEnergyQueryNode>("target", "target_energy")
                        .Child<TargetAccelerationNode>("target", "target_acceleration")
                        .Child<PredictiveAimNode>(WeaponType::Bullet, "target", "target_acceleration", "aimshot", kAimLeadBiasSeconds)
                        .Child<PredictiveAimNode>(WeaponType::Bomb, "target", "target_acceleration", "bomb_aimshot", kAimLeadBiasSeconds) //Bombs fly slower than bullets, so they need their own (larger) lead
                        .Child<PlayerPositionQueryNode>("target", "nearest_target_position") //Addionally copy to nearest so we can use it later
                        .Child<PredictiveAimNode>(WeaponType::Bullet, "target", "target_acceleration", "nearest_aimshot", kAimLeadBiasSeconds)
                        .End()
                     .Sequence(CompositeDecorator::Success) //If is someone low nearby override target
                        .Child<TimerExpiredNode>("recharge_timer") //Nearest target should be used when recharing
                        .Child<LowestTargetNode>("lowest_target")
                        .Child<PlayerPositionQueryNode>("lowest_target", "lowest_target_position")
                        .Child<PlayerEnergyQueryNode>("lowest_target", "lowest_target_energy")
                        .InvertChild<DistanceThresholdNode>("lowest_target_position", "self_position", kLowEnergyDistanceThreshold)
                        .InvertChild<ScalarThresholdNode<float>>("lowest_target_energy", kLowEnergyThreshold)
                        .Child<LowestTargetNode>("target")
                        .Child<PlayerPositionQueryNode>("target", "target_position")  //Override
                        .Child<PlayerEnergyQueryNode>("target", "target_energy")  //Override
                        .Child<TargetAccelerationNode>("target", "target_acceleration")  //Override
                        .Child<PredictiveAimNode>(WeaponType::Bullet, "target", "target_acceleration", "aimshot", kAimLeadBiasSeconds) //Override
                        .Child<PredictiveAimNode>(WeaponType::Bomb, "target", "target_acceleration", "bomb_aimshot", kAimLeadBiasSeconds) //Override
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
                    .Child<EngagementRangeNode>("local_advantage", "engagement_range", kOrbitDistance, kOutnumberedDistance, kPumpAmplitude, kPumpHalfPeriodTicks)
                    .Child<EnemiesNearTargetNode>("target", kMultifireClusterRadius, "enemies_near_target") //Drives the multifire toggle below
                    .End()
                .Sequence(CompositeDecorator::Success) // Continuously reassess fight-vs-flee using energy relative to the target, instead of a fixed timer.
                    .Child<EnergyDisadvantageNode>("target", "target_energy", "energy_disadvantaged", kEnergyDisadvantageEnterRatio, kEnergyDisadvantageExitRatio, kCriticalEnergyPercent)
                    .Child<TimerSetNode>("recharge_timer", 200)
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
                            .Child<WallAvoidanceNode>(kWallCheckDistance, kWallOpeningDistance)
                            .Child<FleeNode>("nearest_target_position", kLeashDistance, 5.0f, 0.2f, "target_energy")
                            .End()
                        .End()
                    .Sequence()  //Keep enemy distance while reacharging
                        .InvertChild<TimerExpiredNode>("recharge_timer")
                        .Sequence(CompositeDecorator::Success) // Drop a mine behind us to make a chaser break off - only while genuinely running, at speed, with them close but not on top of us.
                            .Child<AtMaxSpeedNode>(kRocketMinSpeedPercent)
                            .Child<DistanceThresholdNode>("target_position", "self_position", kMineMinPursuerDistance)
                            .InvertChild<DistanceThresholdNode>("target_position", "self_position", kMineMaxPursuerDistance)
                            .Child<TimerExpiredNode>("mine_timer")
                            .Child<MineAvailableNode>()
                            .Child<InputActionNode>(InputAction::Mine)
                            .Child<TimerSetNode>("mine_timer", 500)
                            .End()
                        .Sequence(CompositeDecorator::Success) // Rocket clear when someone is closing on us and we're already at running speed.
                            .Child<ShipItemCountThresholdNode>(ShipItemType::Rocket)
                            .InvertChild<RocketActiveQueryNode>()
                            .Child<AtMaxSpeedNode>(kRocketMinSpeedPercent)
                            .InvertChild<DistanceThresholdNode>("target_position", "self_position", kRocketEscapeDistance)
                            .Child<TimerExpiredNode>("rocket_timer")
                            .Child<InputActionNode>(InputAction::Rocket)
                            .Child<TimerSetNode>("rocket_timer", 1500)
                            .End()
                        .Selector() // Steer clear of nearby walls before fleeing so we don't get pinned in a corner.
                            .Child<WallAvoidanceNode>(kWallCheckDistance, kWallOpeningDistance)
                            .Child<FleeNode>("nearest_aimshot", kLeashDistance, 5.0f, 0.2f, "target_energy")
                            .End()
                        .End()
                    .Sequence() // Path to target if they aren't immediately visible.
                        .InvertChild<VisibilityQueryNode>("target_position")
                        .Child<GoToNode>("target_position")
                        .Child<AvoidTeamNode>(kAvoidTeamDistance)
                        .Child<RenderPathNode>(Vector3f(0.0f, 1.0f, 0.5f))
                        .End()
                    .Sequence() // Aim at target and shoot while seeking them.
                        .Child<TimerExpiredNode>("match_startup")
                        .Parallel()
                            .Child<FaceNode>("aimshot")
                            .Child<BlackboardEraseNode>("rushing") // Clear rushing status
                            .Sequence(CompositeDecorator::Success) // Juke away from moderate incoming threats without breaking aim off the target.
                                .Child<DodgeJukeNode>(30.0f)
                                .End()
                            .Selector()
                               .Sequence() // If there is any low target with in this range prioritize
                                    .Child<ShipItemCountThresholdNode>(ShipItemType::Repel, kRushRepelThreshold) //dont go into rush mode with no reps
                                    .Child<PlayerEnergyPercentThresholdNode>(kRushMinEnergyPercent) //only press if we have enough energy ourselves
                                    .Child<ScalarThresholdNode<float>>("local_advantage", 0.0f) //diving while outnumbered loses the exchange ~2:1 no matter how weak the target looks
                                    .InvertChild<DistanceThresholdNode>("target_position", "self_position", kRushDistanceThreshold)
                                    .InvertChild<ScalarThresholdNode<float>>("target_energy", kLowEnergyRushThreshold)
                                    .Child<SeekNode>("aimshot", 0.0f, SeekNode::DistanceResolveType::Static)
                                    .Child<ScalarNode>(1.0f, "rushing") // set rushing status 
                                    .Sequence(CompositeDecorator::Success) //Rocket down a fleeing kill, but only once we're already moving - lit from slow it mostly buys back speed we'd have reached anyway, and it overshoots.
                                        .Child<ShipItemCountThresholdNode>(ShipItemType::Rocket) // check we have rocket items
                                        .Child<PlayerEnergyPercentThresholdNode>(0.6f) // check we have sufficient energy
                                        .InvertChild<RocketActiveQueryNode>() // don't stack one on top of a burn already running
                                        .Child<AtMaxSpeedNode>(kRocketMinSpeedPercent)
                                        .InvertChild<DistanceThresholdNode>("target_position", kRocketChaseMaxDistance)  //dont rocket if too far away
                                        .Child<DistanceThresholdNode>("target_position", kRocketChaseMinDistance)  //dont rocket if right on them you'll overshoot
                                        .Child<TimerExpiredNode>("rocket_timer") // check cooldown period
                                        .Child<InputActionNode>(InputAction::Rocket) // use rockets
                                        .Child<TimerSetNode>("rocket_timer", 1500) // set a rocket cooldown period
                                        .End()
                                    .Child<BlackboardEraseNode>("recharge_timer") // remove recharge status as we're going in for the kill
                                    .Child<BlackboardEraseNode>("orbit_direction") // pick a fresh orbit direction next time we're back to circling
                                    .End()
                                .Sequence() // Press the advantage for a while after the target loses energy (hit or spent shooting) and now has meaningfully less than we do.
                                    .InvertChild<DistanceThresholdNode>("target_position", "self_position", kOrbitDistance * 2.0f) //still needs to be a fight we're actually in, not clear across the map
                                    .Child<ScalarThresholdNode<float>>("local_advantage", 0.0f) //same reason as the rush above - committing forward only pays while we're not outnumbered
                                    .Child<PlayerCurrentEnergyQueryNode>("self_energy")
                                    .Sequence(CompositeDecorator::Success) // (Re)arm the window on a fresh drop - it doesn't need to still be dropping for the window to hold.
                                        .Child<LessThanNode<float>>("target_energy", "target_energy_prev")
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
                                .Selector()
                                    .Child<BlackboardSetQueryNode>("rushing")
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
                                .Child<InputActionNode>(InputAction::Bullet)
                                .End()
                            .Child<ScalarNode>("target_energy", "target_energy_prev") // snapshot for next tick's hit/spend detection above
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