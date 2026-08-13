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
#include <zero/zones/nexus/nodes/NearestTeammatePlayerPositionQueryNode.h>
#include <zero/zones/nexus/nodes/LowestTargetNode.h>
#include <zero/zones/nexus/nodes/FleeNode.h>
#include <zero/zones/nexus/nodes/OrbitNode.h>
#include <zero/zones/nexus/nodes/BroadsideFaceNode.h>
#include <zero/zones/nexus/nodes/DodgeIncomingDamage.h>
#include <zero/zones/nexus/nodes/DodgeJukeNode.h>
#include <zero/zones/nexus/nodes/EngagementPhaseNode.h>
#include <zero/zones/nexus/nodes/WeaponEnergyBandNode.h>
#include <zero/zones/nexus/nodes/TargetEnergyPercentThresholdNode.h>
#include <zero/zones/nexus/nodes/TeamAdvantageNode.h>
#include <zero/zones/nexus/nodes/IsolatedTargetNode.h>
#include <zero/zones/nexus/nodes/HerdingBombAimNode.h>
#include <zero/zones/nexus/nodes/WallEscapeNode.h>
#include <zero/zones/nexus/nodes/TrapTargetNode.h>
#include <zero/zones/trenchwars/nodes/AttachNode.h>
#include <zero/zones/nexus/nodes/PlayerByNameNode.h>
#include <zero/zones/nexus/nodes/PredictiveAimNode.h>
#include <zero/zones/nexus/nodes/ShotSpreadNode.h>
#include <zero/zones/nexus/nodes/TargetAccelerationNode.h>

#include <zero/zones/nexus/Nexus.h>
#include "NexusTeamBehaviorV2.h"

#include <algorithm>

using namespace zero::svs;

namespace zero {
namespace nexus {

// Combat structure and every tuning constant below (except pure aiming/collision geometry, which
// replay data can't speak to) comes from ZeroReplayAnalyzer, a standalone tool (outside this repo)
// that parses SubspaceServer .replay recordings via SubspaceServer's own Replay/Packets/Core/
// Utilities projects, run against 21 real 4v4 SVS league matches (~2M closing samples, ~1.5M
// retreating samples, ~75k bullet-fire samples, 283 kills with killer/target context). Distances
// are already converted from the replay's raw protocol position units into zero's own position
// units (PlayerManager divides raw units by 16.0f parsing S2C position packets), so they compare
// directly against FoursBehavior.cpp's constants without further conversion.
//
// Structural difference from FoursBehavior.cpp: that tree decided "rush", "press the advantage",
// "energy disadvantaged", and "just orbit" as four separately-triggered, independently-ordered
// Sequences, each re-deriving its own overlapping distance/energy conditions, and gated every
// weapon's fire sequence behind its own ad hoc energy/distance checks. Here, EngagementPhaseNode
// (zero/zones/nexus/nodes/EngagementPhaseNode.h) classifies the tick into one phase up front - the
// data actually falls into those same regimes, but now movement and weapon gating both dispatch
// off that single value instead of duplicating the classification logic in multiple places.
// WeaponEnergyBandNode (zero/zones/nexus/nodes/WeaponEnergyBandNode.h) replaces the scattered
// per-weapon energy/distance checks with the empirically observed band for each weapon.
std::unique_ptr<behavior::BehaviorNode> CreateNexusTeamTreeV2(behavior::ExecuteContext& ctx,
                                                              const NexusTeamConfigV2& config) {
  using namespace behavior;

  BehaviorBuilder builder;

  // Every value below comes from the per-mode config (NexusTeamConfigV2.h), which is also where the
  // replay-analysis provenance for each number lives. They're bound to locals here only so the tree
  // body stays readable - node constructors copy these by value, so the config does not need to
  // outlive tree construction.
  const char* queue_command = config.queue_command;

  const float kLowEnergyDistanceThreshold = config.low_energy_distance_threshold;
  const float kLowEnergyTargetPercent = config.low_energy_target_percent;

  const float kFinishTargetPercent = config.finish_target_percent;
  const float kFinishDistance = config.finish_distance;
  const float kPressDistance = config.press_distance;
  const u32 kPressTicks = config.press_ticks;
  const float kDisadvantageEnterRatio = config.disadvantage_enter_ratio;
  const float kDisadvantageExitRatio = config.disadvantage_exit_ratio;
  const float kCriticalEnergyPercent = config.critical_energy_percent;
  const float kFinishMinSelfEnergyPercent = config.finish_min_self_energy_percent;
  const u32 kFinishRepelThreshold = config.finish_repel_threshold;

  const float kOrbitDistance = config.orbit_distance;
  const float kLeashDistance = config.leash_distance;
  const float kTeamRange = config.team_range;
  const float kAvoidTeamDistance = config.avoid_team_distance;

  const float kSwarmRadius = config.swarm_radius;
  const float kCommitAdvantage = config.commit_advantage;
  const float kBreakOffAdvantage = config.break_off_advantage;

  const float kRepelDetectionDistance = config.repel_detection_distance;
  const float kBombRequiredDamageOverlap = config.bomb_required_damage_overlap;
  const float kShotSpreadDistanceThreshold = config.shot_spread_distance_threshold;
  const float kAimLeadBiasSeconds = config.aim_lead_bias_seconds;
  const float kShotSpreadManeuveringNormalizer = config.shot_spread_maneuvering_normalizer;

  const float kEnclosureProbeDistance = config.enclosure_probe_distance;
  const float kSelfMinOpenFraction = config.self_min_open_fraction;
  const float kSelfMinEscapeArc = config.self_min_escape_arc;
  const float kTrapTargetOpenFraction = config.trap_target_open_fraction;
  const float kTrapTargetEscapeArc = config.trap_target_escape_arc;
  const float kTrapBlockDistance = config.trap_block_distance;

  const float kBulletMinEnergyPercent = config.bullet_min_energy_percent;
  const float kBombMinEnergyPercent = config.bomb_min_energy_percent;
  const float kBombMaxDistance = config.bomb_max_distance;
  const float kBombSelfGuardDistance = config.bomb_self_guard_distance;
  const float kThorMinEnergyPercent = config.thor_min_energy_percent;
  const float kThorMaxDistance = config.thor_max_distance;
  const u32 kThorCooldownTicks = config.thor_cooldown_ticks;
  const float kDecoyMinEnergyPercent = config.decoy_min_energy_percent;

  // Never ask for a teammate the roster cannot contain - in twos there is no second-nearest
  // teammate, so an unclamped factor would make the regroup branch permanently unreachable.
  const u32 kRegroupTeammateFactor =
      config.team_size > 1 ? std::min(config.regroup_teammate_factor, config.team_size - 1) : 1;

  // Bombs are herding tools, not sniping tools: across 18k bomb shots the launch heading sat a
  // median of +12 degrees (mean +11, p75 +39) toward the side the target was already sliding, and
  // targets being bombed were moving laterally at a median of 6.9 tiles/sec. Below that lateral
  // speed there's nothing to cut off, so fall through to a plain intercept shot instead.
  const float kBombHerdLeadSeconds = config.bomb_herd_lead_seconds;
  const float kBombHerdMinLateralSpeed = config.bomb_herd_min_lateral_speed;

  // --- Cadence. FoursBehavior.cpp gated firing behind an artificial 30-tick burst window followed
  // by a forced 100-tick cooldown. Real cadence doesn't show that shape: only 2.4% of inter-shot
  // gaps were under 0.2s, so a synthetic extra throttle beyond the weapon's own natural cooldown
  // isn't earning its complexity. What the data does support is a distinct "lull" band (67.6% of
  // gaps fell between 0.2-1.0s, ~19% were 3s+): once it's been a while since the last shot, go
  // broadside instead of continuing to track the target.
  const u32 kFireLullTicks = config.fire_lull_ticks;

  // clang-format off
  builder
    .Selector()
        .Sequence() // Join the queue first thing.
            .Child<TimerExpiredNode>("queue")
            .Child<ChatMessageNode>(ChatMessageNode::Public(queue_command))
            .Child<ChatMessageNode>(ChatMessageNode::Public("?return"))
            .Child<TimerSetNode>("queue", 6000)
            .End()
        .Sequence() // Don't do anything while in spec.
            .Child<PlayerFrequencyQueryNode>("self_freq")
            .Child<EqualityNode<u16>>("self_freq", 8025)
            .Child<ScalarNode>(1.0f, "spectating")
            .End()
        .Sequence() // Match startup begins when we get taken out of spec.
            .Child<BlackboardSetQueryNode>("spectating")
            .Child<TimerSetNode>("match_startup", 3000)
            .Child<BlackboardEraseNode>("spectating")
            .End()
        .Sequence() // Enter the specified ship if not already in it and have been taken out of spec.
            .InvertChild<TimerExpiredNode>("match_startup")
            .InvertChild<ShipQueryNode>("request_ship")
            .Child<ShipRequestNode>("request_ship")
            .End()
        .Sequence() // Fire 1 startup shot and set target position so we can get out of the ready check loop.
            .InvertChild<TimerExpiredNode>("match_startup")
            .Child<TimerExpiredNode>("pre_fire")
            .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bullet)
            .Child<InputActionNode>(InputAction::Bullet)
            .Child<TimerSetNode>("pre_fire", 1500)
            .End()
        .Sequence() // Attach if someone is safe and we have full energy.
            .Child<BlackboardSetQueryNode>("tchat_safe")
            .InvertChild<TimerExpiredNode>("tchat_safe_timer")
            .Child<PlayerEnergyPercentThresholdNode>(1.0f)
            .Child<TimerExpiredNode>("attach_cooldown")
            .InvertChild<AttachedQueryNode>("self")
            .Child<NearestTeammateNode>("nearest_teammate")
            .Child<PlayerPositionQueryNode>("nearest_teammate", "nearest_teammate_position")
            .Child<DistanceThresholdNode>("nearest_teammate_position", kTeamRange)
            .Child<PlayerByNameNode>("tchat_safe", "tchat_safe_player")
            .Child<AttachNode>("tchat_safe_player")
            .Child<TimerSetNode>("attach_cooldown", 100)
            .Child<BlackboardEraseNode>("tchat_safe")
            .Child<BlackboardEraseNode>("tchat_safe_timer")
            .End()
        .Sequence() // Detach if attached.
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
                        .Child<PlayerPositionQueryNode>("target", "nearest_target_position")
                        .Child<PredictiveAimNode>(WeaponType::Bullet, "target", "target_acceleration", "nearest_aimshot", kAimLeadBiasSeconds)
                        .End()
                    .Sequence(CompositeDecorator::Success) // Prefer the enemy the team should collapse on.
                        .Child<TimerExpiredNode>("recharge_timer")
                        .Child<IsolatedTargetNode>("target", kSwarmRadius)
                        .Child<PlayerPositionQueryNode>("target", "target_position")
                        .Child<PlayerEnergyQueryNode>("target", "target_energy")
                        .Child<TargetAccelerationNode>("target", "target_acceleration")
                        .Child<PredictiveAimNode>(WeaponType::Bullet, "target", "target_acceleration", "aimshot", kAimLeadBiasSeconds)
                        .End()
                    .Sequence(CompositeDecorator::Success) // Always finish a nearby, nearly-dead enemy over anything else.
                        .Child<LowestTargetNode>("lowest_target")
                        .Child<PlayerPositionQueryNode>("lowest_target", "lowest_target_position")
                        .Child<PlayerEnergyQueryNode>("lowest_target", "lowest_target_energy")
                        .InvertChild<DistanceThresholdNode>("lowest_target_position", "self_position", kLowEnergyDistanceThreshold)
                        .Child<TargetEnergyPercentThresholdNode>("lowest_target", "lowest_target_energy", kLowEnergyTargetPercent)
                        .Child<LowestTargetNode>("target")
                        .Child<PlayerPositionQueryNode>("target", "target_position")
                        .Child<PlayerEnergyQueryNode>("target", "target_energy")
                        .Child<TargetAccelerationNode>("target", "target_acceleration")
                        .Child<PredictiveAimNode>(WeaponType::Bullet, "target", "target_acceleration", "aimshot", kAimLeadBiasSeconds)
                        .End()
                    .End()
                // Local head count and engagement phase are both read further down this same tick
                // (multifire, movement, weapon gating), so they must be computed before any of
                // those - a consumer that runs first would silently be reading last tick's value.
                // Success-wrapped so a momentarily invalid self/target drops us through to
                // waypoints rather than aborting the whole combat branch.
                .Sequence(CompositeDecorator::Success)
                    .Child<TeamAdvantageNode>(kSwarmRadius, "team_advantage")
                    .End()
                .Sequence(CompositeDecorator::Success)
                    .Child<EngagementPhaseNode>("target", "target_energy", "target_energy_prev", "phase",
                                                 "energy_disadvantaged", "press_until",
                                                 kFinishTargetPercent, kFinishDistance, kPressDistance, kPressTicks,
                                                 kDisadvantageEnterRatio, kDisadvantageExitRatio, kCriticalEnergyPercent)
                    .End()
                .Sequence(CompositeDecorator::Success) // Lay a portal down if we have one but no location.
                    .Child<ShipItemCountThresholdNode>(ShipItemType::Portal, 1)
                    .InvertChild<ShipPortalPositionQueryNode>()
                    .Child<InputActionNode>(InputAction::Portal)
                    .End()
                .Selector(CompositeDecorator::Success) // Multifire toggle based on range - mechanical capability, not a behavior pattern.
                    .Sequence()
                        .Child<ShipCapabilityQueryNode>(ShipCapability_Multifire)
                        .Child<DistanceThresholdNode>("target_position", 35.0f)
                        .InvertChild<ShipMultifireQueryNode>()
                        .Child<EqualityNode<EngagementPhase>>(EngagementPhase::Engaged, "phase")
                        .Child<InputActionNode>(InputAction::Multifire)
                        .End()
                    .Sequence()
                        .Child<ShipCapabilityQueryNode>(ShipCapability_Multifire)
                        .InvertChild<DistanceThresholdNode>("target_position", 35.0f)
                        .Child<ShipMultifireQueryNode>()
                        .Child<InputActionNode>(InputAction::Multifire)
                        .End()
                    .End()
                .Selector(CompositeDecorator::Success) // Antiwarp toggle based on energy.
                    .Sequence()
                        .Child<TimerExpiredNode>("tchat_safe_timer")
                        .Child<ShipCapabilityQueryNode>(ShipCapability_Antiwarp)
                        .Child<PlayerEnergyPercentThresholdNode>(0.75f)
                        .InvertChild<PlayerStatusQueryNode>(Status_Antiwarp)
                        .Child<InputActionNode>(InputAction::Antiwarp)
                        .End()
                    .Sequence()
                        .Child<ShipCapabilityQueryNode>(ShipCapability_Antiwarp)
                        .InvertChild<PlayerEnergyPercentThresholdNode>(0.75f)
                        .Child<PlayerStatusQueryNode>(Status_Antiwarp)
                        .Child<InputActionNode>(InputAction::Antiwarp)
                        .End()
                    .End()
                .Sequence(CompositeDecorator::Success) // Re-arm the recharge/retreat timer for as long as we're disadvantaged.
                    .Child<EqualityNode<EngagementPhase>>(EngagementPhase::Disadvantaged, "phase")
                    .Child<TimerSetNode>("recharge_timer", 200)
                    .End()
                .Selector()
                    .Sequence() // Dodge and use defensive items - always live, except don't bother dodging while finishing a near-dead target.
                        .Sequence(CompositeDecorator::Success)
                            .Child<IncomingDamageQueryNode>(kRepelDetectionDistance, "incoming_damage")
                            .Child<PlayerCurrentEnergyQueryNode>("self_energy")
                            .End()
                        .Sequence(CompositeDecorator::Success) // Portal out if in danger and out of repels.
                            .InvertChild<ShipItemCountThresholdNode>(ShipItemType::Repel)
                            .Child<ShipPortalPositionQueryNode>()
                            .Child<ScalarThresholdNode<float>>("incoming_damage", "self_energy")
                            .Child<TimerExpiredNode>("defense_timer")
                            .Child<InputActionNode>(InputAction::Warp)
                            .Child<TimerSetNode>("defense_timer", 100)
                            .End()
                        .Sequence(CompositeDecorator::Success) // Repel when in danger.
                            .Child<ShipWeaponCapabilityQueryNode>(WeaponType::Repel)
                            .Child<TimerExpiredNode>("defense_timer")
                            .Child<ScalarThresholdNode<float>>("incoming_damage", "self_energy")
                            .Child<InputActionNode>(InputAction::Repel)
                            .Child<TimerSetNode>("defense_timer", 100)
                            .End()
                        .InvertChild<EqualityNode<EngagementPhase>>(EngagementPhase::Finish, "phase")
                        .Child<DodgeIncomingDamage>(0.2f, 30.0f)
                        .End()
                    .Sequence() // Keep distance from the target during ready-check.
                        .InvertChild<TimerExpiredNode>("match_startup")
                        .Selector()
                            .Child<WallEscapeNode>(kEnclosureProbeDistance, kSelfMinOpenFraction, kSelfMinEscapeArc)
                            .Child<FleeNode>("nearest_target_position", kLeashDistance, 5.0f, 0.2f, "target_energy")
                            .End()
                        .End()
                    .Sequence() // Disadvantaged - keep distance and recharge instead of fighting.
                        .Child<EqualityNode<EngagementPhase>>(EngagementPhase::Disadvantaged, "phase")
                        .Selector()
                            .Child<WallEscapeNode>(kEnclosureProbeDistance, kSelfMinOpenFraction, kSelfMinEscapeArc)
                            .Child<FleeNode>("nearest_aimshot", kLeashDistance, 5.0f, 0.2f, "target_energy")
                            .End()
                        .End()
                    .Sequence() // Path to target if not immediately visible.
                        .InvertChild<VisibilityQueryNode>("target_position")
                        .Child<GoToNode>("target_position")
                        .Child<AvoidTeamNode>(kAvoidTeamDistance)
                        .End()
                    .Sequence() // Finish/Press/Engaged - aim, move, and fire.
                        .Child<TimerExpiredNode>("match_startup")
                        .Sequence(CompositeDecorator::Success)
                            .Child<DistanceThresholdNode>("target_position", kShotSpreadDistanceThreshold)
                            .Child<ShotSpreadNode>("aimshot", 3.0f, 1.0f, "target_acceleration", kShotSpreadManeuveringNormalizer)
                            .End()
                        .Parallel()
                            .Selector() // Face the target, or go broadside during a firing lull while orbiting.
                                .Sequence()
                                    .Child<EqualityNode<EngagementPhase>>(EngagementPhase::Engaged, "phase")
                                    .InvertChild<DistanceThresholdNode>("target_position", "self_position", kOrbitDistance)
                                    .Child<TimerExpiredNode>("fire_lull")
                                    .Child<BroadsideFaceNode>("target_position")
                                    .End()
                                .Child<FaceNode>("aimshot")
                                .End()
                            .Sequence(CompositeDecorator::Success)
                                .Child<DodgeJukeNode>(30.0f)
                                .End()
                            // Decoy is an action, not a movement choice, so it runs on its own
                            // rather than as a branch of the movement Selector below - a
                            // Success-decorated sequence sitting inside that Selector would always
                            // report Success and starve every movement branch after it.
                            .Sequence(CompositeDecorator::Success)
                                .Child<EqualityNode<EngagementPhase>>(EngagementPhase::Engaged, "phase")
                                .InvertChild<ShipItemCountThresholdNode>(ShipItemType::Repel)
                                .Child<ShipWeaponCapabilityQueryNode>(WeaponType::Decoy)
                                .Child<WeaponEnergyBandNode>(kDecoyMinEnergyPercent)
                                .Child<TimerExpiredNode>("decoy_timer")
                                .Child<InputActionNode>(InputAction::Decoy)
                                .Child<TimerSetNode>("decoy_timer", 850)
                                .End()
                            .Selector() // Movement. Every branch is undecorated so the Selector actually falls through.
                                .Sequence() // Getting boxed in ourselves outranks anything offensive - break out first.
                                    .Child<WallEscapeNode>(kEnclosureProbeDistance, kSelfMinOpenFraction, kSelfMinEscapeArc)
                                    .Child<BlackboardEraseNode>("orbit_direction")
                                    .End()
                                .Sequence() // The target is cornered - plug their only way out instead of circling them.
                                    .InvertChild<EqualityNode<EngagementPhase>>(EngagementPhase::Disadvantaged, "phase")
                                    // A cornered target that is also nearly dead should just be
                                    // killed - blocking their exit instead would hand them the
                                    // recharge time we were trying to deny them.
                                    .InvertChild<EqualityNode<EngagementPhase>>(EngagementPhase::Finish, "phase")
                                    .Child<ScalarThresholdNode<float>>("team_advantage", kCommitAdvantage)
                                    .Child<TrapTargetNode>("target", "trap_position", kEnclosureProbeDistance,
                                                            kTrapTargetOpenFraction, kTrapTargetEscapeArc, kTrapBlockDistance)
                                    .Child<SeekNode>("trap_position", 0.0f, SeekNode::DistanceResolveType::Zero)
                                    .Child<AvoidTeamNode>(kAvoidTeamDistance)
                                    .Child<BlackboardEraseNode>("orbit_direction")
                                    .End()
                                .Sequence() // Finish - commit to the kill regardless of local numbers.
                                    .Child<EqualityNode<EngagementPhase>>(EngagementPhase::Finish, "phase")
                                    .Child<PlayerEnergyPercentThresholdNode>(kFinishMinSelfEnergyPercent)
                                    .Child<SeekNode>("aimshot", 0.0f, SeekNode::DistanceResolveType::Static)
                                    .Child<BlackboardEraseNode>("orbit_direction")
                                    .Sequence(CompositeDecorator::Success) // Rockets only while actually finishing.
                                        .Child<ShipItemCountThresholdNode>(ShipItemType::Repel, kFinishRepelThreshold)
                                        .Child<ShipItemCountThresholdNode>(ShipItemType::Rocket)
                                        .Child<PlayerEnergyPercentThresholdNode>(0.6f)
                                        .InvertChild<DistanceThresholdNode>("target_position", 30.0f)
                                        .Child<DistanceThresholdNode>("target_position", 10.0f)
                                        .Child<TimerExpiredNode>("rocket_timer")
                                        .Child<InputActionNode>(InputAction::Rocket)
                                        .Child<TimerSetNode>("rocket_timer", 1500)
                                        .End()
                                    .End()
                                .Sequence() // Press - close in, but only while not locally outnumbered.
                                    .Child<EqualityNode<EngagementPhase>>(EngagementPhase::Press, "phase")
                                    .Child<PlayerEnergyPercentThresholdNode>(kFinishMinSelfEnergyPercent)
                                    .Child<ScalarThresholdNode<float>>("team_advantage", kCommitAdvantage)
                                    .Child<SeekNode>("aimshot", 0.0f, SeekNode::DistanceResolveType::Static)
                                    .Child<BlackboardEraseNode>("orbit_direction")
                                    .End()
                                .Sequence() // Locally outnumbered - fall back onto the nearest teammate rather than trading alone.
                                    .InvertChild<ScalarThresholdNode<float>>("team_advantage", kBreakOffAdvantage)
                                    .Child<NearestTeammateNode>("nearest_teammate")
                                    .Child<PlayerPositionQueryNode>("nearest_teammate", "nearest_teammate_position")
                                    .Child<DistanceThresholdNode>("nearest_teammate_position", kAvoidTeamDistance)
                                    .Child<BlackboardEraseNode>("orbit_direction")
                                    .Child<GoToNode>("nearest_teammate_position")
                                    .End()
                                .Sequence() // Regroup with the wider team when we've drifted off alone.
                                    .Child<NearestTeammateNode>("nearest_teammate", kRegroupTeammateFactor)
                                    .Child<PlayerPositionQueryNode>("nearest_teammate", "nearest_teammate_position")
                                    .Child<DistanceThresholdNode>("nearest_teammate_position", kTeamRange)
                                    .Child<GoToNode>("nearest_teammate_position")
                                    .End()
                                .Sequence() // Close the gap while still outside fighting range.
                                    .Child<DistanceThresholdNode>("target_position", "self_position", kOrbitDistance)
                                    .Child<SeekNode>("aimshot", 0.0f, SeekNode::DistanceResolveType::Zero)
                                    .Child<AvoidTeamNode>(kAvoidTeamDistance)
                                    .End()
                                .Sequence() // In range - circle instead of sitting still or crowding in.
                                    .Child<OrbitNode>("aimshot", kOrbitDistance, "orbit_direction")
                                    .Child<AvoidTeamNode>(kAvoidTeamDistance)
                                    .End()
                                .End()
                            .Sequence(CompositeDecorator::Success) // Bomb fire check.
                                .Child<TimerExpiredNode>("match_startup")
                                .InvertChild<EqualityNode<EngagementPhase>>(EngagementPhase::Disadvantaged, "phase")
                                .Child<VectorSubtractNode>("aimshot", "self_position", "target_direction", true)
                                .Child<PlayerVelocityQueryNode>("self_velocity")
                                .Child<VectorDotNode>("self_velocity", "target_direction", "forward_velocity")
                                .Child<ScalarThresholdNode<float>>("forward_velocity", 2.0f)
                                .Child<WeaponEnergyBandNode>(kBombMinEnergyPercent, "target_position", kBombMaxDistance)
                                .Child<ShipWeaponCapabilityQueryNode>(WeaponType::Bomb)
                                .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bomb)
                                .InvertChild<InputQueryNode>(InputAction::Thor)
                                .Child<IncomingDamageQueryNode>("target", kRepelDetectionDistance * 2.5f, 2.75f, "outgoing_damage")
                                .Child<ScalarThresholdNode<float>>("outgoing_damage", kBombRequiredDamageOverlap)
                                .Child<DistanceThresholdNode>("nearest_target_position", kBombSelfGuardDistance)
                                .Child<NearestTeammatePlayerPositionQueryNode>("target", "target_nearest_teammate_position")
                                .Child<DistanceThresholdNode>("target_position", "target_nearest_teammate_position", kBombSelfGuardDistance)
                                // Throw the bomb ahead of where the target is sliding so the blast
                                // denies that lane, instead of trying to snipe them with it. Falls
                                // back to the plain intercept point when they aren't moving
                                // laterally enough to be herded anywhere.
                                .Child<HerdingBombAimNode>("target", "bomb_aimshot", "aimshot", kBombHerdLeadSeconds, kBombHerdMinLateralSpeed)
                                .Child<ShotVelocityQueryNode>(WeaponType::Bomb, "bomb_fire_velocity")
                                .Child<RayNode>("self_position", "bomb_fire_velocity", "bomb_fire_ray")
                                .Child<DynamicPlayerBoundingBoxQueryNode>("target", "target_bounds", 8.0f)
                                .Child<MoveRectangleNode>("target_bounds", "bomb_aimshot", "target_bounds")
                                .Child<RayRectangleInterceptNode>("bomb_fire_ray", "target_bounds")
                                .Child<InputActionNode>(InputAction::Bomb)
                                .End()
                            .Sequence(CompositeDecorator::Success) // Thor fire check - offensive area-denial at moderate range and healthy energy.
                                .Child<TimerExpiredNode>("match_startup")
                                .Child<WeaponEnergyBandNode>(kThorMinEnergyPercent, "target_position", kThorMaxDistance)
                                .Child<ShipWeaponCapabilityQueryNode>(WeaponType::Thor)
                                .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Thor)
                                .Child<TimerExpiredNode>("thor_timer") // thors are a scarce match resource, not a per-cooldown weapon
                                .InvertChild<InputQueryNode>(InputAction::Bomb)
                                .Child<ShotVelocityQueryNode>(WeaponType::Thor, "thor_fire_velocity")
                                .Child<RayNode>("self_position", "thor_fire_velocity", "thor_fire_ray")
                                .Child<DynamicPlayerBoundingBoxQueryNode>("target", "target_bounds", 4.0f)
                                .Child<MoveRectangleNode>("target_bounds", "aimshot", "target_bounds")
                                .Child<RayRectangleInterceptNode>("thor_fire_ray", "target_bounds")
                                .Child<InputActionNode>(InputAction::Thor)
                                .Child<TimerSetNode>("thor_timer", kThorCooldownTicks)
                                .End()
                            .Sequence(CompositeDecorator::Success) // Bullet fire check.
                                .Child<TimerExpiredNode>("match_startup")
                                .InvertChild<EqualityNode<EngagementPhase>>(EngagementPhase::Disadvantaged, "phase")
                                .Child<DynamicPlayerBoundingBoxQueryNode>("target", "target_bounds", 4.0f)
                                .Child<MoveRectangleNode>("target_bounds", "aimshot", "target_bounds")
                                .Selector()
                                    .Child<EqualityNode<EngagementPhase>>(EngagementPhase::Finish, "phase")
                                    .Child<EqualityNode<EngagementPhase>>(EngagementPhase::Press, "phase")
                                    .Child<WeaponEnergyBandNode>(kBulletMinEnergyPercent)
                                    .End()
                                .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bullet)
                                .InvertChild<InputQueryNode>(InputAction::Bomb)
                                .InvertChild<TileQueryNode>(kTileIdSafe)
                                .Child<ShotVelocityQueryNode>(WeaponType::Bullet, "bullet_fire_velocity")
                                .Child<RayNode>("self_position", "bullet_fire_velocity", "bullet_fire_ray")
                                .Child<DynamicPlayerBoundingBoxQueryNode>("target", "target_bounds", 4.0f)
                                .Child<MoveRectangleNode>("target_bounds", "aimshot", "target_bounds")
                                .Child<RayRectangleInterceptNode>("bullet_fire_ray", "target_bounds")
                                .Child<InputActionNode>(InputAction::Bullet)
                                .Child<TimerSetNode>("fire_lull", kFireLullTicks) // reset the "time since last shot" clock the broadside selector checks above
                                .End()
                            .Child<ScalarNode>("target_energy", "target_energy_prev")
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
