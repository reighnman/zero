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
#include <zero/zones/nexus/nodes/NearestTeammatePlayerPositionQueryNode.h>
#include <zero/zones/nexus/nodes/EnergyDisadvantageNode.h>
#include <zero/zones/nexus/nodes/LowestTargetNode.h>
#include <zero/zones/nexus/nodes/OrbitNode.h>
#include <zero/zones/trenchwars/nodes/AttachNode.h>
#include <zero/zones/nexus/nodes/PlayerByNameNode.h>
#include <zero/zones/nexus/nodes/PredictiveAimNode.h>
#include <zero/zones/nexus/nodes/ShotSpreadNode.h>
#include <zero/zones/nexus/nodes/TargetAccelerationNode.h>
#include <zero/zones/nexus/nodes/FleeNode.h>
#include <zero/zones/nexus/nodes/WallAvoidanceNode.h>
#include <zero/zones/nexus/nodes/BounceRayRectangleInterceptNode.h>

#include <zero/zones/nexus/Nexus.h>
#include "TwosBoxBehavior.h"


using namespace zero::svs;

namespace zero {
namespace nexus {

//TODO WIP
  struct Placeholder : public behavior::BehaviorNode {
    Placeholder(const char* something) : something(something) {}
        
    behavior::ExecuteResult Execute(behavior::ExecuteContext & ctx) override {
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

std::unique_ptr<behavior::BehaviorNode> TwosBoxBehavior::CreateTree(behavior::ExecuteContext& ctx) {
  using namespace behavior;

  BehaviorBuilder builder;

  const Vector2f center(512, 512);

  // Pursue targets below these thresholds, otherwise attack from distance
  constexpr float kLowEnergyThreshold = 700.0f;     // Energy threshold
  constexpr float kLowEnergyDistanceThreshold = 18.0f;  // Distance threshold
  constexpr u32 kRushRepelThreshold = 1;  // If we don't have this many reps dont rush targets (in testing)

  // Stop attempting to dodge when target is below these thresholds
  constexpr float kLowEnergyRushThreshold = 300.0f;  // Energy threshold
  constexpr float kRushDistanceThreshold = 8.0f;     // Distance threshold

  // Other dodge
  constexpr float kDodgeVelocityThreshold = 10.0f;
  constexpr float kDodgeRangeSlow = 20.0f;
  constexpr float kDodgeRangeFast = 45.0f;  // When

  // Check for incoming damage within this range, if greater than current energy rep
  constexpr float kRepelDistance = 9.0f;

  // How much damage that is going towards an enemy before we start bombing. This is to limit the frequency of our
  // bombing so it overlaps bullets and is harder to dodge.
  constexpr float kBombRequiredDamageOverlap = 300.0f;

  // How far away a target needs to be before we start varying our shots around the target.
  constexpr float kShotSpreadDistanceThreshold = 40.0f;

  //  If an enemy is near us and we're low energy attempt to pb thor target
  constexpr float kThorEnemyThreshold = 250.0f;

  // Distance away from team before we regroup
  constexpr float kTeamRange = 40.0f;       // Distance away from team member before we regroup
  constexpr int kTeamRangeMemberIndex = 3;  // Use the 3rd furthest teammate as the base, otherwise the next furthest

  // Leash Settings
  constexpr float kLeashDistance = 30.0f;        // Used for low-energy retreating
  constexpr float kLeashDistanceAttack = 20.0f;  // Used for default attack distance

  // Once within this distance of the nearest enemy, stop closing further and circle instead -
  // close enough that they'll eventually fail to dodge a lobbed shot and we can dive in, far
  // enough to have room to maneuver instead of colliding.
  constexpr float kOrbitDistance = 15.0f;

  // Bomb hitbox tolerance multiplier while orbiting - bigger than the bullet/thor multiplier below
  // so bombs land as area denial off a near miss instead of needing a precise direct hit, like
  // lobbing them into blast range instead of sniping with them.
  constexpr float kBombProximityMultiplier = 8.0f;

  // Fire bullets in short windows instead of spraying continuously while orbiting - a burst this
  // long, then a forced pause this long before the next one. Bypassed entirely once rushing.
  constexpr u32 kBurstFireDurationTicks = 30;   // ~0.3s of allowed fire
  constexpr u32 kBurstFireCooldownTicks = 100;  // ~1s forced pause after

  // How long to keep pressing an advantage after the target loses energy (hit or spent shooting)
  // while we still have more than they do - a sustained window instead of a single-tick reaction,
  // since target_energy_prev only differs from target_energy for the one tick the drop happened.
  constexpr u32 kPressAdvantageTicks = 300;  // ~3s

  // How close a wall needs to be before we override movement to steer clear of it while fleeing.
  constexpr float kWallCheckDistance = 5.0f;
  // How far out to search for an opening once a wall is too close.
  constexpr float kWallOpeningDistance = 35.0f;

  // Misc
  constexpr float kAvoidTeamDistance = 8.0f;    // Check to ensure we're not all stacked
  constexpr float kAvoidEnemyDistance = 10.0f;  // Additional check when pathing to prevent enemies from sitting on us
  constexpr float kMultiFireDistance = 35.0f;   // Use multifire for targets over this range
  constexpr float kAvoidWallDistance = 3.0f;

  // How far ahead (in seconds worth of their smoothed acceleration) to bend predicted aim toward
  // where the target is actually trending, instead of assuming they hold their current velocity.
  constexpr float kAimLeadBiasSeconds = 0.2f;

  // Acceleration magnitude (units/sec^2) treated as "fully erratic" for shot spread purposes - a
  // target maneuvering at or above this gets the full spread, below it gets scaled-down spread.
  // Starting guess, needs tuning against real play.
  constexpr float kShotSpreadManeuveringNormalizer = 4.0f;

  // Enter a defensive (recharging) state once our energy drops below this fraction of the
  // target's estimated energy, and don't leave it again until we recover past the higher exit
  // ratio - the gap between the two is a hysteresis band so we don't flicker near parity.
  constexpr float kEnergyDisadvantageEnterRatio = 0.65f;
  constexpr float kEnergyDisadvantageExitRatio = 0.9f;
  // Always treat energy this low as a disadvantage regardless of the target's energy, since being
  // critically low is dangerous even against an equally weak target.
  constexpr float kCriticalEnergyPercent = 0.094f;

  // clang-format off
  builder
    .Selector()
         .InvertChild<PlayerSelfNode>("self")
        .Sequence() //Join the queue first thing and auto join TODO: add command or checks to requeue if something goes wrong later such as recycled arena
            //.InvertChild<BlackboardSetQueryNode>("queued") //Check if we have already joined the queue, if not join
            .Child<TimerExpiredNode>("queue")
            .Child<ChatMessageNode>(ChatMessageNode::Public("?next 3v3pub")) // Invert so this fails and freq is reevaluated.  //TODO replace this with a config var so we dont need 4 behaviors
            .Child<ChatMessageNode>(ChatMessageNode::Public("?return")) // Invert so this fails and freq is reevaluated.  //TODO replace this with a config var so we dont need 4 behaviors
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
            .Child<TimerSetNode>("match_startup", 600)  //Trigger match start timer (assumming 3 sec + however long it takes the other person to ready up)
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
                    .Child<PlayerPositionQueryNode>("self_position") //Always track self position
                    .Child<NearestMemoryTargetNode>("nearest_enemy") //Always track nearest enemey
                    .Child<PlayerPositionQueryNode>("nearest_enemy", "nearest_enemy_position") //Always track nearest enemy position so we can use it for some checks
                    .Child<PlayerEnergyQueryNode>("nearest_enemy", "nearest_enemy_energy")
                    .Child<TargetAccelerationNode>("nearest_enemy", "nearest_enemy_acceleration")
                    .Child<PredictiveAimNode>(WeaponType::Bullet, "nearest_enemy", "nearest_enemy_acceleration", "nearest_aimshot", kAimLeadBiasSeconds)
                    .Sequence() // Default targert is nearest
                        .Child<NearestMemoryTargetNode>("target")
                        .Child<PlayerPositionQueryNode>("target", "target_position")
                        .Child<PlayerEnergyQueryNode>("target", "target_energy")
                        .Child<TargetAccelerationNode>("target", "target_acceleration")
                        .Child<PredictiveAimNode>(WeaponType::Bullet, "target", "target_acceleration", "aimshot", kAimLeadBiasSeconds)
                        .End()
                     .Sequence() // If is someone low nearby override target instead of just using nearest
                        .Child<TimerExpiredNode>("recharge_timer") //if we're recharging we should always be leashing to nearest enemy so ignore low health
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
                        .End()
                .End()
                .Sequence(CompositeDecorator::Success) // If we have a portal but no location, lay one down.
                    .Child<ShipItemCountThresholdNode>(ShipItemType::Portal, 1)
                    .InvertChild<ShipPortalPositionQueryNode>()
                    .Child<InputActionNode>(InputAction::Portal)
                    .End()
                .Selector(CompositeDecorator::Success) // Enable multifire if ship supports it and it's disabled.
                    .Sequence()
                        .Child<ShipCapabilityQueryNode>(ShipCapability_Multifire)
                        .Child<DistanceThresholdNode>("target_position", kMultiFireDistance) // If we are far from enemy, use multifire
                        .InvertChild<ShipMultifireQueryNode>()  //Check if multifire is off
                        .InvertChild<BlackboardSetQueryNode>("rushing") //dont multi if rushing
                        .Child<InputActionNode>(InputAction::Multifire) //Turn on multifire
                        .End()
                    .Sequence()
                        .Child<ShipCapabilityQueryNode>(ShipCapability_Multifire)
                        .InvertChild<DistanceThresholdNode>("target_position",kMultiFireDistance) // If we are far from enemy, turn off multifire
                        .Child<ShipMultifireQueryNode>()  //Check if multifire is on
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
                .Sequence(CompositeDecorator::Success) // Continuously reassess fight-vs-flee using energy relative to the target, instead of a fixed timer.
                    .Child<EnergyDisadvantageNode>("target", "target_energy", "energy_disadvantaged", kEnergyDisadvantageEnterRatio, kEnergyDisadvantageExitRatio, kCriticalEnergyPercent)
                    .Child<TimerSetNode>("recharge_timer", 200)
                    .End()
                .Selector()
                    .Sequence() // Attempt to dodge and use defensive items.
                        .Sequence(CompositeDecorator::Success) // Always check incoming damage so we can use it in repel and portal sequences.
                            .Child<IncomingDamageQueryNode>(kRepelDistance, "incoming_damage")
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
                        .Sequence(CompositeDecorator::Invert)  // If we're moving fast increase dodge distance threshold
                                .Child<PlayerVelocityQueryNode>("self_velocity")
                                .Child<VectorDotNode>("self_velocity", "target_direction", "forward_velocity")
                                .Child<ScalarThresholdNode<float>>("forward_velocity", kDodgeVelocityThreshold)
                                .Child<DodgeIncomingDamage>(0.2f, kDodgeRangeFast)
                                .End()
                        .Child<DodgeIncomingDamage>(0.1f, kDodgeRangeSlow) //was .3 30
                        .End()
                    .Sequence() // Keep distance from the nearest enemy during ready-check instead of sitting still until the match officially starts.
                        .InvertChild<TimerExpiredNode>("match_startup")
                        .Selector() // Steer clear of nearby walls before fleeing so we don't get pinned in a corner.
                            .Child<WallAvoidanceNode>(kWallCheckDistance, kWallOpeningDistance)
                            .Child<FleeNode>("nearest_enemy_position", kLeashDistance, 5.0f, 0.2f, "target_energy")
                            .End()
                        .End()
                    .Sequence()  //Keep enemy distance while reacharging
                        .InvertChild<TimerExpiredNode>("recharge_timer")
                        // FleeNode handles facing away from the target itself once at leash range, but the dodge
                        // block above still reads a stale "target_direction" from last tick to widen its distance
                        // threshold when moving fast, so keep computing it here.
                        .Child<VectorSubtractNode>("nearest_enemy_position", "self_position", "target_direction", true)
                        .Selector() // Steer clear of nearby walls before fleeing so we don't get pinned in a corner.
                            .Child<WallAvoidanceNode>(kWallCheckDistance, kWallOpeningDistance)
                            .Child<FleeNode>("nearest_aimshot", kLeashDistance, 5.0f, 0.2f, "target_energy")
                            .End()
                        .End()
                    .Sequence() // Path to target if they aren't immediately visible.
                        .Child<TimerExpiredNode>("match_startup")
                        .InvertChild<VisibilityQueryNode>("target_position")
                        .Child<GoToNode>("target_position")
                        .Parallel(CompositeDecorator::Success)
                            .Child<AvoidTeamNode>(kAvoidTeamDistance) //Avoid team while pathing
                            .Child<AvoidEnemyNode>(kAvoidEnemyDistance) //Prevent enemies from sitting on top of us if not the same as the target
                            .End()
                        .Child<RenderPathNode>(Vector3f(1.0f, 0.5f, 0.5f))
                        .Sequence(CompositeDecorator::Success) // Bounce a bullet off a wall to hit a target we can't see directly - bombs don't bounce, so this only applies to bullets.
                            .Child<FaceNode>("aimshot")
                            .Child<PlayerEnergyPercentThresholdNode>(0.3f)
                            .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bullet)
                            .InvertChild<TileQueryNode>(kTileIdSafe)
                            .Child<ShotVelocityQueryNode>(WeaponType::Bullet, "bullet_fire_velocity")
                            .Child<RayNode>("self_position", "bullet_fire_velocity", "bullet_fire_ray")
                            .Child<BulletDistanceNode>("bullet_max_distance")
                            .Child<DynamicPlayerBoundingBoxQueryNode>("target", "target_bounds", 4.0f)
                            .Child<MoveRectangleNode>("target_bounds", "aimshot", "target_bounds")
                            .Child<BounceRayRectangleInterceptNode>("bullet_fire_ray", "target_bounds", "bullet_max_distance")
                            .Child<InputActionNode>(InputAction::Bullet)
                            .End()
                        .End()
                    .Sequence() // Aim at target and shoot while seeking them.
                        .Child<TimerExpiredNode>("match_startup") 
                        .Sequence(CompositeDecorator::Success)
                            .Child<DistanceThresholdNode>("target_position", kShotSpreadDistanceThreshold)
                            .Child<ShotSpreadNode>("aimshot", 3.0f, 1.0f, "target_acceleration", kShotSpreadManeuveringNormalizer)
                            .End()
                        .Parallel()     
                            .Child<FaceNode>("aimshot")
                            .Child<BlackboardEraseNode>("rushing")
                            .Selector()
                               .Sequence() // If there is any low target with in this range prioritize
                                    //.Child<ShipItemCountThresholdNode>(ShipItemType::Repel, kRushRepelThreshold) //dont rush if we have no reps
                                    .InvertChild<DistanceThresholdNode>("target_position", "self_position", kLowEnergyDistanceThreshold)
                                    .InvertChild<ScalarThresholdNode<float>>("target_energy", kLowEnergyThreshold)
                                    .Child<ScalarThresholdNode<float>>("self_energy", "target_energy")
                                    .Child<SeekNode>("aimshot", 0.0f, SeekNode::DistanceResolveType::Static)
                                    .Child<ScalarNode>(1.0f, "rushing")
                                    .Child<BlackboardEraseNode>("recharge_timer")
                                    .Child<BlackboardEraseNode>("orbit_direction") // pick a fresh orbit direction next time we're back to circling
                                    .Sequence(CompositeDecorator::Success) //Optionally rocket if the target is too far and we have decent energy
                                        .Child<ShipItemCountThresholdNode>(ShipItemType::Rocket)
                                        .Child<PlayerEnergyPercentThresholdNode>(0.6f)
                                        .InvertChild<DistanceThresholdNode>("target_position", 30.0f)  //dont rocket if too far away
                                        .Child<DistanceThresholdNode>("target_position", 10.0f)  //dont rocket if right on them you'll overshoot
                                        .Child<TimerExpiredNode>("rocket_timer")
                                        .Child<InputActionNode>(InputAction::Rocket)
                                        .Child<TimerSetNode>("rocket_timer", 2000)
                                        .End() 
                                    .End()
                                .Sequence() // Press the advantage for a while after the target loses energy (hit or spent shooting) and now has meaningfully less than we do.
                                    .InvertChild<DistanceThresholdNode>("target_position", "self_position", kOrbitDistance * 2.0f) //still needs to be a fight we're actually in, not clear across the map
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
                                    .InvertChild<BlackboardSetQueryNode>("rushing")
                                    .Child<BlackboardSetQueryNode>("energy_disadvantaged")  // Set by EnergyDisadvantageNode above, relative to the target instead of a flat self-only threshold.
                                    .Sequence(CompositeDecorator::Success)
                                        .Child<ShipWeaponCapabilityQueryNode>(WeaponType::Decoy)
                                        .Child<TimerExpiredNode>("decoy_timer")
                                        .Child<InputActionNode>(InputAction::Decoy)
                                        .Child<TimerSetNode>("decoy_timer", 1000)
                                        .End()
                                    .End()
                                .Sequence(CompositeDecorator::Success)
                                   .InvertChild<BlackboardSetQueryNode>("rushing")
                                   .Child<AvoidTeamNode>(kAvoidTeamDistance)
                                   .Selector()
                                       .Sequence() // Path to teammate if far away - still faces/fires/juke-dodges via this Parallel instead of running blind.
                                           .Child<NearestTeammateNode>("nearest_teammate", kTeamRangeMemberIndex) //Make sure we have at least 1 teammate close, if more than one stay with the broader group
                                           .Child<PlayerPositionQueryNode>("nearest_teammate", "nearest_teammate_position")
                                           .Child<DistanceThresholdNode>("nearest_teammate_position", kTeamRange) //If we're already near teammates dont run to them
                                           .Child<ScalarThresholdNode<float>>("target_energy", kLowEnergyThreshold)  //If we're going for a kill or someone is diving dont run
                                           .Child<GoToNode>("nearest_teammate_position")
                                           .Child<AvoidEnemyNode>(kAvoidEnemyDistance)
                                           .Child<RenderPathNode>(Vector3f(0.0f, 1.0f, 0.5f))
                                           .End()
                                       .Selector() // Close the gap while still far out, then circle instead of closing all the way to melee range.
                                           .Sequence()
                                               .Child<DistanceThresholdNode>("nearest_enemy_position", "self_position", kOrbitDistance)
                                               .Child<SeekNode>("nearest_aimshot", 0.0f, SeekNode::DistanceResolveType::Zero)
                                               .End()
                                           .Child<OrbitNode>("nearest_aimshot", kOrbitDistance, "orbit_direction")
                                           .End()
                                       .End()
                                   .End()
                                .End()
                            .Sequence(CompositeDecorator::Success) // Bomb fire check.
                                .Child<TimerExpiredNode>("match_startup") 
                                .Child<TimerExpiredNode>("recharge_timer") 
                                .InvertChild<WallAvoidanceNode>(kWallCheckDistance, kWallOpeningDistance)
                                .Child<PlayerEnergyPercentThresholdNode>(0.45f)
                                .Child<ShipWeaponCapabilityQueryNode>(WeaponType::Bomb)
                                .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bomb)
                                .InvertChild<InputQueryNode>(InputAction::Thor)
                                .Child<IncomingDamageQueryNode>("target", kRepelDistance * 2.5f, 2.75f, "outgoing_damage")
                                .Child<ScalarThresholdNode<float>>("outgoing_damage", kBombRequiredDamageOverlap) // Check if we have enough bullets overlapping outgoing damage to fire a bomb into.
                                .InvertChild<DistanceThresholdNode>("target_position", 50.0f)  //dont bomb from too far
                                .Child<DistanceThresholdNode>("nearest_enemy_position", 12.0f)  //check to ensure no enemies are on top of us
                                .Child<NearestTeammatePlayerPositionQueryNode>("target", "target_nearest_teammate_position")
                                .Child<DistanceThresholdNode>("target_position", "target_nearest_teammate_position", 12.0f)  //dont bomb at our target if we or a teammate is near them
                                .Child<ShotVelocityQueryNode>(WeaponType::Bomb, "bomb_fire_velocity")
                                .Child<RayNode>("self_position", "bomb_fire_velocity", "bomb_fire_ray")
                                .Child<DynamicPlayerBoundingBoxQueryNode>("target", "target_bounds", kBombProximityMultiplier) // lob range, not a precise hit
                                .Child<MoveRectangleNode>("target_bounds", "aimshot", "target_bounds")
                                .Child<RenderRectNode>("world_camera", "target_bounds", Vector3f(1.0f, 0.0f, 0.0f))
                                .Child<RenderRayNode>("world_camera", "bomb_fire_ray", 50.0f, Vector3f(1.0f, 0.0f, 0.0f))
                                .Child<RayRectangleInterceptNode>("bomb_fire_ray", "target_bounds")
                                .Child<InputActionNode>(InputAction::Bomb)
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
                                .Child<RenderRectNode>("world_camera", "target_bounds", Vector3f(0.0f, 1.0f, 0.0f))
                                .Child<RenderRayNode>("world_camera", "thor_fire_ray", 50.0f, Vector3f(0.0f, 1.0f, 0.0f))
                                .Child<RayRectangleInterceptNode>("thor_fire_ray", "target_bounds")
                                .Child<InputActionNode>(InputAction::Thor) //Thor
                                .End()
                            .Sequence(CompositeDecorator::Success) // Determine if a shot should be fired by using weapon trajectory and bounding boxes.
                                .Child<TimerExpiredNode>("match_startup")             
                                .Child<TimerExpiredNode>("recharge_timer") 
                                .Child<DynamicPlayerBoundingBoxQueryNode>("target", "target_bounds", 4.0f)
                                .Child<MoveRectangleNode>("target_bounds", "aimshot", "target_bounds")
                                .Child<RenderRectNode>("world_camera", "target_bounds", Vector3f(0.0f, 0.0f, 1.0f))
                                .Selector()
                                    .Child<BlackboardSetQueryNode>("rushing")
                                    .Child<PlayerEnergyPercentThresholdNode>(0.3f)
                                    .End()
                                .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bullet)
                                .InvertChild<InputQueryNode>(InputAction::Bomb) // Don't try to shoot a bullet when shooting a bomb.
                                .InvertChild<TileQueryNode>(kTileIdSafe)
                                .Child<ShotVelocityQueryNode>(WeaponType::Bullet, "bullet_fire_velocity")
                                .Child<RayNode>("self_position", "bullet_fire_velocity", "bullet_fire_ray")
                                .Child<DynamicPlayerBoundingBoxQueryNode>("target", "target_bounds", 4.0f)
                                .Child<MoveRectangleNode>("target_bounds", "aimshot", "target_bounds")
                                .Child<RayRectangleInterceptNode>("bullet_fire_ray", "target_bounds")
                                .Selector() // Fire in short bursts instead of spraying continuously, unless committed to finishing a kill.
                                    .Child<BlackboardSetQueryNode>("rushing")
                                    .InvertChild<TimerExpiredNode>("burst_fire_until") // still inside an active burst window
                                    .Sequence() // Pause between bursts is over - open a new window and let this shot through.
                                        .Child<TimerExpiredNode>("burst_ready_at")
                                        .Child<TimerSetNode>("burst_fire_until", kBurstFireDurationTicks)
                                        .Child<TimerSetNode>("burst_ready_at", kBurstFireDurationTicks + kBurstFireCooldownTicks)
                                        .End()
                                    .End()
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
