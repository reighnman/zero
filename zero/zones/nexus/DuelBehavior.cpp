#include <zero/behavior/BehaviorBuilder.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/behavior/nodes/AimNode.h>
#include <zero/behavior/nodes/BlackboardNode.h>
#include <zero/behavior/nodes/InputActionNode.h>
#include <zero/behavior/nodes/ChatNode.h>
#include <zero/behavior/nodes/MapNode.h>
#include <zero/behavior/nodes/MathNode.h>
#include <zero/behavior/nodes/MoveNode.h>
#include <zero/behavior/nodes/PlayerNode.h>
#include <zero/behavior/nodes/RegionNode.h>
#include <zero/behavior/nodes/RenderNode.h>
#include <zero/behavior/nodes/ShipNode.h>
#include <zero/behavior/nodes/ThreatNode.h>
#include <zero/behavior/nodes/TimerNode.h>
#include <zero/behavior/nodes/WaypointNode.h>
#include <zero/zones/svs/nodes/DynamicPlayerBoundingBoxQueryNode.h>
#include <zero/zones/svs/nodes/IncomingDamageQueryNode.h>
#include <zero/zones/svs/nodes/MemoryTargetNode.h>
#include <zero/zones/nexus/nodes/FleeNode.h>
#include <zero/zones/nexus/nodes/FleeDistanceNode.h>
#include <zero/zones/nexus/nodes/PursuedFromBehindNode.h>
#include <zero/zones/nexus/nodes/SlowBombNode.h>
#include <zero/zones/nexus/nodes/OrbitNode.h>
#include <zero/zones/nexus/nodes/EngagementPump.h>
#include <zero/zones/nexus/nodes/BroadsideFaceNode.h>
#include <zero/zones/nexus/nodes/BombBlastSafetyNode.h>
#include <zero/zones/nexus/nodes/ShotLineOfSightNode.h>
#include <zero/zones/nexus/nodes/CruiseSpeedNode.h>
#include <zero/zones/nexus/nodes/WallAvoidanceNode.h>
#include <zero/zones/nexus/nodes/DodgeIncomingDamage.h>
#include <zero/zones/nexus/nodes/DodgeJukeNode.h>
#include <zero/zones/nexus/nodes/EnergyDisadvantageNode.h>
#include <zero/zones/nexus/nodes/FinishableTargetNode.h>
#include <zero/zones/nexus/nodes/TargetEnergyDropNode.h>
#include <zero/zones/nexus/nodes/RelativeShotVelocityNode.h>
#include <zero/zones/nexus/nodes/PredictiveAimNode.h>
#include <zero/zones/nexus/nodes/TargetAccelerationNode.h>

#include <zero/zones/nexus/Nexus.h>
#include "DuelBehavior.h"

using namespace zero::svs;

namespace zero {
namespace nexus {

// 1v1, no items. Derived from FoursBehavior and kept as close to it as the format allows, because
// a duel is not a different game - it is the 1v1 case of the same fight, and the combat nodes are
// shared deliberately so that tuning either one improves the other. A team bot spends a lot of its
// life in exactly this situation: separated from support, one enemy on it, nothing to coordinate
// with. Whatever makes this tree better makes that moment better too.
//
// WHAT IS REMOVED, and why each removal is safe rather than merely convenient:
//
//   * Teammate logic - attach/detach on "SAFE" team chat, TeamCentroidNode cohesion and its flee
//     bias, TeamFocusTargetNode, AvoidTeamNode. There is no team, so all of it is dead weight
//     rather than tuning.
//   * Multi-enemy logic - the LowestTargetNode "someone weaker is nearby" override, and
//     EnemiesNearTargetNode's clustering. With one opponent, nearest / lowest / team-focus are the
//     same player by definition. AvoidEnemyNode goes too: it exists to stop a *second* enemy
//     sitting on us while we fight someone else, which cannot happen here, and against the sole
//     opponent it would only fight OrbitNode for control of the same spacing.
//   * Every item - repel, decoy, portal, mine, rocket, thor, burst - since "?next duelpub" is a
//     no-items duel. Guns and bombs only. The repel/portal emergency branches go with them, and so
//     does IncomingBlastDamageNode, which existed solely to decide whether an incoming shot was
//     lethal enough to be worth spending a repel on. Antiwarp and multifire go too: antiwarp only
//     blocks portals, which no longer exist, and multifire is a spread for catching several
//     bunched enemies.
//
//   * Head-count logic - LocalAdvantageNode, EngagementRangeNode and every `local_advantage` gate
//     on rushing, pressing, finishing and pathing. See the note by kOrbitDistance for why these
//     cannot merely be reused with different thresholds. The engagement *pump* is kept, since it is
//     not head-count logic; it is shared via EngagementPump.h rather than reimplemented here.
//
// WHAT IS DELIBERATELY KEPT IDENTICAL: the whole aim / dodge / orbit / retreat / press / finish
// core, node for node. Those nodes are shared with the team behaviors on purpose, so a fix to how
// this tree leads a shot, breaks off, or commits to a kill lands in the team trees too.
std::unique_ptr<behavior::BehaviorNode> DuelBehavior::CreateTree(behavior::ExecuteContext& ctx) {
  using namespace behavior;

  BehaviorBuilder builder;

  const Vector2f center(512, 512);

  // Don't dodge below this - a target this weak is worth eating a shot to finish.
  constexpr float kLowEnergyRushThreshold = 400.0f;
  // We will rush if the opponent is low energy within this range.
  constexpr float kRushDistanceThreshold = 20.0f;
  // Only press if we have enough energy left to commit to closing the distance.
  constexpr float kRushMinEnergyPercent = 0.5f;

  // Radius used for the outgoing-damage overlap check on the bomb gate below. In FoursBehavior this
  // is expressed as kRepelDistance * 2.5f; repels don't exist here, so it's stated directly.
  constexpr float kBombOverlapRadius = 17.5f;

  // How much damage already flying at the opponent before we add a bomb to it. Bombs are cover fire
  // and area denial: the point is to arrive alongside bullets so the two can't be dodged separately.
  constexpr float kBombRequiredDamageOverlap = 300.0f;

  // Extra clearance beyond the bomb's own blast radius before we're willing to fire one. With no
  // teammates, BombBlastSafetyNode degrades to a pure self-blast check - which is the half that
  // matters most in a duel, since the opponent is usually the only thing near the detonation and we
  // are the only friendly that can be caught by our own splash.
  constexpr float kBombFriendlyBlastMargin = 6.0f;

  // Don't lob a bomb while actively reversing away from the target.
  constexpr float kBombMinForwardVelocity = 0.0f;

  // Bomb hitbox tolerance multiplier - deliberately generous. Bombs here are area denial, not
  // sniping: one that merely forces a dodge has done its job. A dodge-likelihood gate was tried on
  // top of this in the team behaviors and removed, because optimising bomb *accuracy* cost the
  // volume that actually makes the weapon useful.
  constexpr float kBombProximityMultiplier = 8.0f;

  // Don't take bullet shots past this. Measured hit rate is 7.4% at 35-39 tiles and below 7% past
  // that, against 33% at 10-14.
  constexpr float kMaxBulletRange = 35.0f;

  // Base distance to hold from the opponent, and the in-and-out oscillation applied on top of it.
  //
  // NO HEAD-COUNT ANYWHERE IN THIS TREE, which is why the range comes from EngagementPumpNode
  // rather than EngagementRangeNode. In a 1v1 a head-count cannot carry information - there is
  // never a teammate to be supported by or a second enemy to be outnumbered by - and the head-count
  // nodes cannot simply be reused with different thresholds either. LocalAdvantageNode publishes
  // (teammates within radius, self excluded) - (enemies within radius), so a duel with the opponent
  // in range reads -1 every tick. That is what an *even* fight looks like under a convention that
  // excludes self, but EngagementRangeNode reads -1 as outnumbered and every `>= 0` commit gate
  // rejects it, so the bot would sit at the long standoff and never press.
  //
  // The pump is NOT team logic and is shared rather than reimplemented - see EngagementPump.h. It
  // matters more here than anywhere: an opponent watching a single bot hold a fixed radius has the
  // easiest possible lead-shot problem, since the range at impact time is knowable in advance.
  constexpr float kOrbitDistance = 14.4f;
  constexpr float kPumpAmplitude = 4.0f;
  constexpr u32 kPumpHalfPeriodTicks = 150;  // ~1.5s per leg

  // Standoff to break off to, at full energy. FleeNode holds this as a kiting leash rather than
  // running, which is correct while healthy and wrong while hurt - so it is the *healthy* end of a
  // ramp. The measured kill approach starts at a median 30.8 tiles and closes in three seconds, so
  // an injured bot holding a fixed 30 recharges inside the kill funnel. See FleeDistanceNode.
  constexpr float kLeashDistance = 30.0f;
  constexpr float kLeashDistanceHurt = 55.0f;
  constexpr float kLeashHurtEnergyPercent = 0.35f;

  // Below this FleeNode abandons the leash entirely and just opens distance. Raised from 0.20:
  // bots died at 7.5-9.2% energy, and at 20% the killer is already inside 13 tiles.
  constexpr float kFleePanicEnergyPercent = 0.3f;

  // Cruise at less than full speed unless committing to a kill or running. A ship already at
  // maximum has no acceleration left to dodge with and carries momentum it cannot cheaply reverse.
  constexpr float kCruiseSpeedPercent = 0.8f;

  // How long to keep pressing after the opponent loses energy while we still have more than they
  // do - a sustained window rather than a single-tick reaction.
  constexpr u32 kPressAdvantageTicks = 300;  // ~3s

  // Terrain handling. Detection scales with actual speed rather than a fixed radius, so the push
  // out of a pocket starts while there is still somewhere to go instead of once already wedged.
  constexpr float kWallLookaheadSeconds = 0.9f;
  // Fleeing gets a much longer horizon. 0.9s is about 17 tiles at retreat speed - enough to avoid
  // running into a wall, but not enough to avoid COMMITTING TO A DIRECTION THAT DEAD ENDS, which is
  // a different failure. A retreat runs to 30-55 tiles (FleeDistanceNode), so a corridor that closes
  // at 20 tiles reads as clear when the direction is chosen and the problem only appears once we are
  // inside the pocket with the opponent behind us. 2.4s reaches ~45 tiles at retreat speed.
  constexpr float kFleeWallLookaheadSeconds = 2.4f;
  constexpr float kWallCheckDistance = 5.0f;
  constexpr float kWallOpeningDistance = 35.0f;

  // Bullets bounce in this zone, so a blocked lane isn't a dead bullet - but past a ricochet the
  // aim solution is void, so it's only worth taking in tight geometry.
  constexpr float kBulletBounceRange = 12.0f;

  // How far ahead to bend predicted aim toward where the target is actually trending.
  constexpr float kAimLeadBiasSeconds = 0.2f;

  // Energy-relative retreat, identical to the team behaviors. Enters a defensive state once our
  // energy drops below this fraction of the opponent's, and doesn't leave until we recover past the
  // higher exit ratio - the gap is a hysteresis band so the decision doesn't flicker near parity.
  constexpr float kEnergyDisadvantageEnterRatio = 0.65f;
  constexpr float kEnergyDisadvantageExitRatio = 0.9f;
  constexpr float kCriticalEnergyPercent = 0.18f;

  // Absolute floor on ENDING a retreat. The ratio test only says whether we're still losing the
  // comparison, and while we're away recharging so is the opponent - so against an equally hurt
  // one it can clear with both sides near dead, sending us back into a fight we're still too weak
  // for. rec28 showed exactly that in the team trees: retreating% rose but bots still died at
  // 5.9-13.2% energy because the retreat ended before they reached the distance they were heading
  // for. This matters more in a duel, where the only opponent is by definition the one we're
  // recharging against.
  constexpr float kRetreatRecoveryEnergyPercent = 0.5f;

  // --- Reverse-retreat bomb ---
  // A bomb fired while backing away nose-on inherits our velocity and subtracts it from the muzzle
  // speed, so it barely travels and sits in the chaser's path like a mine. See SlowBombNode. This
  // is the only wake weapon a duel has - the arena is no-items, so there are no actual mines - and
  // it matters more here than in the team trees, since the one opponent chasing us is by definition
  // the only threat there is.
  constexpr float kReverseBombMaxGroundSpeed = 4.0f;
  constexpr float kReverseBombRearConeDegrees = 120.0f;
  constexpr float kReverseBombClosingSpeed = 9.0f;
  constexpr float kReverseBombMinDistance = 7.0f;
  constexpr float kReverseBombMaxDistance = 20.0f;
  // Bombs are expensive (BombFireEnergy is a large fraction of max) and this fires exactly when
  // we're hurt and running, so keep it out of the critical band we're retreating to escape.
  constexpr float kReverseBombMinEnergyPercent = 0.35f;
  constexpr u32 kReverseBombCooldownTicks = 150;

  // Hard floor on engaging at all, and the single exemption to it: a fight we can end this second.
  constexpr float kEngageFloorEnergyPercent = 0.15f;
  constexpr float kFinishRelativeEnergyPercent = 0.5f;
  constexpr float kFinishAbsoluteEnergyPercent = 0.15f;
  constexpr float kFinishDistance = 10.0f;


  // clang-format off
  builder
    .Selector()
        .Sequence() //Join the duel queue. "duelpub" is the no-items 1v1 arena - guns and bombs only.
            .Child<TimerExpiredNode>("queue")
            .Child<ChatMessageNode>(ChatMessageNode::Public("?next duelpub"))
            .Child<ChatMessageNode>(ChatMessageNode::Public("?return"))
            .Child<TimerSetNode>("queue", 6000)
            .End()
        .Sequence() // Don't do anything while in spec
            .Child<PlayerFrequencyQueryNode>("self_freq")
            .Child<EqualityNode<u16>>("self_freq", 8025)  //Check spec
            .Child<ScalarNode>(1.0f, "spectating")
            .End()
        .Sequence() // Match startup begins when we get taken out of spec (since we sit in spec when waiting)
            .Child<BlackboardSetQueryNode>("spectating")  //We just came out of spectating
            .Child<TimerSetNode>("match_startup", 3000)  //Safety net only - Nexus.cpp expires this immediately once it sees the "GO!" match start message
            .Child<BlackboardEraseNode>("spectating")
            .End()
        .Sequence() // Enter the specified ship if not already in it and have been taken out of spec.
            .InvertChild<TimerExpiredNode>("match_startup")
            .InvertChild<ShipQueryNode>("request_ship")
            .Child<ShipRequestNode>("request_ship")
            .End()
        .Sequence()  // Fire 1 startup shot so we get out of the ready check loop
            .InvertChild<TimerExpiredNode>("match_startup")
            .Child<TimerExpiredNode>("pre_fire")  // just needs to be longer than match_start
            .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bullet)
            .Child<InputActionNode>(InputAction::Bullet)
            .Child<TimerSetNode>("pre_fire", 1500)
            .End()
           .Selector() // Choose to fight the player or follow waypoints.
            .Sequence() // Find the opponent and either path to them or seek them directly.
                .Sequence(CompositeDecorator::Success)
                    .Child<PlayerPositionQueryNode>("self_position")
                    .Sequence() //There is exactly one enemy, so "target" and "nearest_target" are the same player. Both are populated anyway so the defensive and flee branches below stay byte-identical to FoursBehavior, where they genuinely differ.
                        .Child<NearestMemoryTargetNode>("target")
                        .Child<NearestMemoryTargetNode>("nearest_target")
                        .Child<PlayerEnergyQueryNode>("nearest_target", "nearest_target_energy")
                        .Child<PlayerPositionQueryNode>("nearest_target", "nearest_target_position")
                        .Child<TargetAccelerationNode>("nearest_target", "nearest_target_acceleration")
                        .Child<PredictiveAimNode>(WeaponType::Bullet, "nearest_target", "nearest_target_acceleration", "nearest_aimshot", "nearest_aimshot_world", kAimLeadBiasSeconds)
                        .End()
                     .Sequence(CompositeDecorator::Success) //Derive everything else from the target. No override chain here - with one enemy there is nothing to override with.
                        .Child<PlayerPositionQueryNode>("target", "target_position")
                        .Child<PlayerEnergyQueryNode>("target", "target_energy")
                        .Child<TargetAccelerationNode>("target", "target_acceleration")
                        .Child<PredictiveAimNode>(WeaponType::Bullet, "target", "target_acceleration", "aimshot", "aimshot_world", kAimLeadBiasSeconds)
                        .Child<PredictiveAimNode>(WeaponType::Bomb, "target", "target_acceleration", "bomb_aimshot", "bomb_aimshot_world", kAimLeadBiasSeconds) //Bombs fly slower than bullets, so they need their own (larger) lead
                        .Child<TargetEnergyDropNode>("target", "target_energy", "target_energy_dropped") //Did the opponent just lose energy - identity-checked and staleness-guarded
                        .End()
                .End()
                .Sequence(CompositeDecorator::Success) // Work out how close we should be fighting right now.
                    .Child<EngagementPumpNode>("engagement_range", kOrbitDistance, kPumpAmplitude, kPumpHalfPeriodTicks, "pump_outbound") //One base distance since nothing here varies it, plus the shared in-and-out pump. Published under the same key the movement below reads, so that block stays identical to FoursBehavior.
                    .Child<FleeDistanceNode>("flee_distance", kLeashDistance, kLeashDistanceHurt, kLeashHurtEnergyPercent) //How far to break off scales with how hurt we are - a fixed 30 tiles held injured bots in the exact band killers start their run from
                    .Selector(CompositeDecorator::Success) // Hold something back unless we're committing to a kill or running for our life.
                        .Child<BlackboardSetQueryNode>("rushing")            //Pressing - commit everything
                        .InvertChild<TimerExpiredNode>("recharge_timer")     //Escaping - we want every bit of speed
                        .Child<CruiseSpeedNode>(kCruiseSpeedPercent)
                        .End()
                    .End()
                .Sequence(CompositeDecorator::Success) // Continuously reassess fight-vs-flee using energy relative to the opponent, instead of a fixed timer.
                    .Child<EnergyDisadvantageNode>("nearest_target", "nearest_target_energy", "energy_disadvantaged", kEnergyDisadvantageEnterRatio, kEnergyDisadvantageExitRatio, kCriticalEnergyPercent, kRetreatRecoveryEnergyPercent)
                    .Child<TimerSetNode>("recharge_timer", 200)
                    .End()
                .Sequence(CompositeDecorator::Success) // Hard floor: below this we don't engage at all. Stated separately from the critical percent above so retuning that can't quietly repeal this.
                    .InvertChild<PlayerEnergyPercentThresholdNode>(kEngageFloorEnergyPercent)
                    .Child<TimerSetNode>("recharge_timer", 200)
                    .End()
                .Selector(CompositeDecorator::Success) // Below the engage floor the only fight worth staying in is one we can end. This is the single exception that takes recharge_timer back off.
                    .Sequence()
                        .InvertChild<PlayerEnergyPercentThresholdNode>(kEngageFloorEnergyPercent) // we're under the floor
                        .Child<FinishableTargetNode>("target", "target_energy", kFinishRelativeEnergyPercent, kFinishAbsoluteEnergyPercent) // ...but they're clearly lower and nearly dead
                        .InvertChild<DistanceThresholdNode>("target_position", "self_position", kFinishDistance) // close enough to actually land it
                        .Child<VisibilityQueryNode>("target_position") // no chasing across the map at this energy
                        .Child<ScalarNode>(1.0f, "finishing")
                        .Child<BlackboardEraseNode>("recharge_timer")
                        .End()
                    .Child<BlackboardEraseNode>("finishing")
                    .End()
                .Selector()
                    .Sequence() // Dodge incoming fire, unless the opponent is nearly dead and on top of us - then eat it and finish them.
                        .Sequence(CompositeDecorator::Invert)
                            .InvertChild<ScalarThresholdNode<float>>("target_energy", kLowEnergyRushThreshold)
                            .InvertChild<DistanceThresholdNode>("target_position", "self_position", kRushDistanceThreshold)
                            .End()
                        .Child<DodgeIncomingDamage>(0.2f, 30.0f)
                        .End()
                    .Sequence() // Keep distance during ready-check instead of sitting still until the match officially starts.
                        .InvertChild<TimerExpiredNode>("match_startup")
                        .Selector() // Steer clear of nearby walls before fleeing so we don't get pinned in a corner.
                            .Child<WallAvoidanceNode>(kWallCheckDistance, kWallOpeningDistance, kFleeWallLookaheadSeconds) //Long flee horizon - a retreat commits to 30-55 tiles, so the cast has to reach that far or we pick a corridor that dead ends
                            .Child<FleeNode>("nearest_target_position", kLeashDistance, 5.0f, 0.2f, "nearest_target_energy")
                            .End()
                        .End()
                    .Sequence()  //Keep distance while recharging
                        .InvertChild<TimerExpiredNode>("recharge_timer")
                        .Child<BlackboardEraseNode>("rushing") //We're breaking off, so we are no longer pressing. Without this "rushing" is only cleared inside the aim-and-shoot Parallel below, which this branch skips entirely.
                        .Sequence(CompositeDecorator::Success) // Bomb the chaser while backing away. Nose-on in reverse the bomb sheds its muzzle speed against our own and hangs in their path like a mine - see SlowBombNode. Duel has no items, so this is the only wake weapon available here.
                            .Child<PlayerEnergyPercentThresholdNode>(kReverseBombMinEnergyPercent)
                            .Child<PursuedFromBehindNode>("nearest_target", kReverseBombRearConeDegrees, kReverseBombClosingSpeed)
                            .Child<SlowBombNode>(kReverseBombMaxGroundSpeed) //Gates on the RESULT - if the shot would leave at speed (nose not really back at them, or FleeNode holding broadside) this fails and we don't throw a bomb away
                            .Child<DistanceThresholdNode>("nearest_target_position", "self_position", kReverseBombMinDistance)
                            .InvertChild<DistanceThresholdNode>("nearest_target_position", "self_position", kReverseBombMaxDistance)
                            .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bomb)
                            .Child<TimerExpiredNode>("reverse_bomb_timer")
                            .InvertChild<TileQueryNode>(kTileIdSafe)
                            //No wake-blast guard here, unlike the team trees: a duel has no teammates to catch the blast, and self is covered by the fact we are receding from the bomb at retreat speed the whole time the opponent is closing on it.
                            .Child<InputActionNode>(InputAction::Bomb)
                            .Child<TimerSetNode>("reverse_bomb_timer", kReverseBombCooldownTicks)
                            .End()
                        .Selector() // Steer clear of nearby walls before fleeing so we don't get pinned in a corner.
                            .Child<WallAvoidanceNode>(kWallCheckDistance, kWallOpeningDistance, kFleeWallLookaheadSeconds) //No team centroid to bias toward in a duel, so the escape direction is chosen purely on openness. Long flee horizon for the same reason as the team trees.
                            .Child<FleeNode>("nearest_aimshot", "flee_distance", 5.0f, kFleePanicEnergyPercent, "nearest_target_energy") //Distance scales with injury instead of being a fixed leash, and the panic threshold is raised - see FleeDistanceNode
                            .End()
                        .Sequence(CompositeDecorator::Success) // Keep shooting at whoever is chasing us. Backing off must not mean going silent - this branch takes the whole Selector, so the aim-and-shoot block below never runs while it is active. FleeNode already faces the threat while retreating, so the heading is right and this only needs permission to pull the trigger.
                            .Child<TimerExpiredNode>("match_startup")
                            .InvertChild<DistanceThresholdNode>("nearest_target_position", kMaxBulletRange)
                            .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bullet)
                            .InvertChild<InputQueryNode>(InputAction::Bomb)
                            .InvertChild<TileQueryNode>(kTileIdSafe)
                            .Child<RelativeShotVelocityNode>(WeaponType::Bullet, "bullet_fire_velocity")
                            .Child<RayNode>("self_position", "bullet_fire_velocity", "bullet_fire_ray")
                            .Child<DynamicPlayerBoundingBoxQueryNode>("nearest_target", "nearest_target_bounds", 4.0f)
                            .Child<MoveRectangleNode>("nearest_target_bounds", "nearest_aimshot", "nearest_target_bounds")
                            .Child<RayRectangleInterceptNode>("bullet_fire_ray", "nearest_target_bounds")
                            .Child<ShotLineOfSightNode>("nearest_aimshot_world", kBulletBounceRange)  //Same terrain gate as the main fire check - retreating is when we're most likely to have terrain between us and the chaser
                            .Child<InputActionNode>(InputAction::Bullet)
                            .End()
                        .End()
                    .Sequence() // Path to the opponent if they aren't immediately visible.
                        .InvertChild<VisibilityQueryNode>("target_position")
                        .Child<GoToNode>("target_position")
                        .Child<RenderPathNode>(Vector3f(0.0f, 1.0f, 0.5f))
                        .End()
                    .Sequence() // Aim and shoot while seeking them.
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
                               .Sequence() // Committed to ending a fight we're otherwise too weak to be in. Gated hard by the finish exemption above, so reaching here already means they're nearly dead, close and visible.
                                    .Child<BlackboardSetQueryNode>("finishing")
                                    .Child<SeekNode>("aimshot", 0.0f, SeekNode::DistanceResolveType::Static)
                                    .End()
                               .Sequence() // The opponent is low and close - go finish it.
                                    .Child<PlayerEnergyPercentThresholdNode>(kRushMinEnergyPercent) //only press if we have enough energy ourselves
                                    .InvertChild<DistanceThresholdNode>("target_position", "self_position", kRushDistanceThreshold)
                                    .InvertChild<ScalarThresholdNode<float>>("target_energy", kLowEnergyRushThreshold)
                                    .Child<SeekNode>("aimshot", 0.0f, SeekNode::DistanceResolveType::Static)
                                    .Child<ScalarNode>(1.0f, "rushing") // set rushing status
                                    .Child<BlackboardEraseNode>("recharge_timer") // remove recharge status as we're going in for the kill
                                    .Child<BlackboardEraseNode>("orbit_direction") // pick a fresh orbit direction next time we're back to circling
                                    .End()
                                .Sequence() // Press the advantage for a while after the opponent loses energy and now has meaningfully less than we do.
                                    .InvertChild<DistanceThresholdNode>("target_position", "self_position", kOrbitDistance * 2.0f) //still needs to be a fight we're actually in
                                    .Child<PlayerCurrentEnergyQueryNode>("self_energy")
                                    .Sequence(CompositeDecorator::Success) // (Re)arm the window on a fresh drop - it doesn't need to still be dropping for the window to hold.
                                        .Child<BlackboardSetQueryNode>("target_energy_dropped")
                                        .Child<GreaterThanNode<float>>("self_energy", "target_energy")
                                        .Child<TimerSetNode>("press_advantage_until", kPressAdvantageTicks)
                                        .End()
                                    .InvertChild<TimerExpiredNode>("press_advantage_until") // still inside the window from a recent drop
                                    .Child<SeekNode>("aimshot", 0.0f, SeekNode::DistanceResolveType::Static)
                                    .Child<ScalarNode>(1.0f, "rushing")
                                    .Child<BlackboardEraseNode>("recharge_timer")
                                    .Child<BlackboardEraseNode>("orbit_direction")
                                    .End()
                                .Sequence(CompositeDecorator::Success)
                                    .InvertChild<BlackboardSetQueryNode>("rushing")
                                    .Sequence(CompositeDecorator::Success) // Bake terrain into the attack movement too - additive, so it steers us around walls while still closing/orbiting rather than replacing the attack.
                                        .Child<WallAvoidanceNode>(kWallCheckDistance, kWallOpeningDistance, kWallLookaheadSeconds, "target_position")
                                        .End()
                                    .Selector() // Close the gap while still far out, then circle instead of closing all the way to melee range.
                                        .Sequence()
                                            .Child<DistanceThresholdNode>("target_position", "self_position", "engagement_range")
                                            .Child<SeekNode>("aimshot", 0.0f, SeekNode::DistanceResolveType::Zero)
                                            .End()
                                        .Child<OrbitNode>("aimshot", "engagement_range", "orbit_direction")
                                        .End()
                                    .End()
                                .End()
                            .Sequence(CompositeDecorator::Success) // Bomb fire check.
                                .Child<TimerExpiredNode>("match_startup") // Ensure match countdown timer has expired
                                .Child<TimerExpiredNode>("recharge_timer")  // Ensure we're not still in a fleeing state
                                .Child<VectorSubtractNode>("bomb_aimshot", "self_position", "target_direction", true) //check target aim
                                .Child<PlayerVelocityQueryNode>("self_velocity") // get our current velocity
                                .Child<VectorDotNode>("self_velocity", "target_direction", "forward_velocity")  // compare our velocity to target
                                .Child<ScalarThresholdNode<float>>("forward_velocity", kBombMinForwardVelocity) // don't lob one while actively backing away
                                .Child<PlayerEnergyPercentThresholdNode>(0.45f) // ensure we have enough energy to fire
                                .Child<ShipWeaponCapabilityQueryNode>(WeaponType::Bomb) // ensure bombs are ready to fire
                                .InvertChild<ShipWeaponCooldownQueryNode>(WeaponType::Bomb) // ensure bombs are off cooldown
                                .Child<IncomingDamageQueryNode>("target", kBombOverlapRadius, 2.75f, "outgoing_damage") // check outgoing damage to target
                                .Child<ScalarThresholdNode<float>>("outgoing_damage", kBombRequiredDamageOverlap) // Check if we have enough bullets overlapping to fire a bomb into.
                                .InvertChild<DistanceThresholdNode>("nearest_target_position", 50.0f)  //dont bomb from too far
                                .Child<BombBlastSafetyNode>("bomb_aimshot_world", kBombFriendlyBlastMargin)  //with no teammates this is a pure self-blast check - never bomb somewhere our own explosion catches us
                                .Child<ShotLineOfSightNode>("bomb_aimshot_world", 0.0f, true)  //Bombs don't pass through walls, so the lane has to be clear - but only up to proximity range of the target, since the fuse trips on the ship first and terrain inside that last stretch can't stop the shot.
                                .Child<RelativeShotVelocityNode>(WeaponType::Bomb, "bomb_fire_velocity") // check bomb velocity
                                .Child<RayNode>("self_position", "bomb_fire_velocity", "bomb_fire_ray") // check collision ray
                                .Child<DynamicPlayerBoundingBoxQueryNode>("target", "target_bounds", kBombProximityMultiplier) // lob range, not a precise hit
                                .Child<MoveRectangleNode>("target_bounds", "bomb_aimshot", "target_bounds")
                                .Child<RenderRectNode>("world_camera", "target_bounds", Vector3f(1.0f, 0.0f, 0.0f))
                                .Child<RenderRayNode>("world_camera", "bomb_fire_ray", 50.0f, Vector3f(1.0f, 1.0f, 0.0f))
                                .Child<RayRectangleInterceptNode>("bomb_fire_ray", "target_bounds")
                                .Child<InputActionNode>(InputAction::Bomb) // fire bomb
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
                                .Child<RelativeShotVelocityNode>(WeaponType::Bullet, "bullet_fire_velocity")
                                .Child<RayNode>("self_position", "bullet_fire_velocity", "bullet_fire_ray")
                                .Child<DynamicPlayerBoundingBoxQueryNode>("target", "target_bounds", 4.0f)
                                .Child<MoveRectangleNode>("target_bounds", "aimshot", "target_bounds")
                                .Child<RayRectangleInterceptNode>("bullet_fire_ray", "target_bounds")
                                .Child<ShotLineOfSightNode>("aimshot_world", kBulletBounceRange)  //The intercept test above knows nothing about terrain, so a target behind a wall still produces a valid-looking shot. Bounce allowance kept for tight corners.
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
