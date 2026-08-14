#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/behavior/nodes/AimNode.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Succeeds only when a bomb fired right now has a real chance of detonating on the target, judged
// from the physics rather than from a range constant.
//
// WHAT WAS WRONG. The bomb fire check tested geometry against a bounding box inflated by
// kBombProximityMultiplier (8x the target's size) and fired whenever the trajectory clipped it.
// That is a "lob it near them" rule, and it is range-blind: the same 8x box is just as easy to
// clip at 30 tiles as at 10, even though the bomb takes three times as long to arrive and the
// target has three times as long to not be there. Hence bomb hit rate that sits flat and low
// (8-21%) instead of falling off with range the way it physically must.
//
// THE MODEL. A bomb misses when the target can leave the detonation radius during the bomb's
// flight. Both quantities are computable:
//
//   flight_time    = |aim - self| / muzzle_speed
//                    Solved in the shooter's reference frame, matching behavior::CalculateShot,
//                    which is why it takes the bare muzzle speed and not the world-frame speed.
//                    (Getting that backwards is the under-lead bug PredictiveAimNode fixed.)
//
//   evasion_radius = 0.5 * a * flight_time^2
//                    How far the target can displace itself from the constant-velocity prediction
//                    the aim solver assumed. Uses thrust along their *current heading*, forward or
//                    back, because that is the dodge available to them with no rotation at all -
//                    the cheapest and therefore the one to defend against. Acceleration is
//                    MaximumThrust * 10/16 tiles/sec^2, the conversion ShipController applies.
//
//   hit_radius     = proximity fuse radius + target's ship radius
//                    Where the bomb actually goes off, derived the same way BombBlastSafetyNode
//                    derives it so the two nodes cannot drift apart.
//
// The shot is worth taking when evasion_radius <= hit_radius * tolerance.
//
// WHY THIS IS SELF-TUNING BY RANGE. evasion_radius grows with the *square* of flight time, so
// doubling the range quadruples how far the target can slip the shot while hit_radius stays put.
// The gate therefore tightens with distance on its own and needs no maximum-range magic number -
// and the shape it produces is the accuracy-versus-range collapse the replay corpus measures.
//
// TARGET MOMENTUM. Momentum enters through the aim solution rather than here: PredictiveAimNode
// already leads the target using velocity plus smoothed acceleration, so a target committed to a
// direction is aimed at correctly. What this node adds is the other half - how much of that
// prediction the target can still invalidate before the bomb arrives.
//
// Deliberately conservative in two places, because both errors point toward *not* taking a
// marginal shot rather than toward wasting energy on one: MaximumThrust is used even though the
// target may not hold the prizes for it, and no credit is taken for the rotation time a target
// would need to dodge in any direction other than along its current heading.
struct BombImpactLikelihoodNode : public behavior::BehaviorNode {
  BombImpactLikelihoodNode(const char* target_player_key, const char* aim_key, float tolerance)
      : target_player_key(target_player_key), aim_key(aim_key), tolerance(tolerance) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target || !*opt_target) return behavior::ExecuteResult::Failure;

    Player* target = *opt_target;
    if (target->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_aim = ctx.blackboard.Value<Vector2f>(aim_key);
    if (!opt_aim) return behavior::ExecuteResult::Failure;

    Game& game = *ctx.bot->game;
    auto& settings = game.connection.settings;

    float muzzle_speed = behavior::GetWeaponSpeed(game, *self, WeaponType::Bomb);
    if (muzzle_speed <= 0.0f) return behavior::ExecuteResult::Failure;

    float distance = self->position.Distance(*opt_aim);
    float flight_time = distance / muzzle_speed;

    u32 bombs = game.ship_controller.ship.bombs;
    if (bombs == 0) return behavior::ExecuteResult::Failure;

    u32 level = bombs - 1;

    // Same derivation as BombBlastSafetyNode - keep these two in step.
    float prox_radius = ((settings.ProximityDistance + level) * 18.0f - 14.0f) / 16.0f;
    if (prox_radius < 0.0f) prox_radius = 0.0f;

    float hit_radius = prox_radius + settings.ShipSettings[target->ship].GetRadius() / 16.0f;

    // ShipController integrates velocity as thrust * (10/16) tiles/sec^2.
    float target_acceleration = settings.ShipSettings[target->ship].MaximumThrust * (10.0f / 16.0f);
    float evasion_radius = 0.5f * target_acceleration * flight_time * flight_time;

    return evasion_radius <= hit_radius * tolerance ? behavior::ExecuteResult::Success
                                                    : behavior::ExecuteResult::Failure;
  }

  const char* target_player_key = nullptr;
  const char* aim_key = nullptr;
  float tolerance = 1.0f;
};

}  // namespace nexus
}  // namespace zero
