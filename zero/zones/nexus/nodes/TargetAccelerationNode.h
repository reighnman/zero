#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>

namespace zero {
namespace nexus {

// Estimates the current target's acceleration from tick-to-tick velocity changes, smoothed with an
// exponential moving average to filter out packet jitter and brief thrust taps. Used to bend aim
// prediction toward where a maneuvering target is actually trending instead of assuming they'll
// hold their current velocity for the whole shot, and to scale shot spread by how erratic they've
// actually been.
//
// Resets its estimate whenever the target player changes (e.g. a new target is selected) so a swap
// doesn't get read as a huge, bogus acceleration spike.
struct TargetAccelerationNode : public behavior::BehaviorNode {
  TargetAccelerationNode(const char* target_player_key, const char* output_key, float smoothing = 0.35f)
      : target_player_key(target_player_key), output_key(output_key), smoothing(smoothing) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target || !*opt_target) return behavior::ExecuteResult::Failure;

    Player* target = *opt_target;
    u32 tick = GetCurrentTick();

    // Carry the previous estimate forward by default - only replaced below when we can actually
    // compute a fresh one this tick.
    Vector2f acceleration = ctx.blackboard.ValueOr<Vector2f>(output_key, Vector2f(0, 0));

    auto opt_prev_id = ctx.blackboard.Value<PlayerId>("target_accel_prev_id");
    auto opt_prev_velocity = ctx.blackboard.Value<Vector2f>("target_accel_prev_velocity");
    auto opt_prev_tick = ctx.blackboard.Value<u32>("target_accel_prev_tick");

    bool same_target = opt_prev_id && *opt_prev_id == target->id;

    if (!same_target) {
      acceleration = Vector2f(0, 0);
    } else if (opt_prev_velocity && opt_prev_tick) {
      float dt = TICK_DIFF(tick, *opt_prev_tick) / 100.0f;

      if (dt > 0.0f) {
        Vector2f raw_acceleration = (target->velocity - *opt_prev_velocity) / dt;

        // Exponential moving average so a single noisy tick doesn't cause a wild swing in aim.
        acceleration = acceleration + (raw_acceleration - acceleration) * smoothing;
      }
    }

    ctx.blackboard.Set(output_key, acceleration);
    ctx.blackboard.Set<PlayerId>("target_accel_prev_id", target->id);
    ctx.blackboard.Set("target_accel_prev_velocity", target->velocity);
    ctx.blackboard.Set<u32>("target_accel_prev_tick", tick);

    return behavior::ExecuteResult::Success;
  }

  const char* target_player_key = nullptr;
  const char* output_key = nullptr;
  float smoothing = 0.35f;
};

}  // namespace nexus
}  // namespace zero
