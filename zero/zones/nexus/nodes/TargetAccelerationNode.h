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
// doesn't get read as a huge, bogus acceleration spike, and whenever the previous sample is too old
// to differentiate against - a target reacquired after a retreat has moved arbitrarily far in the
// meantime, and dividing that displacement by the elapsed time is meaningless.
//
// The previous sample lives on the node rather than on the blackboard. It used to sit under fixed
// blackboard keys ("target_accel_prev_id" and friends), which quietly broke as soon as a behavior
// tree ran more than one of these per tick: the nexus trees evaluate a target-override chain, so
// this ran once for the nearest enemy and again for the team focus target, each call overwriting the
// other's sample. Every call then saw a previous id belonging to the *other* target, treated it as a
// target change, and zeroed the estimate - so whenever the team focus target differed from the
// nearest enemy, which is most of the time, the acceleration fed to PredictiveAimNode was
// permanently zero and the aim lead bias did nothing at all. Per-instance state makes each call site
// its own independent series, which is what the code always assumed it had. Safe because one
// ZeroBot runs per process, so a node instance is per-bot state, not shared across a fleet.
struct TargetAccelerationNode : public behavior::BehaviorNode {
  TargetAccelerationNode(const char* target_player_key, const char* output_key, float smoothing = 0.35f)
      : target_player_key(target_player_key), output_key(output_key), smoothing(smoothing) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target || !*opt_target) return behavior::ExecuteResult::Failure;

    Player* target = *opt_target;
    u32 tick = GetCurrentTick();

    s32 age = has_previous ? TICK_DIFF(tick, previous_tick) : 0;
    bool usable_sample = has_previous && previous_id == target->id && age > 0 && (u32)age <= kMaxSampleAgeTicks;

    if (!usable_sample) {
      acceleration = Vector2f(0, 0);
    } else {
      float dt = age / 100.0f;
      Vector2f raw_acceleration = (target->velocity - previous_velocity) / dt;

      // Exponential moving average so a single noisy tick doesn't cause a wild swing in aim.
      acceleration = acceleration + (raw_acceleration - acceleration) * smoothing;
    }

    ctx.blackboard.Set(output_key, acceleration);

    previous_id = target->id;
    previous_velocity = target->velocity;
    previous_tick = tick;
    has_previous = true;

    return behavior::ExecuteResult::Success;
  }

  // Half a second. Past that the target has had time to change direction completely, so the velocity
  // delta says nothing useful about what they're doing right now.
  static constexpr u32 kMaxSampleAgeTicks = 50;

  const char* target_player_key = nullptr;
  const char* output_key = nullptr;
  float smoothing = 0.35f;

  PlayerId previous_id = 0;
  Vector2f previous_velocity;
  u32 previous_tick = 0;
  bool has_previous = false;
  Vector2f acceleration;
};

}  // namespace nexus
}  // namespace zero
