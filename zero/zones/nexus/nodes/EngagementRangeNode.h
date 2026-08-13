#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

#include <cmath>

namespace zero {
namespace nexus {

// Publishes the distance we should currently be holding from the target, for OrbitNode/FleeNode to
// consume. Two things feed it, both measured off the 4v4 replay corpus.
//
// 1. Head-count, via LocalAdvantageNode (see the exchange table in that header). At parity or
//    better, closing in wins the energy trade by 2-4x; while outnumbered the same distance loses
//    it. So there isn't one correct standoff distance - there are two, and which applies depends on
//    who else is nearby.
//
//    Range also decides whether shots land at all. Bullet hit rate, same corpus:
//
//       5-9 tiles  52.7%      20-24 tiles  11.1%
//      10-14 tiles  33.3%      25-29 tiles   8.8%
//      15-19 tiles  16.3%      30-34 tiles   8.4%
//
//    There's a cliff between 10-14 and 15-19 - accuracy halves across it. Anything at or beyond 15
//    tiles is already deep into the flat, ineffective part of the curve, which is why the observed
//    habit of fighting at a median 30 tiles produces such indecisive games.
//
// 2. A slow in-and-out oscillation on top of the chosen base range. Real players do not hold a
//    fixed radius: sustained closing runs last a median 1.7s and sustained backing-off runs 1.4s,
//    each sweeping a median ~9-10 tiles of distance. Holding a constant radius is both visibly
//    robotic and trivially easy to lead a shot against, since the range at impact time is knowable
//    in advance. Oscillating reproduces the human rhythm and keeps the closing speed changing.
//
// The pump is a triangle wave rather than a sine so the approach and retreat legs hold a constant
// rate, which is what the run-length data actually describes - a steady push in, then a steady
// pull back out, not a smoothly easing drift that lingers at the extremes.
struct EngagementRangeNode : public behavior::BehaviorNode {
  EngagementRangeNode(const char* advantage_key, const char* output_key, float supported_distance,
                      float outnumbered_distance, float pump_amplitude, u32 pump_half_period_ticks)
      : advantage_key(advantage_key),
        output_key(output_key),
        supported_distance(supported_distance),
        outnumbered_distance(outnumbered_distance),
        pump_amplitude(pump_amplitude),
        pump_half_period_ticks(pump_half_period_ticks) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    auto opt_advantage = ctx.blackboard.Value<float>(advantage_key);
    if (!opt_advantage.has_value()) return behavior::ExecuteResult::Failure;

    // Outnumbered means strictly fewer of ours than theirs nearby. Even numbers still favor
    // closing, so the split is at zero rather than at a positive margin.
    float base = *opt_advantage < 0.0f ? outnumbered_distance : supported_distance;

    ctx.blackboard.Set<float>(output_key, base + GetPumpOffset());

    return behavior::ExecuteResult::Success;
  }

 private:
  // Triangle wave in [-pump_amplitude, +pump_amplitude] over a full period of two half-periods.
  float GetPumpOffset() const {
    if (pump_amplitude <= 0.0f || pump_half_period_ticks == 0) return 0.0f;

    u32 period = pump_half_period_ticks * 2;
    u32 phase = GetCurrentTick() % period;

    // Rises across the first half-period, falls across the second.
    float t = (float)phase / (float)pump_half_period_ticks;
    float ramp = t <= 1.0f ? t : 2.0f - t;

    return (ramp * 2.0f - 1.0f) * pump_amplitude;
  }

  const char* advantage_key = nullptr;
  const char* output_key = nullptr;

  float supported_distance = 0.0f;
  float outnumbered_distance = 0.0f;
  float pump_amplitude = 0.0f;
  u32 pump_half_period_ticks = 0;
};

}  // namespace nexus
}  // namespace zero
