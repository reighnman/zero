#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/zones/nexus/nodes/EngagementPump.h>

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
// The pump itself now lives in EngagementPump.h and is shared with the duel tree, which has no
// head-count to feed this node but needs the oscillation just as much. This node keeps ownership of
// the part that is genuinely about head-count - which base distance applies - and delegates the
// rest. Behavior here is unchanged by that split.
struct EngagementRangeNode : public behavior::BehaviorNode {
  EngagementRangeNode(const char* advantage_key, const char* output_key, float supported_distance,
                      float outnumbered_distance, float pump_amplitude, u32 pump_half_period_ticks,
                      const char* outbound_key = nullptr)
      : advantage_key(advantage_key),
        output_key(output_key),
        supported_distance(supported_distance),
        outnumbered_distance(outnumbered_distance),
        pump_amplitude(pump_amplitude),
        pump_half_period_ticks(pump_half_period_ticks),
        outbound_key(outbound_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    auto opt_advantage = ctx.blackboard.Value<float>(advantage_key);
    if (!opt_advantage.has_value()) return behavior::ExecuteResult::Failure;

    // Outnumbered means strictly fewer of ours than theirs nearby. Even numbers still favor
    // closing, so the split is at zero rather than at a positive margin.
    float base = *opt_advantage < 0.0f ? outnumbered_distance : supported_distance;

    ctx.blackboard.Set<float>(output_key, base + pump.Update(pump_amplitude, pump_half_period_ticks));

    if (outbound_key) {
      if (pump.IsOutboundLeg()) {
        ctx.blackboard.Set(outbound_key, true);
      } else {
        ctx.blackboard.Erase(outbound_key);
      }
    }

    return behavior::ExecuteResult::Success;
  }

 private:
  const char* advantage_key = nullptr;
  const char* output_key = nullptr;

  float supported_distance = 0.0f;
  float outnumbered_distance = 0.0f;
  float pump_amplitude = 0.0f;
  u32 pump_half_period_ticks = 0;
  const char* outbound_key = nullptr;

  // Per-instance, not on the blackboard - a fixed blackboard key is exactly the bug that silently
  // zeroed the aim-lead estimate. One ZeroBot per process, so this is per-bot state.
  EngagementPump pump;
};

}  // namespace nexus
}  // namespace zero
