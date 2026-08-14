#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

#include <random>

namespace zero {
namespace nexus {

// "Stick and move": the in-and-out engagement oscillation, shared by every nexus behavior.
//
// This is the primary engagement mechanic rather than a decoration on one. Two separate things make
// a bot easy to kill, and this addresses both:
//
//   * SITTING DUCK. A bot holding station is a stationary target that only has to be aimed at once.
//     The offset here never rests - there is no dwell at the extremes, every tick is on the way
//     somewhere - so a shot solved a moment ago is already wrong.
//   * PREDICTABLE. A bot that moves *regularly* is barely better than one that doesn't. The
//     previous implementation was a pure triangle wave: fixed amplitude, fixed period, and a phase
//     taken from GetCurrentTick(), which meant it was not only perfectly forecastable from two
//     observations but IDENTICAL ACROSS EVERY BOT IN THE MATCH - a whole team closing and backing
//     off on the same beat, presenting one rhythm to read instead of four.
//
// So the wave is gone and the motion is now a sequence of randomized legs. Each leg picks a turning
// point on the *opposite* side of the base range and a duration, then travels there at a constant
// rate. What an opponent can no longer know: when we will turn, how far in or out we will go, or
// how fast we will be closing when we get there.
//
// What is preserved from the 4v4 corpus is the shape and the average rhythm. Real players hold
// sustained closing runs of a median 1.7s and backing-off runs of 1.4s, each sweeping some 9-10
// tiles, and each leg is a steady push rather than a smoothly easing drift - hence constant-rate
// legs, and leg durations drawn around the tuned mean rather than replacing it. Callers pass the
// same amplitude and half-period they always did; those are now the *mean* leg reach and duration.
//
// Per-bot divergence falls out for free: each bot is its own process with its own seed, so no two
// pump on the same beat. That is the phase-lock defect fixed rather than merely documented.
struct EngagementPump {
  // Fraction of the full amplitude a leg must at least reach. Legs always cross to the other side -
  // that is the "move" - but how far past centre they go varies.
  static constexpr float kMinReachFraction = 0.5f;
  // Leg durations are drawn uniformly across this fraction band of the mean, so the turn timing is
  // never forecastable while the average cadence still matches the measured one.
  static constexpr float kMinDurationFraction = 0.5f;
  static constexpr float kMaxDurationFraction = 1.5f;

  float Update(float amplitude, u32 mean_leg_ticks) {
    if (amplitude <= 0.0f || mean_leg_ticks == 0) return 0.0f;

    u32 now = GetCurrentTick();

    if (!initialized) {
      leg_start_offset = 0.0f;
      // Arbitrary opening side, so bots don't even agree on which way they start.
      leg_target_offset = NextReach(amplitude) * (Chance() ? 1.0f : -1.0f);
      StartLeg(now, mean_leg_ticks);
      initialized = true;
    } else if (TICK_GTE(now, leg_end_tick)) {
      // Turn around: the leg we just finished ended on one side, so the next one crosses.
      leg_start_offset = leg_target_offset;
      leg_target_offset = NextReach(amplitude) * (leg_target_offset >= 0.0f ? -1.0f : 1.0f);
      StartLeg(now, mean_leg_ticks);
    }

    u32 leg_ticks = TICK_DIFF(leg_end_tick, leg_start_tick);
    if (leg_ticks == 0) return leg_target_offset;

    float t = (float)TICK_DIFF(now, leg_start_tick) / (float)leg_ticks;
    if (t < 0.0f) t = 0.0f;
    if (t > 1.0f) t = 1.0f;

    return leg_start_offset + (leg_target_offset - leg_start_offset) * t;
  }

  // True while the current leg is opening the range rather than closing it. This is what ties
  // broadside usage to the movement rhythm: the outbound leg IS the gap between waves of fire, so a
  // caller can turn side-on during it and back nose-on when the leg flips inward.
  //
  // Deriving it from the leg rather than from a firing timer is what keeps it safe. Broadside
  // prevents the shot ray from crossing the target, so anything that entered broadside *because* we
  // had not fired recently would stop us firing, which would keep us broadside - a latch that never
  // releases. Leg direction cannot do that: the legs alternate on their own, so every broadside
  // window is bounded and is always followed by a facing window.
  bool IsOutboundLeg() const { return initialized && leg_target_offset > leg_start_offset; }

 private:
  void StartLeg(u32 now, u32 mean_leg_ticks) {
    float min_ticks = mean_leg_ticks * kMinDurationFraction;
    float max_ticks = mean_leg_ticks * kMaxDurationFraction;

    std::uniform_real_distribution<float> dist(min_ticks, max_ticks);
    u32 ticks = (u32)dist(rng);
    if (ticks == 0) ticks = 1;

    leg_start_tick = now;
    leg_end_tick = now + ticks;
  }

  float NextReach(float amplitude) {
    std::uniform_real_distribution<float> dist(amplitude * kMinReachFraction, amplitude);
    return dist(rng);
  }

  bool Chance() {
    std::uniform_int_distribution<int> dist(0, 1);
    return dist(rng) == 1;
  }

  std::mt19937 rng{std::random_device{}()};

  float leg_start_offset = 0.0f;
  float leg_target_offset = 0.0f;
  u32 leg_start_tick = 0;
  u32 leg_end_tick = 0;
  bool initialized = false;
};

// Publishes a single base distance plus the pump, for trees where nothing varies the base - i.e.
// 1v1, where there is never a teammate to be supported by or a second enemy to be outnumbered by.
// Use EngagementRangeNode instead wherever a real head-count exists.
struct EngagementPumpNode : public behavior::BehaviorNode {
  EngagementPumpNode(const char* output_key, float distance, float pump_amplitude, u32 pump_mean_leg_ticks,
                     const char* outbound_key = nullptr)
      : output_key(output_key),
        distance(distance),
        pump_amplitude(pump_amplitude),
        pump_mean_leg_ticks(pump_mean_leg_ticks),
        outbound_key(outbound_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    ctx.blackboard.Set<float>(output_key, distance + pump.Update(pump_amplitude, pump_mean_leg_ticks));

    if (outbound_key) {
      if (pump.IsOutboundLeg()) {
        ctx.blackboard.Set(outbound_key, true);
      } else {
        ctx.blackboard.Erase(outbound_key);
      }
    }

    return behavior::ExecuteResult::Success;
  }

  const char* output_key = nullptr;

  float distance = 0.0f;
  float pump_amplitude = 0.0f;
  u32 pump_mean_leg_ticks = 0;
  const char* outbound_key = nullptr;

  // Per-instance, like TargetAccelerationNode's sample: one ZeroBot per process, so this is per-bot
  // state rather than anything shared across a fleet. It must NOT live on the blackboard - a fixed
  // key there is exactly the bug that silently zeroed the aim-lead estimate.
  EngagementPump pump;
};

}  // namespace nexus
}  // namespace zero
