#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

#include <cstdlib>

namespace zero {
namespace teamversus {

// Paces shooting like a person rather than like a weapon cooldown.
//
// A bot that fires on every tick the cooldown allows is instantly identifiable and, more
// practically, wastes energy: at 20 energy a bullet and a 1150 recharge, continuous fire is a real
// drain, and the energy it costs is energy not available to survive the next exchange.
//
// Measured bullet-fire gaps across 46,000 shots in real 4v4 matches:
//
//     <= 0.2s   (same-burst rapid fire)       2.5%
//     0.21-1.0s (short pause / burst rhythm) 68.8%
//     1.01-3.0s (between-volley regroup)     10.3%
//     > 3.0s    (lull between engagements)   18.4%
//
// with a median gap of 0.36s against a weapon floor of 0.24s. So the dominant mode is a shot every
// third of a second or so, not a continuous stream - and the sub-0.2s bucket being only 2.5% is a
// statement about the weapon's own delay, not about restraint.
//
// The long tail is deliberately *not* modelled here. That 18.4% of gaps over three seconds is
// mostly time with no target, no line of sight, or not enough energy - conditions the tree already
// gates on. Reproducing it as a random silence as well would double-count it and leave the bot
// standing mute in front of an enemy for seconds at a time, which is the one failure mode that
// reliably turns into a death spiral.
//
// The gap is redrawn after every permitted shot, and shortens while pressing: committing to a kill
// is exactly when a person stops pacing themselves.
struct FireCadenceNode : public behavior::BehaviorNode {
  FireCadenceNode() {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Tick now = GetCurrentTick();

    if (last_fire_tick != 0 && TICK_DIFF(now, last_fire_tick) < (s32)current_gap_ticks) {
      return behavior::ExecuteResult::Failure;
    }

    last_fire_tick = now;
    current_gap_ticks = RollGap(ctx.blackboard.Has("phase_press"));

    return behavior::ExecuteResult::Success;
  }

  // Bucket boundaries in ticks (centiseconds), matching the measured distribution.
  u32 rapid_min = 20;
  u32 rapid_max = 24;
  u32 normal_min = 25;
  u32 normal_max = 100;
  u32 pause_min = 101;
  u32 pause_max = 300;

  // Bucket weights, renormalised from the measured distribution with the >3s lull bucket removed
  // for the reason described above.
  float rapid_chance = 0.03f;
  float normal_chance = 0.84f;
  // Remainder falls into the pause bucket.

  // While pressing, collapse toward the fast end - the pause bucket is dropped entirely.
  float press_rapid_chance = 0.35f;

 private:
  Tick last_fire_tick = 0;
  u32 current_gap_ticks = 0;

  u32 RollGap(bool pressing) const {
    float roll = (float)rand() / (float)RAND_MAX;

    if (pressing) {
      if (roll < press_rapid_chance) return RandomRange(rapid_min, rapid_max);
      return RandomRange(normal_min, normal_max);
    }

    if (roll < rapid_chance) return RandomRange(rapid_min, rapid_max);
    if (roll < rapid_chance + normal_chance) return RandomRange(normal_min, normal_max);

    return RandomRange(pause_min, pause_max);
  }

  static u32 RandomRange(u32 min, u32 max) {
    if (max <= min) return min;
    return min + (u32)(rand() % (int)(max - min + 1));
  }
};

}  // namespace teamversus
}  // namespace zero
