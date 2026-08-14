#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Detects that the *current* target lost energy since the last sample - we or a teammate landed
// something on them, or they spent energy firing - and publishes it as an existence flag for the
// press-advantage window to arm off.
//
// This replaces the "target_energy_prev" blackboard snapshot that window used to key off, which
// recorded a number and nothing else. Because it carried no identity, it could not tell a target
// *losing* energy from the target simply *changing*: switching from a healthy enemy to a weaker one
// read as a large energy drop and armed a three-second commit-forward window against a target that
// had taken no damage at all. That misread is frequent rather than rare, because the selected target
// changes constantly - TeamFocusTargetNode picks the enemy nearest the team centroid, the centroid
// moves every tick, so the pick flips whenever two enemies are near-equidistant, and on top of that
// the self-defense exception toggles the whole override on and off as the nearest enemy crosses
// kSelfDefenseDistance.
//
// The snapshot was also written only at the end of the aim-and-shoot Parallel, a branch the flee
// path skips entirely, so the first comparison after any retreat was against a value from before it
// - reliably lower now than then, and so reliably a false arm at the exact moment we re-engage.
//
// Keeping the sample on the node, with an identity check and a staleness guard, makes both failure
// modes impossible regardless of where in the tree this runs. Per-instance state is safe here for
// the same reason it is in TargetAccelerationNode: one ZeroBot per process.
struct TargetEnergyDropNode : public behavior::BehaviorNode {
  TargetEnergyDropNode(const char* target_player_key, const char* target_energy_key, const char* output_key,
                       float minimum_drop = 1.0f, u32 max_sample_age_ticks = 25)
      : target_player_key(target_player_key),
        target_energy_key(target_energy_key),
        output_key(output_key),
        minimum_drop(minimum_drop),
        max_sample_age_ticks(max_sample_age_ticks) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target || !*opt_target) return behavior::ExecuteResult::Failure;

    Player* target = *opt_target;

    auto opt_energy = ctx.blackboard.Value<float>(target_energy_key);
    if (!opt_energy) return behavior::ExecuteResult::Failure;

    float energy = *opt_energy;
    u32 tick = GetCurrentTick();

    s32 age = has_previous ? TICK_DIFF(tick, previous_tick) : 0;
    bool comparable = has_previous && previous_id == target->id && age > 0 && (u32)age <= max_sample_age_ticks;

    // minimum_drop absorbs jitter in the estimate itself - target energy is only ever a
    // HeuristicEnergyTracker guess, so a strict inequality would occasionally read noise as a hit.
    bool dropped = comparable && energy < previous_energy - minimum_drop;

    if (dropped) {
      ctx.blackboard.Set(output_key, true);
    } else {
      ctx.blackboard.Erase(output_key);
    }

    previous_id = target->id;
    previous_energy = energy;
    previous_tick = tick;
    has_previous = true;

    return dropped ? behavior::ExecuteResult::Success : behavior::ExecuteResult::Failure;
  }

  const char* target_player_key = nullptr;
  const char* target_energy_key = nullptr;
  const char* output_key = nullptr;
  float minimum_drop = 1.0f;
  u32 max_sample_age_ticks = 25;

  PlayerId previous_id = 0;
  float previous_energy = 0.0f;
  u32 previous_tick = 0;
  bool has_previous = false;
};

}  // namespace nexus
}  // namespace zero
