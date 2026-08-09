#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>

namespace zero {
namespace nexus {

// Tracks whether we're at a meaningful energy disadvantage relative to the current target, instead
// of judging ourselves against a flat self-only threshold. Enters the disadvantaged state once our
// energy drops below `enter_ratio` of the target's estimated energy, and only leaves it once we
// recover past the higher `exit_ratio` - the gap between the two is a hysteresis band so the
// fight/flee decision doesn't flicker every tick near parity. A critical absolute energy percent is
// also an unconditional trigger, since being critically low is dangerous even against an equally
// weak target.
//
// Sets `state_key` on the blackboard (existence-based, like "rushing"/"spectating" elsewhere in
// this tree) while disadvantaged and erases it otherwise, and returns Success/Failure to match.
struct EnergyDisadvantageNode : public behavior::BehaviorNode {
  EnergyDisadvantageNode(const char* target_energy_key, const char* state_key, float enter_ratio, float exit_ratio,
                          float critical_energy_percent)
      : target_energy_key(target_energy_key),
        state_key(state_key),
        enter_ratio(enter_ratio),
        exit_ratio(exit_ratio),
        critical_energy_percent(critical_energy_percent) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_target_energy = ctx.blackboard.Value<float>(target_energy_key);
    if (!opt_target_energy) return behavior::ExecuteResult::Failure;

    float target_energy = *opt_target_energy;
    float max_energy = (float)ctx.bot->game->ship_controller.ship.energy;
    if (max_energy <= 0.0f) return behavior::ExecuteResult::Failure;

    bool currently_disadvantaged = ctx.blackboard.Has(state_key);
    bool critical = (self->energy / max_energy) < critical_energy_percent;
    float ratio = currently_disadvantaged ? exit_ratio : enter_ratio;
    bool disadvantaged = critical || self->energy < target_energy * ratio;

    if (disadvantaged) {
      ctx.blackboard.Set(state_key, true);
    } else {
      ctx.blackboard.Erase(state_key);
    }

    return disadvantaged ? behavior::ExecuteResult::Success : behavior::ExecuteResult::Failure;
  }

  const char* target_energy_key = nullptr;
  const char* state_key = nullptr;
  float enter_ratio = 0.75f;
  float exit_ratio = 1.0f;
  float critical_energy_percent = 0.2f;
};

}  // namespace nexus
}  // namespace zero
