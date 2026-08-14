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
// energy percent drops below `enter_ratio` of the target's estimated energy percent, and only
// leaves it once we recover past the higher `exit_ratio` - the gap between the two is a hysteresis
// band so the fight/flee decision doesn't flicker every tick near parity. A critical absolute
// energy percent is also an unconditional trigger, since being critically low is dangerous even
// against an equally weak target.
//
// Compares percentages of each ship's own true maximum energy rather than raw values. The target's
// energy is only ever an estimate (HeuristicEnergyTracker) since the server doesn't broadcast an
// enemy's exact energy, and with the "Average" heuristic type that estimate is capped at
// (InitialEnergy + MaximumEnergy) / 2 - well below the ship's true maximum. Comparing that capped
// estimate directly against our own exact, uncapped energy meant self almost always looked behind,
// and the hysteresis made that state nearly impossible to escape. Normalizing both to a percent of
// their own ship's true maximum fixes the scale mismatch.
//
// Sets `state_key` on the blackboard (existence-based, like "rushing"/"spectating" elsewhere in
// this tree) while disadvantaged and erases it otherwise, and returns Success/Failure to match.
// `recovery_energy_percent` is an ABSOLUTE floor on leaving the state, and it exists because the
// ratio test alone answers the wrong question. "I am no longer losing the comparison" is not the
// same claim as "I am fit to fight", and against an equally-hurt opponent they diverge completely:
// while we are away recharging, so is the enemy, so the ratio can satisfy with both sides near
// dead and the bot turns straight back into a fight it is still too weak for.
//
// Measured in rec28, after the flee distance was widened: bots retreated *more* (51-64% of hurt
// time, up from 25-57%) but still died at 5.9-13.2% energy, and their retreats plateaued at 36
// tiles against a ramp asking for 49-55. Travelling from orbit range out to 55 tiles takes about
// 2.7s at cruising speed while recharge_timer only holds for 2s past the last disadvantaged tick -
// so the retreat was ending before it arrived. Distance was no longer the binding constraint;
// duration was, and duration is set by this exit condition.
struct EnergyDisadvantageNode : public behavior::BehaviorNode {
  EnergyDisadvantageNode(const char* target_player_key, const char* target_energy_key, const char* state_key,
                          float enter_ratio, float exit_ratio, float critical_energy_percent,
                          float recovery_energy_percent = 0.0f)
      : target_player_key(target_player_key),
        target_energy_key(target_energy_key),
        state_key(state_key),
        enter_ratio(enter_ratio),
        exit_ratio(exit_ratio),
        critical_energy_percent(critical_energy_percent),
        recovery_energy_percent(recovery_energy_percent) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target || !*opt_target) return behavior::ExecuteResult::Failure;

    Player* target = *opt_target;
    if (target->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_target_energy = ctx.blackboard.Value<float>(target_energy_key);
    if (!opt_target_energy) return behavior::ExecuteResult::Failure;

    float self_max_energy = (float)ctx.bot->game->ship_controller.ship.energy;
    float target_max_energy = (float)ctx.bot->game->connection.settings.ShipSettings[target->ship].MaximumEnergy;
    if (self_max_energy <= 0.0f || target_max_energy <= 0.0f) return behavior::ExecuteResult::Failure;

    float self_percent = self->energy / self_max_energy;
    float target_percent = *opt_target_energy / target_max_energy;

    bool currently_disadvantaged = ctx.blackboard.Has(state_key);
    bool critical = self_percent < critical_energy_percent;
    float ratio = currently_disadvantaged ? exit_ratio : enter_ratio;

    // Only holds us in, never puts us in - entering is still decided by the comparison against the
    // target, so this cannot make a healthy bot start retreating.
    bool not_recovered = currently_disadvantaged && self_percent < recovery_energy_percent;

    bool disadvantaged = critical || not_recovered || self_percent < target_percent * ratio;

    if (disadvantaged) {
      ctx.blackboard.Set(state_key, true);
    } else {
      ctx.blackboard.Erase(state_key);
    }

    return disadvantaged ? behavior::ExecuteResult::Success : behavior::ExecuteResult::Failure;
  }

  const char* target_player_key = nullptr;
  const char* target_energy_key = nullptr;
  const char* state_key = nullptr;
  float enter_ratio = 0.75f;
  float exit_ratio = 1.0f;
  float critical_energy_percent = 0.2f;
  float recovery_energy_percent = 0.0f;
};

}  // namespace nexus
}  // namespace zero
