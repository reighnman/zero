#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// The fight state a tick falls into, in priority order. Everything downstream (movement, weapon
// gating, broadside-vs-face) dispatches off this single value instead of each caring node
// re-deriving its own overlapping distance/energy conditions, which is what FoursBehavior.cpp did
// across four independently-triggered sequences (rush / press-advantage / energy-disadvantage /
// default orbit).
enum class EngagementPhase {
  // Default fight/orbit state - none of the below apply.
  Engaged,
  // Our own energy is low relative to the target (or in absolute terms), so we're backing off to
  // recharge instead of pressing the fight.
  Disadvantaged,
  // The target's energy visibly dropped while we still have more than they do - worth closing in
  // and finishing rather than continuing to orbit/kite.
  Press,
  // The target is already close to dead and within finishing range - commit fully.
  Finish,
};

// Classifies the current tick into one EngagementPhase and writes it to `phase_key`.
//
// Every threshold here comes from ZeroReplayAnalyzer (see FoursBehaviorV2.cpp) run against 21 real
// 4v4 SVS league replays:
//   - Finish: the player who actually got killed was, at the moment of death, at a median of just
//     10% of their own max energy (p75 20%), and the killer-to-target distance at that moment had
//     a median of ~11 units. `finish_target_percent`/`finish_distance` recognize that exact
//     situation instead of a guessed absolute energy value.
//   - Press: the target's energy dropped this tick (hit or spent shooting) while we still have
//     more than they do, and we're still within `press_distance` of them - re-arms a
//     `press_ticks` window on each fresh drop rather than reacting for only the single tick the
//     drop is visible.
//   - Disadvantaged: the same relative-energy hysteresis EnergyDisadvantageNode uses (enter/exit
//     ratio band, plus an absolute critical floor), folded in here so callers get one phase value
//     instead of a separate flag to check in addition to this one.
struct EngagementPhaseNode : public behavior::BehaviorNode {
  EngagementPhaseNode(const char* target_player_key, const char* target_energy_key,
                       const char* target_energy_prev_key, const char* phase_key, const char* disadvantage_state_key,
                       const char* press_until_key, float finish_target_percent, float finish_distance,
                       float press_distance, u32 press_ticks, float disadvantage_enter_ratio,
                       float disadvantage_exit_ratio, float critical_energy_percent)
      : target_player_key(target_player_key),
        target_energy_key(target_energy_key),
        target_energy_prev_key(target_energy_prev_key),
        phase_key(phase_key),
        disadvantage_state_key(disadvantage_state_key),
        press_until_key(press_until_key),
        finish_target_percent(finish_target_percent),
        finish_distance(finish_distance),
        press_distance(press_distance),
        press_ticks(press_ticks),
        disadvantage_enter_ratio(disadvantage_enter_ratio),
        disadvantage_exit_ratio(disadvantage_exit_ratio),
        critical_energy_percent(critical_energy_percent) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target || !*opt_target) return behavior::ExecuteResult::Failure;
    Player* target = *opt_target;
    if (target->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_target_energy = ctx.blackboard.Value<float>(target_energy_key);
    if (!opt_target_energy) return behavior::ExecuteResult::Failure;
    float target_energy = *opt_target_energy;

    float self_max_energy = (float)ctx.bot->game->ship_controller.ship.energy;
    float target_max_energy = (float)ctx.bot->game->connection.settings.ShipSettings[target->ship].MaximumEnergy;
    if (self_max_energy <= 0.0f || target_max_energy <= 0.0f) return behavior::ExecuteResult::Failure;

    float self_percent = self->energy / self_max_energy;
    float target_percent = target_energy / target_max_energy;
    float distance = self->position.Distance(target->position);

    EngagementPhase phase = EngagementPhase::Engaged;

    // Disadvantage hysteresis lives in its own dedicated key rather than being read back out of
    // phase_key. phase_key holds the *winning* phase for the tick, so a tick that resolves to
    // Press or Finish would otherwise erase the memory that we were disadvantaged and silently
    // collapse the enter/exit band down to a single threshold.
    bool was_disadvantaged = ctx.blackboard.Has(disadvantage_state_key);
    bool critical = self_percent < critical_energy_percent;
    float ratio = was_disadvantaged ? disadvantage_exit_ratio : disadvantage_enter_ratio;
    bool disadvantaged = critical || self_percent < target_percent * ratio;

    if (disadvantaged) {
      ctx.blackboard.Set(disadvantage_state_key, true);
      phase = EngagementPhase::Disadvantaged;
    } else {
      ctx.blackboard.Erase(disadvantage_state_key);
    }

    // Press: re-arm the window whenever a fresh energy drop is seen while we're still ahead.
    auto opt_target_energy_prev = ctx.blackboard.Value<float>(target_energy_prev_key);
    if (opt_target_energy_prev && target_energy < *opt_target_energy_prev && self->energy > target_energy &&
        distance < press_distance) {
      ctx.blackboard.Set<u32>(press_until_key, GetCurrentTick() + press_ticks);
    }
    bool pressing = ctx.blackboard.Value<u32>(press_until_key).value_or(0) > GetCurrentTick();
    if (pressing) phase = EngagementPhase::Press;

    // Finish takes priority over everything - a target this close to dead and this close to us is
    // always worth committing to, even out of a disadvantaged state (see kRushMinEnergyPercent-
    // style self-energy floor still checked separately before we act on this phase).
    if (target_percent < finish_target_percent && distance < finish_distance) {
      phase = EngagementPhase::Finish;
    }

    ctx.blackboard.Set(phase_key, phase);

    return behavior::ExecuteResult::Success;
  }

  const char* target_player_key = nullptr;
  const char* target_energy_key = nullptr;
  const char* target_energy_prev_key = nullptr;
  const char* phase_key = nullptr;
  const char* disadvantage_state_key = nullptr;
  const char* press_until_key = nullptr;
  float finish_target_percent = 0.2f;
  float finish_distance = 12.0f;
  float press_distance = 56.0f;
  u32 press_ticks = 300;
  float disadvantage_enter_ratio = 0.65f;
  float disadvantage_exit_ratio = 0.9f;
  float critical_energy_percent = 0.1f;
};

}  // namespace nexus
}  // namespace zero
