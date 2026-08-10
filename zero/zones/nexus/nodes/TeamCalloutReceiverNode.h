#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>

namespace zero {
namespace nexus {

// Reacts to a low-energy target called out by a teammate over team chat (see TeamCalloutNode,
// which sends the message, and Nexus.cpp's ChatEvent handler, which parses it into
// "team_callout_target_name" on the blackboard).
//
// Only actually prioritizes the named target if they're within `range` of self, and only for
// `priority_ticks` after we first notice a given callout - a teammate re-broadcasting the same
// target doesn't restart the window, but a new target name does. On success, writes the resolved
// Player* to `target_output_key` so the caller can treat it exactly like any other target
// override (LowestTargetNode elsewhere in these trees does the same thing).
struct TeamCalloutReceiverNode : public behavior::BehaviorNode {
  TeamCalloutReceiverNode(const char* target_output_key, float range, u32 priority_ticks)
      : target_output_key(target_output_key), range(range), priority_ticks(priority_ticks) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_callout_name = ctx.blackboard.Value<std::string>("team_callout_target_name");
    if (!opt_callout_name || opt_callout_name->empty()) return behavior::ExecuteResult::Failure;

    const std::string& callout_name = *opt_callout_name;
    u32 tick = GetCurrentTick();

    // Only (re)arm the priority window when this is a different target than the last one we
    // reacted to, so a teammate repeating the same callout doesn't extend it indefinitely.
    std::string last_name = ctx.blackboard.ValueOr<std::string>("team_callout_last_name", std::string());

    if (callout_name != last_name) {
      ctx.blackboard.Set<std::string>("team_callout_last_name", callout_name);
      ctx.blackboard.Set<u32>("team_callout_priority_expiry", tick + priority_ticks);
    }

    u32 expiry = ctx.blackboard.ValueOr<u32>("team_callout_priority_expiry", 0U);
    if (TICK_GTE(tick, expiry)) return behavior::ExecuteResult::Failure;

    Player* callout_target = ctx.bot->game->player_manager.GetPlayerByName(callout_name.c_str());
    if (!callout_target || callout_target->ship >= 8) return behavior::ExecuteResult::Failure;
    if (callout_target->frequency == self->frequency) return behavior::ExecuteResult::Failure;

    if (self->position.DistanceSq(callout_target->position) > range * range) {
      return behavior::ExecuteResult::Failure;
    }

    ctx.blackboard.Set(target_output_key, callout_target);

    return behavior::ExecuteResult::Success;
  }

  const char* target_output_key = nullptr;
  float range = 0.0f;
  u32 priority_ticks = 0;
};

}  // namespace nexus
}  // namespace zero
