#pragma once

#include <zero/BotController.h>
#include <zero/ChatQueue.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>

#include <cstdio>

namespace zero {
namespace nexus {

// Announces a low-energy target to team chat so nearby human or bot teammates can help finish
// them off, rate-limited by a cooldown (`cooldown_key`/`cooldown_ticks`) so it doesn't spam chat
// every tick while we continue engaging the same weak target.
struct TeamCalloutNode : public behavior::BehaviorNode {
  TeamCalloutNode(const char* target_player_key, const char* cooldown_key, u32 cooldown_ticks)
      : target_player_key(target_player_key), cooldown_key(cooldown_key), cooldown_ticks(cooldown_ticks) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target || !*opt_target) return behavior::ExecuteResult::Failure;

    Player* target = *opt_target;

    u32 tick = GetCurrentTick();
    u32 timeout = ctx.blackboard.ValueOr<u32>(cooldown_key, 0U);
    if (!TICK_GTE(tick, timeout)) return behavior::ExecuteResult::Failure;

    char message[128];
    snprintf(message, sizeof(message), "Focus %s, low energy!", target->name);

    ctx.bot->bot_controller->chat_queue.SendTeam(message);

    ctx.blackboard.Set<u32>(cooldown_key, tick + cooldown_ticks);

    return behavior::ExecuteResult::Success;
  }

  const char* target_player_key = nullptr;
  const char* cooldown_key = nullptr;
  u32 cooldown_ticks = 0;
};

}  // namespace nexus
}  // namespace zero
