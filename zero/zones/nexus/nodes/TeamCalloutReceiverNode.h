#pragma once

#include <zero/BotController.h>
#include <zero/ChatQueue.h>
#include <zero/RegionRegistry.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>

#include <cstdio>

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
// override, and acknowledges the callout in team chat once per target actually confirmed in range.
//
// Applies the same validity checks LowestTargetNode/NearestMemoryTargetNode use elsewhere in these
// trees (respawning, zeroed position, network sync, pathfinding-reachable, not in a safe tile) so a
// teammate's callout can't hand us a target we have no real way to act on.
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

    auto& game = *ctx.bot->game;

    Player* callout_target = game.player_manager.GetPlayerByName(callout_name.c_str());
    if (!IsValidTarget(game, *self, callout_target, *ctx.bot->bot_controller->region_registry)) {
      return behavior::ExecuteResult::Failure;
    }

    if (self->position.DistanceSq(callout_target->position) > range * range) {
      return behavior::ExecuteResult::Failure;
    }

    // Let the team know the callout was picked up - once per target we actually confirm and start
    // prioritizing, not every tick we continue to act on it.
    std::string acknowledged_name =
        ctx.blackboard.ValueOr<std::string>("team_callout_acknowledged_name", std::string());

    if (callout_name != acknowledged_name) {
      ctx.blackboard.Set<std::string>("team_callout_acknowledged_name", callout_name);

      char message[64];
      snprintf(message, sizeof(message), "On it, %s!", callout_target->name);
      ctx.bot->bot_controller->chat_queue.SendFrequency(self->frequency, message);
    }

    ctx.blackboard.Set(target_output_key, callout_target);

    return behavior::ExecuteResult::Success;
  }

  const char* target_output_key = nullptr;
  float range = 0.0f;
  u32 priority_ticks = 0;

 private:
  // Mirrors LowestTargetNode/NearestMemoryTargetNode's validity checks so a teammate's callout
  // can't override us onto a target those nodes would never have picked themselves.
  static bool IsValidTarget(Game& game, const Player& self, Player* target, RegionRegistry& region_registry) {
    if (!target) return false;
    if (target->ship >= 8) return false;
    if (target->frequency == self.frequency) return false;
    if (target->IsRespawning()) return false;
    if (target->position == Vector2f(0, 0)) return false;
    if (!IsSynchronized(game, *target)) return false;
    if (!region_registry.IsConnected(self.position, target->position)) return false;

    bool in_safe = game.connection.map.GetTileId(target->position) == kTileIdSafe;
    if (in_safe) return false;

    return true;
  }

  static bool IsSynchronized(Game& game, Player& player) {
    // If the player is within our view, but we haven't received any packets, then they left where
    // we last saw them and should be ignored.
    if (game.radar.InRadarView(player.position)) {
      return game.player_manager.IsSynchronized(player);
    }

    // Try to path to where we last saw the player until their old position is in view.
    return true;
  }
};

}  // namespace nexus
}  // namespace zero
