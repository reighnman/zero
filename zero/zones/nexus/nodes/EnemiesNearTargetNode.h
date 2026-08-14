#pragma once

#include <zero/BotController.h>
#include <zero/RegionRegistry.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Counts live enemies clustered within `radius` of the target, including the target itself, and
// publishes it as a float for the tree to threshold on.
//
// This exists to decide multifire. Multifire spreads shots into a fan instead of a single line: it
// costs more energy per trigger pull and is worse against one target, but it covers ground rather
// than a point, so it pays exactly when several enemies are bunched together and a spread can catch
// more than one of them.
//
// The previous rule keyed multifire off raw distance to the target - on beyond 35 tiles, off inside
// it - which had become dead logic, since bullets are no longer fired past kMaxBulletRange (35)
// at all. So multifire was effectively never on.
struct EnemiesNearTargetNode : public behavior::BehaviorNode {
  EnemiesNearTargetNode(const char* target_player_key, float radius, const char* output_key)
      : target_player_key(target_player_key), radius(radius), output_key(output_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target.has_value()) return behavior::ExecuteResult::Failure;

    Player* target = opt_target.value();
    if (!target) return behavior::ExecuteResult::Failure;

    Game& game = *ctx.bot->game;
    RegionRegistry& region_registry = *ctx.bot->bot_controller->region_registry;

    float radius_sq = radius * radius;
    int count = 0;

    for (size_t i = 0; i < game.player_manager.player_count; ++i) {
      Player* player = game.player_manager.players + i;

      if (player->ship >= 8) continue;
      if (player->frequency == self->frequency) continue;
      if (player->IsRespawning()) continue;
      if (player->position == Vector2f(0, 0)) continue;
      if (!region_registry.IsConnected(self->position, player->position)) continue;
      if (game.connection.map.GetTileId(player->position) == kTileIdSafe) continue;

      if (player->position.DistanceSq(target->position) > radius_sq) continue;

      ++count;
    }

    ctx.blackboard.Set<float>(output_key, (float)count);

    return behavior::ExecuteResult::Success;
  }

  const char* target_player_key = nullptr;
  float radius = 0.0f;
  const char* output_key = nullptr;
};

}  // namespace nexus
}  // namespace zero
