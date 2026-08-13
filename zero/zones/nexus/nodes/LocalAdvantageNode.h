#pragma once

#include <zero/BotController.h>
#include <zero/RegionRegistry.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Counts live teammates minus live enemies inside `radius` of self and publishes the difference as
// a float, so the tree can gate on it with an ordinary ScalarThresholdNode.
//
// This exists because range alone is the wrong axis for deciding how close to fight. Measured over
// the 4v4 replay corpus, the energy exchange (damage dealt to the nearest enemy vs damage taken,
// over the following second) splits almost entirely on local head-count rather than on distance:
//
//   advantage:      -2      -1       0      +1      +2      <- (friends - enemies) within 25 tiles
//    5-9 tiles:   0.57    1.10    3.07    4.26       -      <- ratio, >1.0 means we win the trade
//   10-14 tiles:   0.58    1.13    2.46    3.55    9.32
//   15-19 tiles:   0.68    1.21    1.96    2.89    4.32
//   20-24 tiles:   0.67    1.21    1.94    2.53    3.54
//
// So closing in is the single most profitable thing to do at parity or better, and the single most
// expensive thing to do while outnumbered - the same distance flips from a 4:1 win to a 1:2 loss
// purely on who else is nearby. A fixed orbit distance cannot express that; this can.
//
// Self is excluded from the friendly count, so the value is a true differential: 0 means the
// remaining teammates and enemies nearby cancel out and it's our presence that tips the fight.
struct LocalAdvantageNode : public behavior::BehaviorNode {
  LocalAdvantageNode(float radius, const char* output_key) : radius(radius), output_key(output_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    Game& game = *ctx.bot->game;
    RegionRegistry& region_registry = *ctx.bot->bot_controller->region_registry;

    float radius_sq = radius * radius;
    int friends = 0;
    int enemies = 0;

    for (size_t i = 0; i < game.player_manager.player_count; ++i) {
      Player* player = game.player_manager.players + i;

      if (player->id == self->id) continue;
      if (player->ship >= 8) continue;
      if (player->IsRespawning()) continue;
      if (player->position == Vector2f(0, 0)) continue;
      if (!IsSynchronized(game, *player)) continue;
      if (!region_registry.IsConnected(self->position, player->position)) continue;
      if (game.connection.map.GetTileId(player->position) == kTileIdSafe) continue;

      if (player->position.DistanceSq(self->position) > radius_sq) continue;

      if (player->frequency == self->frequency) {
        ++friends;
      } else {
        ++enemies;
      }
    }

    ctx.blackboard.Set<float>(output_key, (float)(friends - enemies));

    return behavior::ExecuteResult::Success;
  }

 private:
  inline bool IsSynchronized(Game& game, Player& player) {
    // If the player is within our view, but we haven't received any packets, then they left where we
    // last saw them and should be ignored.
    if (game.radar.InRadarView(player.position)) {
      return game.player_manager.IsSynchronized(player);
    }

    return true;
  }

  float radius = 0.0f;
  const char* output_key = nullptr;
};

}  // namespace nexus
}  // namespace zero
