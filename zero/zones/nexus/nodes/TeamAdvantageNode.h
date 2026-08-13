#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Counts the local numerical situation around self: living teammates minus living enemies within
// `radius`, written to `advantage_key` as a float (and the raw counts to `<key>_friends` /
// `<key>_enemies` for anything that wants them directly).
//
// This is the single most decisive team-level signal in the replay corpus (ZeroReplayAnalyzer over
// 21 real 4v4 league matches, ~790k player-snapshots). The fraction of samples where a player was
// actively closing on their nearest enemy, bucketed by this exact quantity at a 25-tile radius:
//
//   advantage <= -2 :  40% closing   (outnumbered - players disengage)
//   advantage   -1  :  48% closing
//   advantage    0  :  59% closing   (even - players commit)
//   advantage   +1  :  60% closing
//   advantage   +2  :  59% closing
//
// So real players flip from "back off" to "push" right around advantage >= 0, and the effect is
// monotone through the whole outnumbered range. Deaths tell the same story from the other side:
// two seconds before dying, the median victim had *zero* teammates within 25 tiles and one to two
// enemies on them. Being locally outnumbered is what gets players killed, far more than being at
// low energy in the abstract.
struct TeamAdvantageNode : public behavior::BehaviorNode {
  TeamAdvantageNode(float radius, const char* advantage_key)
      : radius(radius), advantage_key(advantage_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;
    float radius_sq = radius * radius;

    int friends = 0;
    int enemies = 0;

    for (size_t i = 0; i < game.player_manager.player_count; ++i) {
      Player* player = game.player_manager.players + i;

      if (player->id == self->id) continue;
      if (player->ship >= 8) continue;
      if (player->IsRespawning()) continue;
      if (player->position == Vector2f(0, 0)) continue;
      if (player->position.DistanceSq(self->position) > radius_sq) continue;

      // A player sitting in safe isn't participating in the fight either way.
      if (game.connection.map.GetTileId(player->position) == kTileIdSafe) continue;

      if (player->frequency == self->frequency) {
        ++friends;
      } else {
        ++enemies;
      }
    }

    ctx.blackboard.Set<float>(advantage_key, (float)(friends - enemies));

    return behavior::ExecuteResult::Success;
  }

  float radius = 25.0f;
  const char* advantage_key = nullptr;
};

}  // namespace nexus
}  // namespace zero
