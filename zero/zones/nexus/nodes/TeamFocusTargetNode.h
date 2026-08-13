#pragma once

#include <zero/BotController.h>
#include <zero/RegionRegistry.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Picks the enemy nearest to our team's centre of mass, rather than the one nearest to us.
//
// This is how a group converges on one target without any communication between bots. Every member
// of the team computes very nearly the same centroid, so every member picks very nearly the same
// enemy - agreement falls out of the shared reference point instead of needing chat, a leader, or a
// shared blackboard. It is the same trick TeamCentroidNode uses for cohesion, applied to target
// selection.
//
// The measured gap this closes, from the 1-human-vs-7-bots replay (rec9, 12-5): the share of
// engaged time spent attacking the same enemy as at least one teammate ran ~72% for the human's
// team (74.8 / 71.3 / 65.7 / 74.9) against ~61% for the all-bot team (64.2 / 68.7 / 53.5 / 56.4).
// The 21-match human league corpus sits at 58-75%, so ~72% is normal competent play and the bots
// were the outlier. Each bot independently chasing its own nearest enemy is what pulls a team apart
// into four separate duels, which is the shape the corpus says loses - see the isolation findings.
//
// Attack *arc* is expected to tighten as a consequence rather than needing its own rule: teammates
// that are already clustered (TeamCentroidNode) and now also agree on the target will naturally
// approach it from a common side. League humans hold a median 47-73 degrees of spread between
// attackers against the bots' 69-89, so if that gap does not close on its own, an explicit
// same-side approach anchor is the next thing to try.
struct TeamFocusTargetNode : public behavior::BehaviorNode {
  TeamFocusTargetNode(const char* player_key) : player_key(player_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    Game& game = *ctx.bot->game;
    RegionRegistry& region_registry = *ctx.bot->bot_controller->region_registry;

    // Centre of mass of the whole live team, self included - self is part of the group whose
    // collective position decides what the group should be hitting.
    Vector2f sum = self->position;
    size_t count = 1;

    for (size_t i = 0; i < game.player_manager.player_count; ++i) {
      Player* player = game.player_manager.players + i;

      if (player->id == self->id) continue;
      if (player->ship >= 8) continue;
      if (player->frequency != self->frequency) continue;
      if (player->IsRespawning()) continue;
      if (player->position == Vector2f(0, 0)) continue;
      if (!region_registry.IsConnected(self->position, player->position)) continue;

      sum += player->position;
      ++count;
    }

    Vector2f centroid = sum * (1.0f / (float)count);

    Player* best = nullptr;
    float best_distance_sq = std::numeric_limits<float>::max();

    for (size_t i = 0; i < game.player_manager.player_count; ++i) {
      Player* player = game.player_manager.players + i;

      if (player->ship >= 8) continue;
      if (player->frequency == self->frequency) continue;
      if (player->IsRespawning()) continue;
      if (player->position == Vector2f(0, 0)) continue;
      if (!IsSynchronized(game, *player)) continue;
      if (!region_registry.IsConnected(self->position, player->position)) continue;
      if (game.connection.map.GetTileId(player->position) == kTileIdSafe) continue;

      float distance_sq = player->position.DistanceSq(centroid);

      if (distance_sq < best_distance_sq) {
        best_distance_sq = distance_sq;
        best = player;
      }
    }

    if (!best) return behavior::ExecuteResult::Failure;

    ctx.blackboard.Set(player_key, best);

    return behavior::ExecuteResult::Success;
  }

 private:
  inline bool IsSynchronized(Game& game, Player& player) {
    if (game.radar.InRadarView(player.position)) {
      return game.player_manager.IsSynchronized(player);
    }

    return true;
  }

  const char* player_key = nullptr;
};

}  // namespace nexus
}  // namespace zero
