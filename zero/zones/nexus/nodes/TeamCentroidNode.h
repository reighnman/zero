#pragma once

#include <zero/BotController.h>
#include <zero/RegionRegistry.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Average position of our live teammates, excluding self - "where my team is". Fails when no
// teammate is alive, which correctly leaves a lone survivor to fight rather than rally to nobody.
//
// This exists because regrouping toward another *player* does not converge. The previous cohesion
// rule pathed to the second-nearest teammate, but that teammate is itself running its own cohesion
// rule toward a third player, so every bot chases a reference that is simultaneously running away.
// A mutual pursuit of moving points has no fixed point; the group diffuses instead of gathering.
// The centroid is a fixed point of exactly that process - if everyone moves toward it, everyone
// arrives - so it makes the same rule actually converge.
//
// Why this matters, from the 1-human-vs-7-bots replay (rec5), which ended 12-0:
//
//   * All 12 deaths happened with ZERO teammates within 25 tiles. Median distance from the victim
//     to their nearest teammate at the moment of death was 58.6 tiles, against a 31.5 tile
//     live-player baseline. Not one death was a fair fight lost - every one was someone caught
//     alone by two enemies (median enemies near the victim: 2).
//   * The teams split cleanly on cohesion and on nothing else. Median distance to nearest teammate
//     was 20.7-33.2 tiles for the winning side and 36.3-50.7 for the losing side; damage
//     dealt/taken ran ~6.0-7.6 / ~1.0-1.8 for the winners against ~2.5-2.8 / ~3.9-5.3 for the
//     losers. Both sides ran the same behavior, and the bots on the winning side matched or beat
//     the human on shooting (Gwythyr 35.6% bullet hit rate against phong's 27.7%).
//
// So the difference was not aim, movement or dodging - the losing team simply came apart. The
// winning team had a human on it holding a sensible position, which gave its bots a stable thing
// to cluster around. This node supplies that anchor when there is no human to provide one.
struct TeamCentroidNode : public behavior::BehaviorNode {
  TeamCentroidNode(const char* output_key) : output_key(output_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    Game& game = *ctx.bot->game;
    RegionRegistry& region_registry = *ctx.bot->bot_controller->region_registry;

    Vector2f sum(0, 0);
    size_t count = 0;

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

    if (count == 0) return behavior::ExecuteResult::Failure;

    ctx.blackboard.Set<Vector2f>(output_key, sum * (1.0f / (float)count));

    return behavior::ExecuteResult::Success;
  }

  const char* output_key = nullptr;
};

}  // namespace nexus
}  // namespace zero
