#pragma once

#include <zero/BotController.h>
#include <zero/RegionRegistry.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Succeeds when a bomb fired at `aim_key` would not catch us or a teammate in its explosion, and
// fails when it would - so it reads as a gate in the bomb fire sequence, and failing it simply
// leaves the bot to fire bullets instead.
//
// The mechanic this models, from the game code rather than from guesswork:
//
//   * A bomb does NOT detonate on a teammate. WeaponManager's collision scan skips any player on
//     the weapon's own frequency (`player->frequency == weapon.frequency` -> continue), so a
//     friendly standing in the flight path is passed straight through.
//   * The explosion, once it happens, damages everyone inside it regardless of frequency -
//     ShipController applies blast damage to self whenever the detonation is within
//     `BombExplodePixels`, with no team check. That is the team-damage path.
//
// So the question is not "is a teammate in the way" but "where will this bomb actually go off, and
// is one of ours standing there". Two kinds of detonation point are considered:
//
//   1. The aim point, where we expect it to reach the target.
//   2. Any point along the flight path where an *enemy* sits within proximity-trigger range, since
//      the bomb will fuse there instead of continuing to the target.
//
// Measured against the human 4v4 corpus, the bot needed this: humans fire bombs into a lane
// containing one of their own 7.8% of the time against 10.4% for bullets - they discriminate by
// weapon. The bot did the opposite, 11.6% for bombs against 10.0% for bullets, i.e. no
// discrimination at all, and its median teammate-to-firing-line distance when bombing was 13.9
// tiles against the humans' 21.7.
struct BombBlastSafetyNode : public behavior::BehaviorNode {
  BombBlastSafetyNode(const char* aim_key, float safety_margin)
      : aim_key(aim_key), safety_margin(safety_margin) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_aim = ctx.blackboard.Value<Vector2f>(aim_key);
    if (!opt_aim.has_value()) return behavior::ExecuteResult::Failure;

    Game& game = *ctx.bot->game;
    auto& settings = game.connection.settings;

    // Bomb level is the bomb upgrade count minus one, the same value ShipController stamps onto the
    // weapon when firing. No bombs at all means nothing to check.
    u32 bombs = game.ship_controller.ship.bombs;
    if (bombs == 0) return behavior::ExecuteResult::Success;

    u32 level = bombs - 1;

    // Blast damage reaches BombExplodePixels * (1 + level) pixels; 16 pixels to the tile.
    float blast_radius = (settings.BombExplodePixels + settings.BombExplodePixels * level) / 16.0f;
    float danger_radius = blast_radius + safety_margin;

    // Proximity fuse radius, matching WeaponManager's derivation.
    float prox_radius = ((settings.ProximityDistance + level) * 18.0f - 14.0f) / 16.0f;
    if (prox_radius < 0.0f) prox_radius = 0.0f;

    Vector2f from = self->position;
    Vector2f to = *opt_aim;

    Vector2f along = to - from;
    float length = along.Length();
    if (length < 1.0f) return behavior::ExecuteResult::Failure;

    Vector2f direction = along * (1.0f / length);

    // The aim point is always a candidate detonation site.
    if (IsFriendlyCaught(ctx, *self, to, danger_radius)) return behavior::ExecuteResult::Failure;

    // So is anywhere an enemy would trip the fuse early.
    for (size_t i = 0; i < game.player_manager.player_count; ++i) {
      Player* player = game.player_manager.players + i;

      if (player->id == self->id) continue;
      if (player->ship >= 8) continue;
      if (player->frequency == self->frequency) continue;
      if (player->IsRespawning()) continue;
      if (player->position == Vector2f(0, 0)) continue;

      float projected = (player->position - from).Dot(direction);
      if (projected < 0.0f || projected > length) continue;

      Vector2f closest = from + direction * projected;

      // GetRadius() is in PIXELS; positions and prox_radius are in tiles. Without the /16 this
      // added ~14 tiles of slop to the fuse check, so almost any enemy anywhere near the lane
      // counted as tripping the fuse early and the node then vetoed the shot on a friendly near
      // that imaginary detonation point. That is a large part of why the bots fire so few bombs.
      float ship_radius = settings.ShipSettings[player->ship].GetRadius() / 16.0f;
      if (closest.Distance(player->position) > prox_radius + ship_radius) continue;

      if (IsFriendlyCaught(ctx, *self, closest, danger_radius)) return behavior::ExecuteResult::Failure;
    }

    return behavior::ExecuteResult::Success;
  }

 private:
  // Self counts as a friendly here: our own blast hurts us just as much, and the shooter is the one
  // friendly guaranteed to be near the near end of the flight path.
  bool IsFriendlyCaught(behavior::ExecuteContext& ctx, Player& self, const Vector2f& detonation, float radius) {
    if (self.position.Distance(detonation) <= radius) return true;

    Game& game = *ctx.bot->game;

    for (size_t i = 0; i < game.player_manager.player_count; ++i) {
      Player* player = game.player_manager.players + i;

      if (player->id == self.id) continue;
      if (player->ship >= 8) continue;
      if (player->frequency != self.frequency) continue;
      if (player->IsRespawning()) continue;
      if (player->position == Vector2f(0, 0)) continue;

      // A teammate sitting on a safe tile can't be damaged, so they shouldn't veto the shot.
      if (game.connection.map.GetTileId(player->position) == kTileIdSafe) continue;

      if (player->position.Distance(detonation) <= radius) return true;
    }

    return false;
  }

  const char* aim_key = nullptr;
  float safety_margin = 0.0f;
};

}  // namespace nexus
}  // namespace zero
