#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>

namespace zero {
namespace nexus {

// Detects when a wall is close enough to be a problem and, if so, overrides steering toward the
// most open nearby direction until it isn't.
//
// This exists to be checked ahead of movement nodes like FleeNode: a plain retreat/seek force has
// no awareness of the map, so backing straight away from a threat can drive the ship into a wall
// or corner and pin it there. Placing this node earlier in a Selector lets it intercept and
// override steering before that happens, rather than reacting only after the ship is already stuck
// bouncing off of something.
//
// `wall_distance` is how close a wall has to be before this triggers at all - keep this small so
// it only fires when a wall is actually in the way, not just anywhere nearby. `opening_distance` is
// how far out to search for the best escape heading once triggered - keep this larger so it finds
// genuinely open space rather than just the nearest gap.
struct WallAvoidanceNode : public behavior::BehaviorNode {
  WallAvoidanceNode(float wall_distance, float opening_distance)
      : wall_distance(wall_distance), opening_distance(opening_distance) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;

    if (!IsWallNearby(game, *self, wall_distance)) return behavior::ExecuteResult::Failure;

    Vector2f opening_direction = FindOpenDirection(game, *self, opening_distance);
    auto& steering = ctx.bot->bot_controller->steering;

    steering.Face(game, self->position + opening_direction);
    // Avoid Seek here because it corrects for our current velocity, which fights the escape.
    steering.force += opening_direction * 1000.0f;

    return behavior::ExecuteResult::Success;
  }

  float wall_distance = 0.0f;
  float opening_distance = 0.0f;

 private:
  // Casts a ring of rays out to `wall_distance` and returns true if any of them hit a wall.
  static bool IsWallNearby(Game& game, const Player& self, float wall_distance) {
    constexpr size_t kSampleCount = 16;
    constexpr float kTwoPi = 6.28318f;

    float radius = game.connection.settings.ShipSettings[self.ship].GetRadius();

    for (size_t i = 0; i < kSampleCount; ++i) {
      float angle = (kTwoPi / kSampleCount) * i;
      Vector2f direction = Rotate(Vector2f(1, 0), angle);
      Vector2f start = self.position + direction * radius;

      CastResult result = game.GetMap().Cast(start, direction, wall_distance, self.frequency);
      if (result.hit) return true;
    }

    return false;
  }

  // Casts a ring of rays out to `opening_distance` and returns the direction with the most open
  // space.
  static Vector2f FindOpenDirection(Game& game, const Player& self, float opening_distance) {
    constexpr size_t kSampleCount = 16;
    constexpr float kTwoPi = 6.28318f;

    float radius = game.connection.settings.ShipSettings[self.ship].GetRadius();

    Vector2f best_direction = self.GetHeading();
    float best_distance = -1.0f;

    for (size_t i = 0; i < kSampleCount; ++i) {
      float angle = (kTwoPi / kSampleCount) * i;
      Vector2f direction = Rotate(Vector2f(1, 0), angle);
      Vector2f start = self.position + direction * radius;

      CastResult result = game.GetMap().Cast(start, direction, opening_distance, self.frequency);
      float distance = result.hit ? result.distance : opening_distance;

      if (distance > best_distance) {
        best_distance = distance;
        best_direction = direction;
      }
    }

    return best_direction;
  }
};

}  // namespace nexus
}  // namespace zero
