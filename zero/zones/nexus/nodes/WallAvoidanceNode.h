#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>

namespace zero {
namespace nexus {

// Adds a steering force away from terrain we're about to run into, or that is closing in around us.
//
// IMPORTANT: this node **always returns Failure**, on purpose. It contributes to `steering.force`
// and then steps aside so whatever follows it still runs. Every call site looks like
// `Selector( WallAvoidance, Flee )`, so returning Failure means the flee/orbit/seek underneath
// still executes and its force sums with ours - wall awareness is *baked into* the movement rather
// than replacing it. That matters because the previous version took over: when it fired, the
// retreat never ran, the aim-and-shoot branch below it never ran either, and the bot stopped both
// defending and shooting for as long as it was near terrain. In tight geometry that was most of the
// time. Rotation is deliberately left alone here for the same reason - the movement node keeps
// facing the threat, and Actuator bends the heading toward the summed force on its own.
//
// The original version had three problems that between them made it both too eager and too late:
//
//   * It fired if *any* of 16 rays hit within wall_distance - including a wall directly behind us
//     that we were already moving away from. In tight terrain that is almost always true, so it
//     overrode steering more or less permanently, which is why the bots would not fight near walls.
//   * wall_distance was a fixed 5 tiles. At a fighting speed of ~20 tiles/sec that is a quarter of
//     a second of warning, far too late to turn a ship with real momentum. Detection has to scale
//     with how fast we are actually travelling.
//   * The escape heading was simply the most open ray, with no regard for where we were trying to
//     go, so escaping a wall could send us straight back across the enemy, or deeper into the
//     pocket we were already in.
//
// This version instead:
//   * Triggers primarily on a cast along our *velocity*, out to speed * lookahead_seconds, so it
//     reacts to terrain we are actually heading into and ignores terrain we are leaving behind.
//     A short fixed-radius check stays as a backstop for walls we are already scraping.
//   * Scores candidate escape headings by open distance, then biases toward `preferred_key` (the
//     retreat direction or the team) so getting off a wall keeps serving the current intent.
//   * Recognises being cornered - most directions blocked at short range - and in that case commits
//     hard to the widest opening and ignores the preference, because getting out at all is now the
//     only thing that matters. This is the case that was killing bots: they would settle into a
//     pocket, take fire, and then die on the way out. Reacting at half the search range means the
//     push out of a pocket starts while there is still somewhere to go, rather than once wedged.
struct WallAvoidanceNode : public behavior::BehaviorNode {
  WallAvoidanceNode(float wall_distance, float opening_distance, float lookahead_seconds = 0.9f,
                    const char* preferred_key = nullptr)
      : wall_distance(wall_distance),
        opening_distance(opening_distance),
        lookahead_seconds(lookahead_seconds),
        preferred_key(preferred_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;
    float radius = game.connection.settings.ShipSettings[self->ship].GetRadius();

    float speed = self->velocity.Length();

    // How far ahead terrain matters right now. Standing still we only care about contact range;
    // at speed we need enough room to actually turn.
    float lookahead = speed * lookahead_seconds;
    if (lookahead < wall_distance) lookahead = wall_distance;

    bool blocked_ahead = false;

    if (speed > 1.0f) {
      Vector2f travel = Normalize(self->velocity);
      CastResult ahead = game.GetMap().Cast(self->position + travel * radius, travel, lookahead, self->frequency);
      blocked_ahead = ahead.hit;
    }

    // Backstop: something is genuinely within touching distance, whatever direction we're going.
    bool scraping = IsWallWithin(game, *self, wall_distance);

    // How enclosed are we? Counted at half the search range, so a pocket registers before we're
    // wedged into the back of it.
    int blocked_count = 0;
    float best_open = 0.0f;
    Vector2f best_open_direction = self->GetHeading();

    ScanOpenings(game, *self, opening_distance, &blocked_count, &best_open, &best_open_direction);

    bool cornered = blocked_count >= kCorneredBlockedRays;

    if (!blocked_ahead && !scraping && !cornered) return behavior::ExecuteResult::Failure;

    Vector2f escape = best_open_direction;

    // When there's still room, keep serving whatever the tree was trying to do; when boxed in,
    // getting out is the only objective.
    if (!cornered && preferred_key) {
      auto opt_preferred = ctx.blackboard.Value<Vector2f>(preferred_key);

      if (opt_preferred.has_value()) {
        Vector2f to_preferred = *opt_preferred - self->position;

        if (to_preferred.LengthSq() > 1.0f) {
          escape = ChooseBiasedOpening(game, *self, opening_distance, Normalize(to_preferred));
        }
      }
    }

    auto& steering = ctx.bot->bot_controller->steering;

    // Avoid Seek here because it corrects for our current velocity, which fights the escape.
    // Scaled by severity: a wall coming up at range gets a nudge that the retreat can still steer
    // against, while being wedged gets a force large enough to dominate whatever else is pushing.
    steering.force += escape * (cornered || scraping ? 1000.0f : 400.0f);

    // Deliberately Failure - see the note at the top. We contribute force and let the movement node
    // underneath us keep running, instead of replacing the bot's defense and offense with a wall
    // reflex.
    return behavior::ExecuteResult::Failure;
  }

  float wall_distance = 0.0f;
  float opening_distance = 0.0f;
  float lookahead_seconds = 0.9f;
  const char* preferred_key = nullptr;

 private:
  static constexpr size_t kSampleCount = 16;
  static constexpr float kTwoPi = 6.28318f;

  // Of 16 directions, how many must be blocked at short range before we call it a pocket.
  static constexpr int kCorneredBlockedRays = 11;

  static bool IsWallWithin(Game& game, const Player& self, float distance) {
    float radius = game.connection.settings.ShipSettings[self.ship].GetRadius();

    for (size_t i = 0; i < kSampleCount; ++i) {
      float angle = (kTwoPi / kSampleCount) * i;
      Vector2f direction = Rotate(Vector2f(1, 0), angle);

      CastResult result = game.GetMap().Cast(self.position + direction * radius, direction, distance, self.frequency);
      if (result.hit) return true;
    }

    return false;
  }

  // Single sweep producing both the enclosure count and the most open heading, so we don't cast the
  // same 16 rays twice.
  static void ScanOpenings(Game& game, const Player& self, float opening_distance, int* blocked_count,
                           float* best_open, Vector2f* best_direction) {
    float radius = game.connection.settings.ShipSettings[self.ship].GetRadius();
    float enclosure_range = opening_distance * 0.5f;

    *blocked_count = 0;
    *best_open = -1.0f;
    *best_direction = self.GetHeading();

    for (size_t i = 0; i < kSampleCount; ++i) {
      float angle = (kTwoPi / kSampleCount) * i;
      Vector2f direction = Rotate(Vector2f(1, 0), angle);

      CastResult result =
          game.GetMap().Cast(self.position + direction * radius, direction, opening_distance, self.frequency);
      float distance = result.hit ? result.distance : opening_distance;

      if (distance < enclosure_range) ++(*blocked_count);

      if (distance > *best_open) {
        *best_open = distance;
        *best_direction = direction;
      }
    }
  }

  // Most open heading, weighted toward one we'd rather be travelling in anyway.
  static Vector2f ChooseBiasedOpening(Game& game, const Player& self, float opening_distance,
                                      const Vector2f& preferred) {
    float radius = game.connection.settings.ShipSettings[self.ship].GetRadius();

    Vector2f best_direction = self.GetHeading();
    float best_score = -1.0f;

    for (size_t i = 0; i < kSampleCount; ++i) {
      float angle = (kTwoPi / kSampleCount) * i;
      Vector2f direction = Rotate(Vector2f(1, 0), angle);

      CastResult result =
          game.GetMap().Cast(self.position + direction * radius, direction, opening_distance, self.frequency);
      float distance = result.hit ? result.distance : opening_distance;

      // Openness is the requirement; alignment with where we wanted to go breaks the ties. A
      // direction pointing away from the preference keeps its raw openness, so a dead end never
      // wins just for pointing the right way.
      float alignment = direction.Dot(preferred);
      if (alignment < 0.0f) alignment = 0.0f;

      float score = distance * (1.0f + alignment);

      if (score > best_score) {
        best_score = score;
        best_direction = direction;
      }
    }

    return best_direction;
  }
};

}  // namespace nexus
}  // namespace zero
