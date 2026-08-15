#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace teamversus {

// Detects being backed against terrain and, when it happens, overrides movement toward the most
// open direction that still keeps us pointed roughly at the fight.
//
// Two things distinguish this from a plain wall-avoidance steering force:
//
//  1. It is a *gate*, meant to sit ahead of the normal movement branch in a Selector. Blending an
//     avoidance force into a movement force that is actively pushing into a corner just produces a
//     smaller push into the corner. Once terrain is genuinely in the way, terrain wins outright
//     until it isn't.
//
//  2. It scores escape directions by openness *and* by whether they keep the target in front of us.
//     Openness alone will happily send us out the back of a corridor, away from the fight, which
//     both abandons the position and turns a recoverable pin into a long walk back. Weighting by
//     bearing to the target means that among comparably open directions we take the one that keeps
//     us in the engagement.
//
// Walls in this game bounce rather than stop, so being near one is not itself dangerous. What is
// dangerous is having a retreat vector shoved somewhere unintended by a bounce, and being pinned in
// a pocket where a bomb's blast cannot be escaped. Hence the trigger distance is deliberately
// small: this should fire when a wall is actually in the way, not merely nearby.
struct TerrainAvoidNode : public behavior::BehaviorNode {
  TerrainAvoidNode(float wall_distance, float opening_distance, const char* toward_key = nullptr)
      : wall_distance(wall_distance), opening_distance(opening_distance), toward_key(toward_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;
    float radius = game.connection.settings.ShipSettings[self->ship].GetRadius();

    // Only intervene when we are actually moving into something. A wall beside us that we are
    // travelling parallel to is not a problem and does not need a correction.
    Vector2f travel = self->velocity;
    if (travel.LengthSq() < 1.0f) travel = self->GetHeading();
    travel = Normalize(travel);

    CastResult ahead = game.GetMap().Cast(self->position + travel * radius, travel, wall_distance, self->frequency);
    if (!ahead.hit) return behavior::ExecuteResult::Failure;

    Vector2f toward;
    bool has_toward = false;

    if (toward_key) {
      auto opt_toward = ctx.blackboard.Value<Vector2f>(toward_key);
      if (opt_toward) {
        Vector2f delta = *opt_toward - self->position;
        if (delta.LengthSq() > 0.0f) {
          toward = Normalize(delta);
          has_toward = true;
        }
      }
    }

    Vector2f escape = FindEscapeDirection(game, *self, radius, toward, has_toward);

    auto& steering = ctx.bot->bot_controller->steering;

    steering.Face(game, self->position + escape);
    // Deliberately not Seek: Seek corrects for current velocity, and current velocity is exactly
    // what is driving us into the wall, so it would fight the escape it is supposed to be making.
    steering.force += escape * escape_force;
    // Loosen the rotation clamp so the hull is free to turn all the way to the escape heading
    // instead of being held near a now-irrelevant aim point.
    steering.SetRotationThreshold(0.0f);

    return behavior::ExecuteResult::Success;
  }

  float wall_distance = 5.0f;
  float opening_distance = 35.0f;
  const char* toward_key = nullptr;

  float escape_force = 1000.0f;

  // How much a direction's alignment with the fight is worth relative to how open it is, expressed
  // as a fraction of `opening_distance`. A fully target-facing direction is credited this much extra
  // clearance, so it wins ties and near-ties but never beats a genuinely blocked-versus-open call.
  float toward_bias = 0.35f;

 private:
  Vector2f FindEscapeDirection(Game& game, const Player& self, float radius, const Vector2f& toward,
                               bool has_toward) const {
    constexpr size_t kSampleCount = 16;
    constexpr float kTwoPi = 6.28318f;

    Vector2f best_direction = self.GetHeading();
    float best_score = -1.0f;

    for (size_t i = 0; i < kSampleCount; ++i) {
      float angle = (kTwoPi / kSampleCount) * (float)i;
      Vector2f direction = Rotate(Vector2f(1, 0), angle);

      CastResult result =
          game.GetMap().Cast(self.position + direction * radius, direction, opening_distance, self.frequency);

      float clearance = result.hit ? result.distance : opening_distance;
      float score = clearance;

      if (has_toward) {
        // Dot is -1..1; map to 0..1 so facing away is merely unrewarded rather than penalised into
        // being unselectable when it is the only open direction left.
        float alignment = (direction.Dot(toward) + 1.0f) * 0.5f;
        score += alignment * toward_bias * opening_distance;
      }

      if (score > best_score) {
        best_score = score;
        best_direction = direction;
      }
    }

    return best_direction;
  }
};

}  // namespace teamversus
}  // namespace zero
