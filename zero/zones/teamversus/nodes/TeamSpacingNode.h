#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/zones/teamversus/nodes/CombatMath.h>

namespace zero {
namespace teamversus {

// Holds the team in a workable formation as a steering blend, without ever taking over movement.
//
// Real teams sit at a median 26 tiles apart with a p25 of 15 and a p10 of 8, and that spacing
// barely changes between engaged and disengaged play - it is a stable formation, not an artifact of
// fighting. This node reproduces the band by pushing apart below it and pulling together above it,
// and doing nothing at all inside it.
//
// Both edges of the band earn their place:
//
//  - Too close is a real cost, not just aesthetics. Bomb blast radius is around 10 tiles, so two
//    teammates inside that share every bomb aimed at either of them. Clustering also means one
//    dodge decision moves both ships the same way, which turns a single well-led shot into two hits.
//  - Too far is what gets people killed. Distance from own support was the strongest predictor of
//    who dies next among everything measured, and it held two seconds ahead of the kill rather than
//    only at the moment of it.
//
// This intentionally blends a force rather than pathing anywhere. A GoTo toward a teammate abandons
// aim, abandons the standoff, and reads as a bot that has stopped fighting to go stand next to
// someone - which is not what the spacing data describes. Getting genuinely rescued from isolation
// is the Regroup posture's job; this is the gentle continuous correction that stops it being
// needed in the first place.
struct TeamSpacingNode : public behavior::BehaviorNode {
  TeamSpacingNode(float min_spacing, float max_spacing) : min_spacing(min_spacing), max_spacing(max_spacing) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;
    auto& pm = game.player_manager;

    Vector2f correction;
    float count = 0.0f;

    for (size_t i = 0; i < pm.player_count; ++i) {
      Player* mate = pm.players + i;

      if (!IsLiveTeammate(game, *self, *mate)) continue;

      Vector2f to_mate = mate->position - self->position;
      float distance = to_mate.Length();

      if (distance <= 0.0f) continue;

      Vector2f direction = to_mate * (1.0f / distance);

      if (distance < min_spacing) {
        // Push apart, weighted by how far inside the minimum we are. Squared so the correction
        // gets urgent quickly at genuinely overlapping range rather than being uniformly mild.
        float overlap = (min_spacing - distance) / min_spacing;
        correction -= direction * (overlap * overlap * separation_force);
        count += 1.0f;
      } else if (distance > max_spacing) {
        // Pull together, but only linearly and only from the nearest teammate's direction - this is
        // a lean, not a summons.
        float excess = (distance - max_spacing) / max_spacing;
        if (excess > 1.0f) excess = 1.0f;

        correction += direction * (excess * cohesion_force);
        count += 1.0f;
      }
    }

    if (count <= 0.0f) return behavior::ExecuteResult::Failure;

    ctx.bot->bot_controller->steering.force += correction / count;

    return behavior::ExecuteResult::Success;
  }

  // Roughly the blast radius, so teammates stop sharing bombs.
  float min_spacing = 11.0f;
  // A little over the measured median spacing, so the pull only starts once we are genuinely
  // spreading rather than at every normal fluctuation.
  float max_spacing = 32.0f;

  float separation_force = 14.0f;
  float cohesion_force = 6.0f;
};

}  // namespace teamversus
}  // namespace zero
