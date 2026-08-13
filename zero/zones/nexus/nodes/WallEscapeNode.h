#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/zones/nexus/nodes/EnclosureQuery.h>

namespace zero {
namespace nexus {

// Takes over movement only when we are genuinely getting boxed in, and steers out along the widest
// remaining opening.
//
// This deliberately replaces the blanket "a wall is within N tiles, override everything" rule that
// WallAvoidanceNode applies. That rule has two failure modes pulling in opposite directions: it
// fires constantly in tight map geometry, overriding legitimate combat movement so the bot refuses
// to fight anywhere near an obstacle, and it still doesn't see a corner coming, because a corner
// and a flat wall look identical to a nearest-wall-distance check. Walls are not inherently
// dangerous in this game - contact just bounces you - so the thing worth reacting to is the loss of
// escape options, not the presence of terrain.
//
// Triggers when either measure of enclosure goes bad:
//   - `min_open_fraction`: too few directions remain clear at all.
//   - `min_escape_arc`: the clear directions that do remain are a single narrow corridor, which is
//     what a corner or dead end looks like even when a decent number of rays still get out.
//
// Returns Failure when there's still room to maneuver, so the tree falls through and the bot keeps
// fighting normally near obstacles instead of being shoved into open ground.
struct WallEscapeNode : public behavior::BehaviorNode {
  WallEscapeNode(float probe_distance, float min_open_fraction, float min_escape_arc)
      : probe_distance(probe_distance), min_open_fraction(min_open_fraction), min_escape_arc(min_escape_arc) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;
    float radius = game.connection.settings.ShipSettings[self->ship].GetRadius();

    EnclosureReport report = GetEnclosure(game, self->position, self->frequency, radius, probe_distance);

    bool boxed_in = report.open_fraction < min_open_fraction || report.escape_arc < min_escape_arc;
    if (!boxed_in) return behavior::ExecuteResult::Failure;

    auto& steering = ctx.bot->bot_controller->steering;

    steering.Face(game, self->position + report.escape_direction);
    // Deliberately not Seek here - Seek corrects against current velocity, which fights the escape
    // when momentum is already carrying us into the pocket we're trying to leave.
    steering.force += report.escape_direction * kEscapeForce;

    return behavior::ExecuteResult::Success;
  }

  float probe_distance = 12.0f;
  float min_open_fraction = 0.35f;
  float min_escape_arc = 1.2f;  // radians, ~70 degrees

 private:
  static constexpr float kEscapeForce = 1000.0f;
};

}  // namespace nexus
}  // namespace zero
