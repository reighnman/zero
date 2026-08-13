#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>

namespace zero {
namespace nexus {

// Returns whichever perpendicular-to-target heading is closer to our current facing, so turning to
// face it costs the smaller rotation. Either direction along that axis is equally useful for a
// forward/backward dodge, since Actuator already picks whichever of forward/backward thrust
// requires less rotation to reach.
inline Vector2f GetBroadsideDirection(const Player& self, const Vector2f& target_position) {
  Vector2f away_direction = Normalize(self.position - target_position);
  Vector2f broadside = Perpendicular(away_direction);

  if (broadside.Dot(self.GetHeading()) < 0.0f) {
    broadside = -broadside;
  }

  return broadside;
}

// Faces broadside to a target instead of directly at it. FleeNode uses the same idea while holding
// at leash range - ships only thrust forward/backward along their current heading, so a dodge is
// fastest when that heading is already perpendicular to the threat, instead of costing a rotation
// after the fact. This is the same stance for combat idle windows (e.g. between burst-fire volleys
// while orbiting) instead of only during a defensive retreat.
struct BroadsideFaceNode : public behavior::BehaviorNode {
  BroadsideFaceNode(const char* position_key) : position_key(position_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_position = ctx.blackboard.Value<Vector2f>(position_key);
    if (!opt_position.has_value()) return behavior::ExecuteResult::Failure;

    Vector2f target_position = *opt_position;
    Vector2f broadside_direction = GetBroadsideDirection(*self, target_position);

    ctx.bot->bot_controller->steering.Face(*ctx.bot->game, self->position + broadside_direction);

    return behavior::ExecuteResult::Success;
  }

  const char* position_key = nullptr;
};

}  // namespace nexus
}  // namespace zero
