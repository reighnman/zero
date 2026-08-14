#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Succeeds when the given enemy is behind us relative to our own travel *and* actively closing -
// i.e. genuinely chasing us down, rather than merely being nearby.
//
// This is the condition a mine actually needs. A mine is dropped in place with no velocity, so it
// only ever threatens someone who drives into it, which means it only works in our wake. Dropping
// one while moving toward an enemy leaves it behind us where nobody is going, and simply spends the
// one mine we're allowed to have active.
//
// That is exactly what the bots were doing. Measured over rec11, radial velocity at the moment each
// mine was laid (negative = closing on the enemy, positive = opening the range):
//
//     Aither -23.0    rajani -20.2    boann -18.4    Damayanti -16.7
//     Gwythyr  +7.2   Vesna   +2.7    Lalita +20.6
//
// Four of seven bots laid mines while charging *at* the enemy at 17-23 tiles/sec. For comparison
// the human's two mines across earlier replays were at +19.2 and +19.5, i.e. flat-out retreat.
// Being at speed and inside a distance band, which is all the old gate checked, does not
// distinguish running away from running in.
//
// The closing check matters as much as the direction: a pursuer travelling fast toward us is the
// one who cannot avoid a mine appearing in their path, while someone drifting after us at walking
// pace will simply steer around it.
struct PursuedFromBehindNode : public behavior::BehaviorNode {
  PursuedFromBehindNode(const char* target_player_key, float rear_cone_degrees, float min_closing_speed)
      : target_player_key(target_player_key),
        rear_cone_degrees(rear_cone_degrees),
        min_closing_speed(min_closing_speed) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_target = ctx.blackboard.Value<Player*>(target_player_key);
    if (!opt_target.has_value()) return behavior::ExecuteResult::Failure;

    Player* target = opt_target.value();
    if (!target) return behavior::ExecuteResult::Failure;

    // We have to actually be going somewhere for "behind" to mean anything.
    if (self->velocity.LengthSq() < 1.0f) return behavior::ExecuteResult::Failure;

    Vector2f travel = Normalize(self->velocity);

    Vector2f to_target = target->position - self->position;
    if (to_target.LengthSq() < 1.0f) return behavior::ExecuteResult::Failure;

    to_target = Normalize(to_target);

    // Rear cone: the enemy must lie more than (180 - cone/2) degrees off our direction of travel.
    float cone_radians = rear_cone_degrees * 3.14159265f / 180.0f;
    float max_dot = -cosf(cone_radians * 0.5f);

    if (travel.Dot(to_target) > max_dot) return behavior::ExecuteResult::Failure;

    // And they have to be running us down, not just trailing along.
    Vector2f toward_us = Normalize(self->position - target->position);
    float closing_speed = target->velocity.Dot(toward_us);

    return closing_speed >= min_closing_speed ? behavior::ExecuteResult::Success
                                              : behavior::ExecuteResult::Failure;
  }

  const char* target_player_key = nullptr;
  float rear_cone_degrees = 90.0f;
  float min_closing_speed = 0.0f;
};

}  // namespace nexus
}  // namespace zero
