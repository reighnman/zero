#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Holds cruising speed below the ship's top speed by adding a braking force once we're over the
// cap, so the bot keeps acceleration in reserve instead of committing everything to closing.
//
// The tactical shape this serves: teams mostly hold station and trade shots until somebody fails a
// dodge, and only then does anyone commit. A ship already at maximum speed has nothing left to
// accelerate with when it needs to dodge, and carries momentum it cannot cheaply reverse, so
// approaching flat-out is the wrong default. Full speed is for actually pressing a target, and for
// running away - both of which the tree exempts.
//
// This is a *soft* cap, deliberately, because it has to work without touching core steering.
// Steering::Seek asks for `GetMaxSpeed` and contributes `desired_velocity - current_velocity` to
// the shared force accumulator; there is no per-tick speed limit to set, and adding one would mean
// editing Steering.h, which is core and shared with every other zone. Instead this adds an opposing
// force proportional to how far over the cap we are, and the two balance out:
//
//     seek forward term ~ (max_speed - speed),  braking term = (speed - cap) * gain
//     equilibrium at     speed = (max_speed + gain*cap) / (1 + gain)
//
// With gain 4 and a cap at 0.8 * max, that settles around 0.84 * max - close enough to the intent,
// and it degrades gracefully rather than fighting whatever else is pushing that tick. Returns
// Failure when already under the cap so it never gates anything in a Selector.
struct CruiseSpeedNode : public behavior::BehaviorNode {
  CruiseSpeedNode(float speed_percent, float gain = 4.0f) : speed_percent(speed_percent), gain(gain) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    // Same conversion Steering::GetMaxSpeed uses for its main term: raw speed settings are in
    // 1/10 pixel per tick.
    float max_speed = ctx.bot->game->ship_controller.ship.speed / 10.0f / 16.0f;
    if (max_speed <= 0.0f) return behavior::ExecuteResult::Failure;

    float cap = max_speed * speed_percent;
    float speed = self->velocity.Length();

    if (speed <= cap) return behavior::ExecuteResult::Failure;

    ctx.bot->bot_controller->steering.force += Normalize(self->velocity) * -((speed - cap) * gain);

    return behavior::ExecuteResult::Success;
  }

  float speed_percent = 0.8f;
  float gain = 4.0f;
};

}  // namespace nexus
}  // namespace zero
