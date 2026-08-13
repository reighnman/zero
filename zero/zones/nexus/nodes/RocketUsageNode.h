#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// The two timing primitives for spending a rocket.
//
// A rocket is a short burst of extra thrust and a raised speed cap (ShipController swaps in
// RocketThrust/RocketSpeed while ship.rocket_end_tick is in the future). It is a limited item, so
// the question is never "would this help" but "is this the moment where it helps most".
//
// It helps most when we are *already* at or near our normal top speed: the boost then buys real
// closing or breaking distance. Fired from a standstill or mid-turn, most of the burn is spent just
// getting back up to the speed we could have reached anyway, which is the waste to avoid - and when
// chasing, it also risks overshooting straight past the target.

// Succeeds when our current speed is at least `speed_percent` of what this ship can normally do.
// Normal top speed is ship.speed (the player's own upgraded rating), not ShipSettings::MaximumSpeed,
// which is the afterburner/rocket ceiling - see ShipController's `afterburners ? MaximumSpeed :
// ship.speed`. Raw speed settings are in units of 1/10 pixel per tick, so /10/16 converts to
// tiles/sec to match Player::velocity.
struct AtMaxSpeedNode : public behavior::BehaviorNode {
  AtMaxSpeedNode(float speed_percent) : speed_percent(speed_percent) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    float max_speed = ctx.bot->game->ship_controller.ship.speed / 10.0f / 16.0f;
    if (max_speed <= 0.0f) return behavior::ExecuteResult::Failure;

    float speed = self->velocity.Length();

    return speed >= max_speed * speed_percent ? behavior::ExecuteResult::Success
                                              : behavior::ExecuteResult::Failure;
  }

  float speed_percent = 0.0f;
};

// Succeeds while a rocket burn is still running, so the tree can invert it and avoid stacking a
// second rocket onto one that is already doing its job.
struct RocketActiveQueryNode : public behavior::BehaviorNode {
  RocketActiveQueryNode() {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    u32 rocket_end_tick = ctx.bot->game->ship_controller.ship.rocket_end_tick;

    return TICK_GT(rocket_end_tick, GetCurrentTick()) ? behavior::ExecuteResult::Success
                                                      : behavior::ExecuteResult::Failure;
  }
};

}  // namespace nexus
}  // namespace zero
