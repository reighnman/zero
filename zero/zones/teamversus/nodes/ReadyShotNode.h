#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>
#include <zero/zones/teamversus/nodes/CombatMath.h>

namespace zero {
namespace teamversus {

// Fires the single bullet that signals "ready" during the pre-match check.
//
// The match protocol is: the zone pulls us out of spec into a ship, every player fires one shot to
// confirm they are at their keyboard, and once everyone has the zone announces "GO!". So this shot
// is not optional and it is not cosmetic - it is a precondition for the match starting at all. If it
// never goes out, "GO!" never arrives, and the bot sits inert until the lifecycle safety net gives
// up and declares the match live on its own, which means it starts fighting a match that never
// actually began.
//
// That makes "did the shot actually leave the ship" the entire problem, and it is not the same
// question as "did we press the fire key". Pressing it is free and always succeeds; the shot can
// still fail to materialise for at least four reasons:
//
//   - We are still inside the spawn delay. ShipController::Update returns immediately while
//     enter_delay > 0, so the input is read and discarded.
//   - The bullet is on cooldown from a previous attempt.
//   - Energy is below the bullet's fire cost.
//   - The ship has no guns, or we are sitting on a safe tile.
//
// A node that pressed once and assumed success would silently hang the entire match on any of those.
// So this presses, then *confirms* on a later tick by watching ship.next_bullet_tick advance - which
// ShipController only does when a bullet genuinely fires - and re-attempts if it doesn't.
//
// It also re-signals periodically while still waiting for "GO!". If the zone missed our shot, or the
// ready check was reset, one more shot costs 1.2% of our energy tank and is exactly what a human
// would do when the match fails to start. It stops as soon as the match goes live.
//
// Like the other decision nodes here, this only decides; the tree presses the key. That keeps the
// confirmation logic in one place and out of the input plumbing.
struct ReadyShotNode : public behavior::BehaviorNode {
  ReadyShotNode() {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;
    auto& ship = game.ship_controller.ship;

    // Back in spec - the match we readied for is over. Re-arm so the next one gets its own signal.
    if (self->ship >= 8) {
      Reset();
      return behavior::ExecuteResult::Failure;
    }

    // The signal worked (or the match started without needing it). Nothing more to do.
    if (ctx.blackboard.Has("match_live")) return behavior::ExecuteResult::Failure;

    Tick now = GetCurrentTick();

    // Resolve any outstanding attempt before considering a new one.
    if (press_tick != 0) {
      if (ship.next_bullet_tick != pre_press_next_bullet_tick) {
        // next_bullet_tick only moves when a bullet actually leaves the ship, so this is proof the
        // shot went out rather than an assumption that the keypress was honoured.
        Log(LogLevel::Info, "TeamVersus: ready shot fired.");

        press_tick = 0;
        last_confirmed_tick = now == 0 ? 1 : now;

        return behavior::ExecuteResult::Failure;
      }

      if (TICK_DIFF(now, press_tick) < (s32)confirm_window_ticks) {
        // Still waiting to see whether it took.
        return behavior::ExecuteResult::Failure;
      }

      // The press was swallowed - spawn delay, safe tile, no energy. Fall through and try again.
      press_tick = 0;
    }

    // Already signalled recently and still waiting on the zone. Don't machine-gun the ready check.
    if (last_confirmed_tick != 0 && TICK_DIFF(now, last_confirmed_tick) < (s32)resignal_interval_ticks) {
      return behavior::ExecuteResult::Failure;
    }

    // Nothing fires during the spawn delay - the whole ship update is skipped - so pressing here
    // would burn an attempt and a confirmation window for nothing.
    if (self->enter_delay > 0.0f) return behavior::ExecuteResult::Failure;

    if (ship.guns == 0) return behavior::ExecuteResult::Failure;
    if (TICK_GT(ship.next_bullet_tick, now)) return behavior::ExecuteResult::Failure;

    float fire_energy = (float)game.connection.settings.ShipSettings[self->ship].BulletFireEnergy * (float)ship.guns;
    if (self->energy <= fire_energy) return behavior::ExecuteResult::Failure;

    // Record what we're about to change, so the next tick can tell whether it changed.
    pre_press_next_bullet_tick = ship.next_bullet_tick;
    press_tick = now == 0 ? 1 : now;

    return behavior::ExecuteResult::Success;
  }

  // How long to wait for a press to show up as a fired bullet before assuming it was swallowed.
  // Comfortably longer than a round trip through the ship update.
  u32 confirm_window_ticks = 25;

  // How long to wait for "GO!" after a confirmed shot before signalling again, in case the zone
  // never saw it. Long enough not to look like spam, short enough to rescue a stalled ready check
  // before the lifecycle safety net expires.
  u32 resignal_interval_ticks = 800;

 private:
  Tick press_tick = 0;
  Tick last_confirmed_tick = 0;
  u32 pre_press_next_bullet_tick = 0;

  void Reset() {
    press_tick = 0;
    last_confirmed_tick = 0;
    pre_press_next_bullet_tick = 0;
  }
};

}  // namespace teamversus
}  // namespace zero
