#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/WeaponManager.h>

namespace zero {
namespace nexus {

// Succeeds when we may legally and safely lay another mine. The tree supplies the *tactical*
// conditions (that we're running, at speed, with a pursuer at the right distance); this node only
// answers "is a mine available and would laying it here hurt our own side".
//
// A mine is just a bomb with the alternate bit set - ShipController fires it from
// InputAction::Mine, dropping it in place with no velocity and charging LandmineFireEnergy rather
// than the usual bomb cost. Note that InputAction::Mine takes precedence over InputAction::Bomb in
// the same tick, so the two must never be pressed together.
//
// The cap is per *simultaneously active* mine, MaxMines in the ship settings (1 in this zone), so
// this counts our own live mines in the weapon manager rather than tracking a fired count.
//
// How the human actually uses this, measured over two bot-vs-human replays - he laid exactly one
// mine per match, both times in an identical situation:
//
//     energy 5-12%,  pursuer ~10 tiles back,  own speed ~20 t/s (near max),
//     radial velocity +19 t/s, i.e. essentially all of that speed pointed straight away
//
// So it is a flat-out escape tool: dropped while running for your life from someone who is close
// but not yet on top of you, to make them break off the chase. Note what is *not* required - a
// healthy energy buffer. Safety comes from the separation speed carrying us clear of our own blast
// long before the pursuer reaches it, which is why this only checks that we can pay the fire cost
// rather than demanding we could also survive the explosion.
struct MineAvailableNode : public behavior::BehaviorNode {
  MineAvailableNode() {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    Game& game = *ctx.bot->game;
    auto& settings = game.connection.settings;
    auto& ship_settings = settings.ShipSettings[self->ship];
    auto& ship = game.ship_controller.ship;

    // Mines come out of the bomb slot.
    if (ship.bombs == 0) return behavior::ExecuteResult::Failure;

    u32 level = ship.bombs - 1;

    // Can we pay for it? ShipController refuses the shot outright if not.
    float fire_cost = (float)(ship_settings.LandmineFireEnergy + ship_settings.LandmineFireEnergyUpgrade * level);
    if (self->energy <= fire_cost) return behavior::ExecuteResult::Failure;

    // Are we already at the simultaneous-mine cap?
    u32 active = 0;
    for (size_t i = 0; i < game.weapon_manager.weapon_count; ++i) {
      Weapon& weapon = game.weapon_manager.weapons[i];

      if (weapon.player_id != self->id) continue;
      if (!weapon.data.alternate) continue;
      if (weapon.data.type != WeaponType::Bomb && weapon.data.type != WeaponType::ProximityBomb) continue;

      ++active;
    }

    if (active >= ship_settings.MaxMines) return behavior::ExecuteResult::Failure;

    // Don't leave one sitting on top of a teammate who is following us out.
    float explode_pixels = (float)(settings.BombExplodePixels + settings.BombExplodePixels * level);
    float blast_tiles = explode_pixels / 16.0f;

    for (size_t i = 0; i < game.player_manager.player_count; ++i) {
      Player* player = game.player_manager.players + i;

      if (player->id == self->id) continue;
      if (player->ship >= 8) continue;
      if (player->frequency != self->frequency) continue;
      if (player->IsRespawning()) continue;
      if (player->position == Vector2f(0, 0)) continue;

      if (player->position.DistanceSq(self->position) <= blast_tiles * blast_tiles) {
        return behavior::ExecuteResult::Failure;
      }
    }

    return behavior::ExecuteResult::Success;
  }
};

}  // namespace nexus
}  // namespace zero
