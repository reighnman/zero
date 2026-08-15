#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/zones/teamversus/nodes/CombatMath.h>

namespace zero {
namespace teamversus {

// Decides when to drop a decoy - a temporary mirrored copy of ourselves that enemies have to sort
// out from the real thing.
//
// The measured usage pattern is the opposite of what a "defensive item" naming suggests, and it is
// worth stating because it is easy to get backwards:
//
//     energy at time of use : median 60%
//     distance to nearest enemy : p25 26t   median 37t   p75 48t
//
// Compare to repels, which get spent at 30% energy and 13 tiles. Decoys are used *far away and at
// healthy energy* - they are a misdirection played before the fight closes, not a panic response
// once it has. That makes sense mechanically: a decoy mirrors position and heading, so it is
// convincing at the range where a defender is picking a target off the radar and useless at the
// range where they can see which one is shooting.
//
// The other gate is that a decoy is only worth anything if somebody is in a position to be fooled.
// Dropping one with no enemy looking in our direction just spends an item.
struct DecoyDeceptionNode : public behavior::BehaviorNode {
  DecoyDeceptionNode() {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;

    if (game.ship_controller.ship.decoys == 0) return behavior::ExecuteResult::Failure;

    // Decoys share the bomb cooldown.
    if (TICK_GT(game.ship_controller.ship.next_bomb_tick, GetCurrentTick())) {
      return behavior::ExecuteResult::Failure;
    }

    if (TICK_DIFF(GetCurrentTick(), last_use_tick) < (s32)cooldown_ticks && last_use_tick != 0) {
      return behavior::ExecuteResult::Failure;
    }

    float energy_percent = GetSelfEnergyPercent(game, *self);
    if (energy_percent < min_energy_percent) return behavior::ExecuteResult::Failure;

    float target_distance = ctx.blackboard.ValueOr<float>("target_distance", 0.0f);
    if (target_distance < min_distance) return behavior::ExecuteResult::Failure;

    // Only worth it if someone could actually be deceived - an enemy who is roughly facing us and
    // far enough out that a mirrored ship is genuinely ambiguous.
    if (!HasDeceivableEnemy(game, *self)) return behavior::ExecuteResult::Failure;

    last_use_tick = GetCurrentTick();

    return behavior::ExecuteResult::Success;
  }

  // Matches the measured median energy at use. Below this the item is better saved, since a decoy
  // does not stop damage and we are about to need something that does.
  float min_energy_percent = 0.55f;

  // A little inside the measured p25 of 26 tiles. Closer than this and a decoy is transparent.
  float min_distance = 24.0f;

  // Decoys are limited and the effect lasts a while; there is no value in stacking them.
  u32 cooldown_ticks = 900;

  // How far off an enemy's nose we still count them as looking at us, as a dot product.
  float facing_threshold = 0.5f;

 private:
  Tick last_use_tick = 0;

  bool HasDeceivableEnemy(Game& game, const Player& self) const {
    auto& pm = game.player_manager;

    for (size_t i = 0; i < pm.player_count; ++i) {
      Player* enemy = pm.players + i;

      if (!IsLiveEnemy(game, self, *enemy)) continue;

      Vector2f to_self = self.position - enemy->position;
      float distance = to_self.Length();

      if (distance < min_distance) continue;
      if (distance <= 0.0f) continue;

      if ((to_self * (1.0f / distance)).Dot(enemy->GetHeading()) >= facing_threshold) return true;
    }

    return false;
  }
};

}  // namespace teamversus
}  // namespace zero
