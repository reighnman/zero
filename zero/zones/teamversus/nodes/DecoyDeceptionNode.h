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
// This is scoped to one situation and one only: breaking away hurt. The tree requires the Recover
// posture on top of everything below, so a decoy is never spent while we are pressing, poking, or
// otherwise winning. The measured human usage points elsewhere (median 60% energy, median 37 tiles
// to the nearest enemy - a misdirect played before the fight closes rather than after), but a decoy
// dropped at healthy energy is a decoy we do not have while disengaging at 20%, and there are only
// two of them.
//
// It is deliberately *not* gated on an inbound lethal volley, which is where it first sat. A decoy
// has no effect whatsoever on a shot already in the air, so tying it to one spends it at the exact
// moment it cannot help. What it buys is the next volley going to the wrong ship, which is worth
// something only if we will be somewhere else by then - i.e. while withdrawing.
//
// What a decoy can and cannot do bounds when it is worth pressing:
//
//  - It mirrors our position and heading, so it is convincing to someone reading the radar or
//    tracking a contact at range, and transparent to someone close enough to watch which of the two
//    ships is firing. Hence the distance band: too close and it fools nobody, too far and there was
//    no urgency to spend it.
//  - It does not stop damage. Anything already in flight still arrives. What it buys is the *next*
//    volley being aimed at the wrong ship, which is worth something only if somebody is currently
//    aiming at us at all - hence the facing check. Dropped with nobody looking our way, it is simply
//    an item deleted.
//
// The press-rate cooldown lives in the tree (ItemCooldownNode) rather than here, so that every item
// shares one debounce mechanism and the timer is claimed by the branch that actually presses the key
// rather than by whichever node happened to evaluate the decision.
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

    // Low health, checked here as well as through the posture gate in the tree. Recover is entered
    // on a two-man local disadvantage as well as on energy, so the posture alone does not actually
    // mean "hurt" and would let a healthy bot burn decoys every time it was momentarily outnumbered.
    float energy_percent = GetSelfEnergyPercent(game, *self);
    if (energy_percent > max_energy_percent) return behavior::ExecuteResult::Failure;

    // Somebody has to be in a position to be fooled, at a range where the copy is ambiguous.
    if (!HasDeceivableEnemy(game, *self)) return behavior::ExecuteResult::Failure;

    return behavior::ExecuteResult::Success;
  }

  // Energy at or below which a decoy is worth spending. Sits at the band where we are losing the
  // exchange and need the next volley to go somewhere else.
  float max_energy_percent = 0.45f;

  // Closer than this an enemy can simply see which ship is real; further than this we had other
  // options and did not need to spend the item.
  float min_distance = 12.0f;
  float max_distance = 45.0f;

  // How far off an enemy's nose we still count them as looking at us, as a dot product.
  float facing_threshold = 0.5f;

 private:
  bool HasDeceivableEnemy(Game& game, const Player& self) const {
    auto& pm = game.player_manager;

    for (size_t i = 0; i < pm.player_count; ++i) {
      Player* enemy = pm.players + i;

      if (!IsLiveEnemy(game, self, *enemy)) continue;

      Vector2f to_self = self.position - enemy->position;
      float distance = to_self.Length();

      if (distance < min_distance || distance > max_distance) continue;

      if ((to_self * (1.0f / distance)).Dot(enemy->GetHeading()) >= facing_threshold) return true;
    }

    return false;
  }
};

}  // namespace teamversus
}  // namespace zero
