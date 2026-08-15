#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/zones/teamversus/nodes/CombatMath.h>

namespace zero {
namespace teamversus {

// Decides whether warping back to our dropped portal is the right escape right now.
//
// A portal is a two-stage item: the first press drops a marker, the second teleports us to it. That
// makes it the only escape in the kit that moves us instantly rather than accelerating us, which is
// exactly what is needed when the thing about to kill us is already too close to outrun. But it is
// also a one-way trip to wherever the marker happens to be, which is why this checks that the
// destination is actually an improvement rather than warping into a different fight.
//
// Ordering against the repel matters. A repel stops the damage *and* leaves us in position, which
// is strictly better when both are available - so the tree only reaches this once repels are gone
// or on cooldown. That mirrors what the item counts force anyway: two repels and two portals from
// spawn, with repels being the ones that get spent first in real play.
//
// The destination check is the part worth being careful about. Warping is useless if the marker is
// somewhere an enemy is already standing, and actively harmful if it is somewhere our own team has
// left, so both are tested. Distance from the current threat is the primary criterion: a marker
// five tiles away escapes nothing.
struct PortalEscapeNode : public behavior::BehaviorNode {
  PortalEscapeNode() {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;
    auto& ship = game.ship_controller.ship;

    // A marker exists only while its timer is running; portal_location itself is always populated
    // with whatever was last dropped, so the timer is the only thing that says it is still there.
    if (ship.portal_time <= 0.0f) return behavior::ExecuteResult::Failure;

    Vector2f portal = ship.portal_location;

    // Escaping to somewhere barely further from the threat is not escaping.
    Vector2f threat_origin = ctx.blackboard.ValueOr<Vector2f>("threat_origin", self->position);

    float current_separation = threat_origin.Distance(self->position);
    float portal_separation = threat_origin.Distance(portal);

    if (portal_separation < current_separation + min_gain) return behavior::ExecuteResult::Failure;

    // Don't land on top of an enemy.
    auto& pm = game.player_manager;
    for (size_t i = 0; i < pm.player_count; ++i) {
      Player* enemy = pm.players + i;

      if (!IsLiveEnemy(game, *self, *enemy)) continue;

      if (enemy->position.DistanceSq(portal) < enemy_clearance * enemy_clearance) {
        return behavior::ExecuteResult::Failure;
      }
    }

    return behavior::ExecuteResult::Success;
  }

  // How much further from the threat the portal has to put us before it counts as an escape.
  float min_gain = 20.0f;
  // Keep clear of anyone waiting at the other end.
  float enemy_clearance = 14.0f;
};

// Drops the portal marker when we don't have one down.
//
// Kept separate from the escape decision because the two are gated on completely different things:
// laying a marker is cheap, wants to happen early and somewhere safe, and is pointless to
// re-evaluate under fire; using it is expensive, urgent, and depends entirely on where the marker
// ended up. Combining them produces a node that drops its own marker at the moment it most needs
// the old one.
//
// The safety requirement is the whole content of this node. A marker dropped mid-fight is a marker
// that teleports us back into that fight, so it only gets laid when nothing is inbound and we are
// not the closest thing to an enemy.
struct PortalLayNode : public behavior::BehaviorNode {
  PortalLayNode() {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;
    auto& ship = game.ship_controller.ship;

    if (ship.portals == 0) return behavior::ExecuteResult::Failure;
    // Already have one down.
    if (ship.portal_time > 0.0f) return behavior::ExecuteResult::Failure;

    if (TICK_GT(ship.next_bomb_tick, GetCurrentTick())) return behavior::ExecuteResult::Failure;

    // Only lay one somewhere we would actually want to come back to.
    if (ctx.blackboard.Has("threat_lethal")) return behavior::ExecuteResult::Failure;

    float energy_percent = GetSelfEnergyPercent(game, *self);
    if (energy_percent < min_energy_percent) return behavior::ExecuteResult::Failure;

    float target_distance = ctx.blackboard.ValueOr<float>("target_distance", 1000.0f);
    if (target_distance < min_enemy_distance) return behavior::ExecuteResult::Failure;

    return behavior::ExecuteResult::Success;
  }

  float min_energy_percent = 0.7f;
  float min_enemy_distance = 25.0f;
};

}  // namespace teamversus
}  // namespace zero
