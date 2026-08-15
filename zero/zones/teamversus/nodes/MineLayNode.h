#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/zones/teamversus/nodes/CombatMath.h>

namespace zero {
namespace teamversus {

// Decides when to drop a mine in the path of someone chasing us.
//
// A mine is a bomb that sits still for two minutes. That makes it worth almost nothing thrown at a
// moving fight and worth a great deal in one specific situation: we are backing off, an enemy is
// following us, and the ground between us is ground they have to cross. The blast radius is around
// ten tiles, so it does not need to be stepped on - it needs to be somewhere they will pass near.
//
// The conditions are narrow on purpose. There is one mine slot per player and twelve per team, a
// mine costs the full landmine energy, and a badly placed one is a hazard our own side has to
// navigate around for the rest of the round. So this requires all of:
//
//  - We are actually withdrawing, not just momentarily pointed away.
//  - Someone is genuinely pursuing - closing, close enough to matter, and behind us.
//  - No teammate is anywhere near where it would be laid, because a mine cannot tell friend from
//    foe on detonation any more than a bomb's blast can.
//  - Enough energy that laying it does not leave us unable to survive the chase we're in.
//
// The pursuit test uses closing speed rather than distance alone. An enemy sitting at the same
// range is not chasing us, and mining for them accomplishes nothing except giving away the item.
struct MineLayNode : public behavior::BehaviorNode {
  MineLayNode() {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto& game = *ctx.bot->game;
    auto& ship = game.ship_controller.ship;
    auto& settings = game.connection.settings;

    if (ship.bombs == 0) return behavior::ExecuteResult::Failure;
    if (TICK_GT(ship.next_bomb_tick, GetCurrentTick())) return behavior::ExecuteResult::Failure;

    // Mine slots: ours and the team's. Also refuses if we are sitting on an existing mine, which
    // would waste the drop.
    size_t self_max_mines = settings.ShipSettings[self->ship].MaxMines;
    size_t team_max_mines = settings.TeamMaxMines;

    size_t self_mine_count = 0;
    size_t team_mine_count = 0;
    bool on_existing_mine = false;

    game.weapon_manager.GetMineCounts(*self, self->position, &self_mine_count, &team_mine_count, &on_existing_mine);

    if (on_existing_mine) return behavior::ExecuteResult::Failure;
    if (self_mine_count >= self_max_mines) return behavior::ExecuteResult::Failure;
    if (team_mine_count >= team_max_mines) return behavior::ExecuteResult::Failure;

    // Energy: the mine costs its own fire energy, and we need to still be alive afterward.
    float mine_energy = (float)settings.ShipSettings[self->ship].LandmineFireEnergy;
    if (self->energy - mine_energy < (float)ship.energy * min_energy_after) {
      return behavior::ExecuteResult::Failure;
    }

    if (!IsWithdrawing(ctx, *self)) return behavior::ExecuteResult::Failure;

    // A mine detonates on blast just like a bomb, and does not care whose side stepped on it.
    float blast_radius = GetBlastRadius(game, ship.bombs > 0 ? (u16)(ship.bombs - 1) : (u16)0);
    float clearance = blast_radius * team_clearance_multiplier;

    if (!HasPursuer(game, *self, blast_radius)) return behavior::ExecuteResult::Failure;

    auto& pm = game.player_manager;
    for (size_t i = 0; i < pm.player_count; ++i) {
      Player* mate = pm.players + i;

      if (!IsLiveTeammate(game, *self, *mate)) continue;

      if (mate->position.DistanceSq(self->position) < clearance * clearance) {
        return behavior::ExecuteResult::Failure;
      }
    }

    return behavior::ExecuteResult::Success;
  }

  // Don't drop below this fraction of max energy after paying for the mine.
  //
  // This was 0.3 and it made the node unreachable, which is why seven bots laid zero mines across a
  // whole match while every human in the corpus lays one or two per game. A landmine costs 500 of a
  // 1700 tank - 29% - so a floor of 0.3 after payment demands 59% *before*, while the posture gate
  // just above demands Recover or Regroup, and Recover is entered below 45% energy. The two
  // conditions were very nearly disjoint: the only way to satisfy both was the rare Regroup or the
  // outnumbered-at-full-health path.
  //
  // Two gates that cannot both be true is a silent feature deletion, and it does not announce
  // itself - the branch simply never fires and nothing logs. Worth checking for wherever an energy
  // floor sits underneath a posture that is itself defined by energy.
  //
  // 0.12 puts the requirement at ~41% before the drop, which brackets where humans actually lay
  // them: median energy 33-55%, at 15-21 tiles from the nearest enemy.
  float min_energy_after = 0.12f;

  // How much of the blast radius to keep clear of teammates. Above 1.0 because a teammate walking
  // toward the spot is as much of a problem as one already standing on it.
  float team_clearance_multiplier = 1.5f;

  // How far past the blast radius our own momentum has to carry us before the mine goes off. Above
  // 1.0 for the same reason: the pursuer's closing speed is an estimate and arriving exactly at the
  // rim of the blast is not clear of it.
  float self_clearance_multiplier = 1.3f;

  // A pursuer has to be closing at least this fast, in tiles/sec, to count - and this is the whole
  // point of the item rather than a detail. A mine is visible and stationary, so a chaser with time
  // and room simply steers around it and we have spent the drop for nothing. Someone committed at
  // speed is the opposite case: momentum is what makes a Subspace ship hard to redirect, thrust only
  // acts along the heading, and at better than ten tiles a second they cannot kill and rebuild that
  // much velocity inside the couple of seconds it takes them to arrive. The faster they are chasing,
  // the less able they are to avoid what we leave behind.
  float min_closing_speed = 10.0f;
  // ...and be close enough that they reach it before they have thought about it. Further out and the
  // mine is just a landmark they route around.
  float max_pursuer_distance = 25.0f;
  // ...and be behind us rather than in front, as a dot product against our travel direction.
  float behind_threshold = -0.2f;

 private:
  static bool IsWithdrawing(behavior::ExecuteContext& ctx, const Player& self) {
    // Posture is the honest signal here. Radial sign alone flips constantly during a normal orbit
    // and reads as retreating about 40% of the time, which would scatter mines through the middle
    // of every fight.
    return ctx.blackboard.Has("phase_recover") || ctx.blackboard.Has("phase_regroup");
  }

  // A pursuer worth mining for, *and* one far enough back that our own momentum carries us clear of
  // the blast before they reach it.
  //
  // That second half is the part that was missing, and it is why bots were laying mines with an
  // enemy six to twelve tiles away against a ten tile blast radius - hurting themselves and any
  // teammate nearby. The mine sits still at the point we drop it; the only thing that gets us out of
  // its blast is our own velocity, over however long the pursuer takes to arrive. So the check is a
  // race, and both sides of it have to be measured: how long until they trip it, and how far we will
  // have travelled by then.
  //
  // It falls out sensibly at both ends. At six tiles with someone closing at twelve tiles/sec there
  // is half a second before it goes off and we cover six tiles - inside the blast, so no mine. At
  // twenty tiles that is over a second and a half, we cover twenty, and the mine is free.
  bool HasPursuer(Game& game, const Player& self, float blast_radius) const {
    Vector2f travel = self.velocity;
    float own_speed = travel.Length();
    if (own_speed < 1.0f) return false;
    travel = travel * (1.0f / own_speed);

    auto& pm = game.player_manager;

    for (size_t i = 0; i < pm.player_count; ++i) {
      Player* enemy = pm.players + i;

      if (!IsLiveEnemy(game, self, *enemy)) continue;

      Vector2f to_enemy = enemy->position - self.position;
      float distance = to_enemy.Length();

      if (distance <= 0.0f || distance > max_pursuer_distance) continue;

      Vector2f direction = to_enemy * (1.0f / distance);

      // Behind us relative to where we're going.
      if (direction.Dot(travel) > behind_threshold) continue;

      // Closing: their velocity relative to ours has a component pointed at us.
      float closing_speed = (enemy->velocity - self.velocity).Dot(-direction);
      if (closing_speed < min_closing_speed) continue;

      // How long until they reach the ground we are standing on, at the speed they are covering it.
      float approach_speed = enemy->velocity.Dot(-direction);
      if (approach_speed < 1.0f) continue;

      float seconds_to_trip = distance / approach_speed;

      // Where we will be by then, travelling as we are now. Nothing else moves us clear - the mine
      // does not travel and we cannot outrun our own momentum.
      if (own_speed * seconds_to_trip < blast_radius * self_clearance_multiplier) continue;

      return true;
    }

    return false;
  }
};

}  // namespace teamversus
}  // namespace zero
