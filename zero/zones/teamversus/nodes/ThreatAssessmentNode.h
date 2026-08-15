#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/WeaponManager.h>
#include <zero/zones/teamversus/nodes/CombatMath.h>

namespace zero {
namespace teamversus {

// Reported as time_to_impact when nothing is inbound. Finite so downstream threshold comparisons
// stay well defined.
inline constexpr float kNoThreatTime = 999.0f;

// Bullets are tested against our hull rather than a blast radius, but positions are sampled at
// roughly 10Hz and our own velocity is changing, so a shot projected to clip the edge is worth
// treating as a hit. This widens the hull a little for that intersection test only.
inline constexpr float kBulletHitboxSlop = 1.6f;

// How close a bomb has to pass before it goes off on us, in tiles, mirroring WeaponManager exactly.
//
// This is a different and much smaller number than the blast radius, and confusing the two is the
// single biggest error a threat model can make here. The blast reaches ten tiles; the fuse trips at
// under four. A bomb whose closest approach to us is six tiles does not do a reduced six-tile-worth
// of damage - it does *nothing at all*, because it never detonates. It sails past and goes off
// somewhere else entirely.
//
// A plain (non-proximity) bomb has no sensor and only detonates on contact, which the same formula
// gives as a quarter of a tile.
inline float GetWeaponFuseRadius(Game& game, const Weapon& weapon) {
  bool is_prox = weapon.data.type == WeaponType::ProximityBomb || weapon.data.type == WeaponType::Thor;

  float radius_pixels = 18.0f;

  if (is_prox) {
    float prox = (float)game.connection.settings.ProximityDistance + (float)weapon.data.level;
    // Thors carry a wider sensor than their level alone implies.
    if (weapon.data.type == WeaponType::Thor) prox += 3.0f;

    radius_pixels = prox * 18.0f;
  }

  radius_pixels -= 14.0f;
  if (radius_pixels < 0.0f) radius_pixels = 0.0f;

  return radius_pixels / 16.0f;
}

// Everything the defensive half of the tree needs to know about what is currently flying at us.
struct ThreatReport {
  // Total damage we expect to actually take if we hold our current course, in raw energy.
  float damage = 0.0f;
  // Damage still landing after the dodge we get *for free* - thrusting along the heading we already
  // hold, with no rotation and no aim given up. Deliberately the pessimistic of the two estimates,
  // because this is what the repel and portal decisions run on and a safety margin computed from a
  // best case is not a margin. The optimistic version below assumes a turn that the movement system
  // only actually performs sometimes, and never while pressing.
  float unavoidable_damage = 0.0f;
  // Damage that a full committed break - turn onto the escape axis, then thrust - would remove. This
  // is the number for deciding whether such a break is worth its cost in aim, and it is the node
  // that would perform the turn that reads it.
  float avoidable_damage = 0.0f;
  // Unit vector pointing away from the threat axis - the direction that most increases our miss
  // distance. Zero when there is no threat.
  Vector2f escape_direction;
  // Seconds until the soonest threat reaches us. Large when nothing is inbound.
  float time_to_impact = 0.0f;
  // Position the soonest threat is coming from, for facing decisions.
  Vector2f origin;
  u32 count = 0;
};

// Scans enemy weapons and estimates how much damage is genuinely on a collision course with us.
//
// The important thing this does differently from a naive scan is bomb damage. WeaponManager's
// GetEstimatedWeaponDamage returns a bomb's *maximum* damage with no distance falloff at all, while
// the real formula falls off linearly from the detonation centre - a bomb at the rim of its blast
// does literally nothing. Feeding the maximum into a dodge or repel decision overstates incoming
// bomb damage several-fold, which produces two expensive failure modes at once: repels spent on
// grazes, and panic-dodging away from bombs that were never going to hurt.
//
// So for bombs this projects where the bomb will be at closest approach and applies the real
// falloff at that distance. For bullets, the estimator is already exact (`ExactDamage` arenas do
// not roll damage down), so it is used as-is.
//
// The escape direction is derived by projecting ourselves onto the threat's ray and pushing away
// from that projection, which is the direction that grows the miss distance fastest. Multiple
// simultaneous threats are combined weighted by damage, so a bullet stream and a bomb arriving from
// different sides produce a sensible compromise rather than whichever was scanned last.
inline ThreatReport AssessThreats(behavior::ExecuteContext& ctx, Player* self, float check_distance) {
  ThreatReport report;

  auto& game = *ctx.bot->game;
  auto& weapon_man = game.weapon_manager;

  float ship_radius = game.connection.settings.ShipSettings[self->ship].GetRadius();
  float check_distance_sq = check_distance * check_distance;

  Vector2f weighted_escape;
  float soonest_impact = kNoThreatTime;

  for (size_t i = 0; i < weapon_man.weapon_count; ++i) {
    Weapon& weapon = weapon_man.weapons[i];

    if (weapon.frequency == self->frequency) continue;
    if (weapon.data.type == WeaponType::Repel || weapon.data.type == WeaponType::Decoy) continue;
    if (weapon.data.type == WeaponType::Burst && !(weapon.flags & WEAPON_FLAG_BURST_ACTIVE)) continue;

    float weapon_distance_sq = weapon.position.DistanceSq(self->position);
    if (weapon_distance_sq > check_distance_sq) continue;

    bool is_bomb = weapon.data.type == WeaponType::Bomb || weapon.data.type == WeaponType::ProximityBomb ||
                   weapon.data.type == WeaponType::Thor;
    bool is_mine = is_bomb && weapon.data.alternate;

    // Work in our own frame, so "will this reach me" accounts for the fact that we're moving too.
    Vector2f relative_velocity = weapon.velocity - self->velocity;
    float relative_speed = relative_velocity.Length();

    // A mine isn't going anywhere, so closest approach is simply where it already sits. Treat it as
    // a threat purely on current separation instead of trying to intersect a zero-length ray.
    float approach_distance = 0.0f;
    float approach_time = 0.0f;

    if (is_mine || relative_speed < 0.01f) {
      approach_distance = sqrtf(weapon_distance_sq);
      approach_time = 0.0f;
    } else {
      Vector2f direction = relative_velocity * (1.0f / relative_speed);
      Vector2f to_self = self->position - weapon.position;

      float along = to_self.Dot(direction);
      // Already gone past us.
      if (along < 0.0f) continue;

      approach_time = along / relative_speed;

      // Ignore anything that will expire before it gets here. A little slop, because alive time and
      // our sampling of it are both approximate.
      constexpr u32 kSlopTicks = 30;
      float remaining_seconds = TICK_DIFF(MAKE_TICK(weapon.end_tick + kSlopTicks), GetCurrentTick()) / 100.0f;
      if (approach_time > remaining_seconds) continue;

      Vector2f closest_point = weapon.position + direction * along;
      approach_distance = closest_point.Distance(self->position);
    }

    // How much further from the shot we could be by the time it gets here. Two answers, because the
    // safety decisions and the movement decision are asking different questions - see
    // GetDodgeDistance. Zero for a mine, which is not travelling toward us and whose fuse trips on
    // contact rather than after a flight.
    float free_dodge = 0.0f;
    float committed_dodge = 0.0f;

    if (!is_mine && relative_speed >= 0.01f) {
      Vector2f threat_direction = relative_velocity * (1.0f / relative_speed);

      free_dodge = GetDodgeDistance(game, *self, threat_direction, approach_time, false);
      committed_dodge = GetDodgeDistance(game, *self, threat_direction, approach_time, true);
    }

    float damage = 0.0f;
    float unavoidable = 0.0f;
    float after_break = 0.0f;

    if (is_bomb) {
      // A bomb goes off at its closest approach to whoever tripped its sensor - WeaponManager holds
      // it until the separation stops shrinking, then detonates and rolls the position back - so
      // closest approach is both whether it detonates and the distance the falloff is evaluated at.
      //
      // The gate is the *fuse* radius, not the blast radius, and that distinction is the whole point
      // of this block. The blast reaches ten tiles but the sensor trips at under four, so a bomb
      // passing six tiles away is not a reduced hit, it is no hit: it never goes off on us at all.
      // Testing against the blast radius instead credited every bomb that sailed past with three to
      // five hundred phantom damage, which inflated every threat total downstream and is exactly the
      // sort of error that produces defensive decisions nobody can explain afterwards.
      //
      // The same cutoff makes dodging a bomb all-or-nothing rather than a linear reduction. Get
      // outside the sensor and it does not detonate; that is worth far more than shading the falloff
      // and is why a bomb aimed dead-on is so much more dangerous than one that merely comes near.
      float fuse_radius = GetWeaponFuseRadius(game, weapon) + ship_radius;

      if (approach_distance > fuse_radius) continue;

      float max_damage = (float)GetEstimatedWeaponDamage(weapon, game.connection);
      u16 level = weapon.data.level;

      damage = max_damage * GetBlastDamageFraction(game, level, approach_distance);

      float free_distance = approach_distance + free_dodge;
      float break_distance = approach_distance + committed_dodge;

      unavoidable = free_distance > fuse_radius ? 0.0f : max_damage * GetBlastDamageFraction(game, level, free_distance);
      after_break = break_distance > fuse_radius ? 0.0f : max_damage * GetBlastDamageFraction(game, level, break_distance);
    } else {
      // Bullets have no fuse, so they only matter if the shot actually crosses our hull.
      float hit_extent = ship_radius * kBulletHitboxSlop;
      if (approach_distance > hit_extent) continue;

      damage = (float)GetEstimatedWeaponDamage(weapon, game.connection);

      // All or nothing: clear the hull and it does nothing at all.
      unavoidable = (approach_distance + free_dodge) > hit_extent ? 0.0f : damage;
      after_break = (approach_distance + committed_dodge) > hit_extent ? 0.0f : damage;
    }

    if (damage <= 0.0f) continue;

    Vector2f escape;

    if (is_mine || relative_speed < 0.01f) {
      escape = Normalize(self->position - weapon.position);
    } else {
      Ray ray(weapon.position, Normalize(relative_velocity));
      Vector2f closest_hit = ray.GetClosestPosition(self->position);
      Vector2f away = self->position - closest_hit;

      // Dead-on shot: we are sitting exactly on the ray, so "away from it" is undefined. Break the
      // tie perpendicular to the shot, which is the shortest path out of its path.
      if (away.LengthSq() < 0.0001f) {
        away = Perpendicular(Normalize(relative_velocity));
      }

      escape = Normalize(away);
    }

    weighted_escape += escape * damage;
    report.damage += damage;
    report.unavoidable_damage += unavoidable;
    report.avoidable_damage += damage - after_break;

    if (approach_time < soonest_impact) {
      soonest_impact = approach_time;
      report.origin = weapon.position;
    }

    ++report.count;
  }

  if (report.count > 0) {
    report.escape_direction = Normalize(weighted_escape);
    report.time_to_impact = soonest_impact;
  } else {
    report.time_to_impact = kNoThreatTime;
  }

  return report;
}

// Runs AssessThreats and publishes the result, so the defensive branches all read one consistent
// snapshot instead of each rescanning the weapon list with slightly different parameters.
//
// Published as separate scalar keys rather than the struct, because the generic comparison nodes
// the tree is built from only understand scalars, and "incoming damage exceeds my current energy"
// wants to be expressible as a plain threshold comparison.
struct ThreatAssessmentNode : public behavior::BehaviorNode {
  ThreatAssessmentNode(float check_distance) : check_distance(check_distance) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    ThreatReport report = AssessThreats(ctx, self, check_distance);

    ctx.blackboard.Set<float>("threat_damage", report.damage);
    ctx.blackboard.Set<float>("threat_unavoidable_damage", report.unavoidable_damage);
    ctx.blackboard.Set<float>("threat_avoidable_damage", report.avoidable_damage);
    ctx.blackboard.Set<float>("threat_count", (float)report.count);
    ctx.blackboard.Set<float>("threat_time", report.time_to_impact);
    ctx.blackboard.Set<Vector2f>("threat_escape", report.escape_direction);
    ctx.blackboard.Set<Vector2f>("threat_origin", report.origin);

    // Existence-only flag for the branches that only care whether this is survivable. Comparing
    // against current energy rather than a fixed number is the whole point: the same bomb is a
    // minor inconvenience at full health and fatal at 20%.
    if (report.count > 0 && report.damage >= self->energy) {
      ctx.blackboard.Set<bool>("threat_lethal", true);
    } else {
      ctx.blackboard.Erase("threat_lethal");
    }

    // The flag that justifies spending a defensive item: damage we cannot dodge out of, arriving in
    // an amount that leaves us critically low.
    //
    // It used to test whether the volley would *kill* us, and that was too late by a wide margin.
    // Measured across tv9, bots repelled at a median 7-21% energy having already absorbed 436-528
    // damage in the preceding second, while the human repelled at 33% with nothing landing yet. That
    // is the difference the threshold makes: a shot only becomes individually lethal once you are
    // already down to nothing, so a kill test necessarily fires after the fight is lost rather than
    // to prevent losing it. A repel exists to stop the hit that would put us in that state, not to
    // fail to survive it.
    //
    // Testing post-dodge damage rather than damage on the current course remains the important half:
    // that is the difference between "something dangerous is pointed at me" - true constantly in a
    // firefight - and "I am going to be hit by it".
    //
    // The recharge that lands before impact is included because the comparison is against the energy
    // we will have when it hits, not the energy we have now. Without it a bot sitting exactly on the
    // threshold spends an item on damage it was going to absorb with energy to spare.
    auto& game = *ctx.bot->game;
    float recharge_per_second = (float)game.ship_controller.ship.recharge / 10.0f;
    float energy_at_impact = self->energy + recharge_per_second * report.time_to_impact;

    float max_energy = (float)game.ship_controller.ship.energy;
    if (energy_at_impact > max_energy) energy_at_impact = max_energy;

    if (report.count > 0 && (energy_at_impact - report.unavoidable_damage) < max_energy * survival_floor) {
      ctx.blackboard.Set<bool>("threat_unavoidable", true);
    } else {
      ctx.blackboard.Erase("threat_unavoidable");
    }

    return report.count > 0 ? behavior::ExecuteResult::Success : behavior::ExecuteResult::Failure;
  }

  float check_distance = 20.0f;

  // Energy, as a fraction of a full tank, below which being knocked is considered losing the fight
  // rather than taking a hit. Sized from where the strong player actually spends a repel - around a
  // third of a tank, before the damage lands, rather than at the 7-21% the bots were reaching by
  // absorbing it first. Above this we are still in the exchange and an item is not the answer.
  float survival_floor = 0.20f;
};

}  // namespace teamversus
}  // namespace zero
