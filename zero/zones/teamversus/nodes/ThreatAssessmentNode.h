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

// Everything the defensive half of the tree needs to know about what is currently flying at us.
struct ThreatReport {
  // Total damage we expect to actually take if we hold our current course, in raw energy.
  float damage = 0.0f;
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

    float damage = 0.0f;

    if (is_bomb) {
      // Only bombs need a detonation model at all: the proximity fuse trips before contact, so the
      // blast centre sits at roughly the closest approach point rather than on our hull. That means
      // closest approach is also the distance the falloff should be evaluated at.
      float blast_radius = GetBlastRadius(game, weapon.data.level);

      // Outside the blast entirely - genuinely harmless, so skip it rather than dodging a bomb that
      // was going to sail past doing nothing.
      if (approach_distance > blast_radius) continue;

      float max_damage = (float)GetEstimatedWeaponDamage(weapon, game.connection);
      damage = max_damage * GetBlastDamageFraction(game, weapon.data.level, approach_distance);
    } else {
      // Bullets have no fuse, so they only matter if the shot actually crosses our hull.
      if (approach_distance > ship_radius * kBulletHitboxSlop) continue;

      damage = (float)GetEstimatedWeaponDamage(weapon, game.connection);
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

    return report.count > 0 ? behavior::ExecuteResult::Success : behavior::ExecuteResult::Failure;
  }

  float check_distance = 20.0f;
};

}  // namespace teamversus
}  // namespace zero
