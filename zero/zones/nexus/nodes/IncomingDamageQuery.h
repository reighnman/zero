#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

struct IncomingDamageReport {
  Vector2f average_direction;
  Vector2f average_origin;
  float average_damage;
  u32 weapon_count;
};

// Fraction of a blast weapon's maximum damage we would actually take if it went off at
// `miss_distance` tiles from us, mirroring the falloff ShipController applies:
//
//     distance = max(0, dist_pixels - kBombSize)
//     damage   = (explode_pixels - distance) * bomb_damage / explode_pixels
//
// Returns 0 outside the blast and 1 for a dead-centre hit. Without this, a bomb that will pass at
// the rim of its own blast - doing essentially nothing - is scored as a full-damage hit, which both
// makes the bot panic-dodge harmless shots and makes it spend repels on survivable ones.
inline float GetBlastDamageFraction(Connection& connection, u8 level, float miss_distance) {
  float explode_pixels = (float)(connection.settings.BombExplodePixels + connection.settings.BombExplodePixels * level);
  if (explode_pixels <= 0.0f) return 1.0f;

  constexpr float kBombSize = 2.0f;
  float distance_pixels = miss_distance * 16.0f - kBombSize;
  if (distance_pixels < 0.0f) distance_pixels = 0.0f;
  if (distance_pixels >= explode_pixels) return 0.0f;

  return (explode_pixels - distance_pixels) / explode_pixels;
}

// Nearest future approach of a weapon to a player, in tiles. Negative when it is already moving
// away, so callers can discard it.
inline float GetClosestApproach(Player& self, Weapon& weapon) {
  Vector2f delta = weapon.position - self.position;
  Vector2f relative_velocity = weapon.velocity - self.velocity;

  float speed_sq = relative_velocity.LengthSq();
  if (speed_sq < 0.0001f) return delta.Length();

  float t = -delta.Dot(relative_velocity) / speed_sq;
  if (t < 0.0f) return -1.0f;

  float remaining_seconds = TICK_DIFF(weapon.end_tick, GetCurrentTick()) / 100.0f;
  if (remaining_seconds <= 0.0f) return -1.0f;
  if (t > remaining_seconds) t = remaining_seconds;

  return (delta + relative_velocity * t).Length();
}

// Scans nearby enemy weapons and estimates how much damage is on a collision course with self
// within `check_distance`, along with the averaged origin/direction of the threat. Shared by
// DodgeIncomingDamage (hard escape) and DodgeJukeNode (light nudge that keeps aim on target).
inline IncomingDamageReport GetIncomingDamage(behavior::ExecuteContext& ctx, Player* self, float check_distance) {
  float distance_sq = check_distance * check_distance;
  float ship_radius = ctx.bot->game->connection.settings.ShipSettings[self->ship].GetRadius();
  float bounds_extent = ship_radius * 2.0f;

  Rectangle self_bounds(self->position - Vector2f(bounds_extent, bounds_extent),
                        self->position + Vector2f(bounds_extent, bounds_extent));

  Vector2f average_direction;
  Vector2f average_origin;
  float average_damage = 0.0f;
  size_t incoming_count = 0;

  auto& weapon_man = ctx.bot->game->weapon_manager;
  for (size_t i = 0; i < weapon_man.weapon_count; ++i) {
    Weapon& weapon = weapon_man.weapons[i];

    if (weapon.frequency == self->frequency) continue;
    if (weapon.data.type == WeaponType::Repel || weapon.data.type == WeaponType::Decoy) continue;
    if (weapon.data.type == WeaponType::Burst && !(weapon.flags & WEAPON_FLAG_BURST_ACTIVE)) continue;
    if (weapon.position.DistanceSq(self->position) > distance_sq) continue;

    bool is_mine = (weapon.data.type == WeaponType::Bomb || weapon.data.type == WeaponType::ProximityBomb) &&
                   weapon.data.alternate;

    Vector2f relative_velocity = weapon.velocity - self->velocity;

    // A weapon that isn't closing on us cannot reach us, and its "incoming direction" is
    // meaningless. This matters because Normalize() returns a zero vector unchanged instead of
    // NaN, so a near-stationary relative velocity used to yield direction = (0,0) - which makes the
    // ray test below degenerate and poisons average_direction, and downstream leaves
    // DodgeIncomingDamage with no usable escape direction. The case is real, not theoretical: a
    // mine has exactly zero velocity, and a slow bomb tracked by a slow ship is close to it. Those
    // are precisely the conditions the freeze-ups were reported under.
    constexpr float kMinClosingSpeedSq = 0.25f;
    if (relative_velocity.LengthSq() < kMinClosingSpeedSq) continue;

    Vector2f direction = Normalize(relative_velocity);
    Rectangle check_bounds = self_bounds;

    if (weapon.data.type == WeaponType::Bomb) {
      // Grow by bomb size plus some extra pixels to be certain.
      check_bounds = check_bounds.Grow(6.0f / 16.0f);
    } else if (weapon.data.type == WeaponType::ProximityBomb) {
      float prox_radius =
          ((float)ctx.bot->game->connection.settings.ProximityDistance + (float)weapon.data.level) + (2.0f / 16.0f);

      check_bounds = check_bounds.Grow(prox_radius);
    }

    Rectangle view_bounds = check_bounds.Translate(-self->position);
    view_bounds = view_bounds.Translate(weapon.position);

    ctx.bot->game->line_renderer.PushRect(view_bounds, Vector3f(1, 0, 0));

    // The amount of ticks that we are uncertain of with weapon alive time.
    // This will cause it to attempt to dodge weapons that might be right outside of hitting range.
    constexpr u32 kSlopTicks = 30;

    float remaining_distance =
        weapon.velocity.Length() * (TICK_DIFF(MAKE_TICK(weapon.end_tick + kSlopTicks), GetCurrentTick()) / 100.0f);

    float dist = 0.0f;

    if (RayBoxIntersect(Ray(weapon.position, direction), check_bounds, &dist, nullptr)) {
      // Ignore weapons that will time out before reaching us.
      if (dist > remaining_distance && !is_mine) continue;

      // Reduce the amount of impact this weapon will have based on its distance away.
      float threat_percent = (check_distance - dist) / (check_distance * 0.7f);
      if (threat_percent > 1.0f) threat_percent = 1.0f;
      if (threat_percent < 0.0f) threat_percent = 0.0f;

      float damage = (float)GetEstimatedWeaponDamage(weapon, ctx.bot->game->connection) * threat_percent;

      // Scale a blast weapon down by how far off-centre it will actually go off. GetEstimatedWeaponDamage
      // reports the maximum a bomb can do, which is only right for a near-direct hit.
      if (weapon.data.type == WeaponType::Bomb || weapon.data.type == WeaponType::ProximityBomb ||
          weapon.data.type == WeaponType::Thor) {
        float closest = GetClosestApproach(*self, weapon);
        if (closest < 0.0f) continue;

        damage *= GetBlastDamageFraction(ctx.bot->game->connection, weapon.data.level, closest);
        if (damage <= 0.0f) continue;
      }
      Vector2f weighted_direction = direction * threat_percent;

      // Reduce the effect of this direction if the damage is lower than average.
      if (average_damage > 0) {
        weighted_direction *= damage / average_damage;
      }

      // Running mean: new_mean = (sample + n * old_mean) / (n + 1). This ASSIGNS - it used to be
      // `+=`, which adds the new mean on top of the old one and compounds every iteration. With two
      // equal-damage weapons that reported 2x the true mean, with three about 3.7x, and it grew from
      // there. DodgeIncomingDamage then multiplies by weapon_count on top, so the error landed
      // hardest exactly when several weapons were inbound - the outnumbered fights where the bot
      // could least afford it. Dodging short-circuits the entire fight Selector, so an inflated
      // estimate did not merely cause a needless dodge, it suppressed that tick's offense
      // altogether.
      //
      // No threshold retune is needed, which is why this can be fixed on its own: for a single
      // weapon the old expression already produced the true value (n=0 makes it (d + 0)/1), so the
      // one-weapon case that the 0.2 damage_percent_threshold was tuned against is unchanged. Only
      // the multi-weapon case moves, and it moves from wrong to correct.
      average_direction =
          (weighted_direction + (float)incoming_count * average_direction) / ((float)incoming_count + 1);
      average_origin = (weapon.position + (float)incoming_count * average_origin) / ((float)incoming_count + 1);
      average_damage = (damage + (float)incoming_count * average_damage) / ((float)incoming_count + 1);
      ++incoming_count;
    }
  }

  ctx.bot->game->line_renderer.Render(ctx.bot->game->camera);

  IncomingDamageReport report;
  report.average_damage = average_damage;
  report.average_direction = average_direction;
  report.average_origin = average_origin;
  report.weapon_count = (u32)incoming_count;

  return report;
}

}  // namespace nexus
}  // namespace zero
