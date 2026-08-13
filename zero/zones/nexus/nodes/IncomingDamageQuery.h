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
      Vector2f weighted_direction = direction * threat_percent;

      // Reduce the effect of this direction if the damage is lower than average.
      if (average_damage > 0) {
        weighted_direction *= damage / average_damage;
      }

      average_direction +=
          (weighted_direction + (float)incoming_count * average_direction) / ((float)incoming_count + 1);
      average_origin += (weapon.position + (float)incoming_count * average_origin) / ((float)incoming_count + 1);
      average_damage += (damage + (float)incoming_count * average_damage) / ((float)incoming_count + 1);
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
