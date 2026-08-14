#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/WeaponManager.h>

#include <unordered_set>

namespace zero {
namespace nexus {

// Estimates how much damage is actually about to land on us, modelling the game's real blast
// falloff instead of assuming every bomb is a dead-centre hit.
//
// The existing estimators (svs::IncomingDamageQueryNode and the shared ThreatNode) both sum
// GetEstimatedWeaponDamage, which returns BombDamageLevel flat - the *maximum* a bomb can do. But
// ShipController applies blast damage on a linear falloff from the detonation point:
//
//     distance = max(0, dist_pixels - kBombSize)
//     damage   = (explode_pixels - distance) * bomb_damage / explode_pixels
//
// with explode_pixels = BombExplodePixels * (1 + level). So a bomb going off at the rim of its
// blast does nothing at all, one at half radius does about half, and only a near-direct hit does
// the full amount. Treating them all as full damage overstates incoming damage by several times
// whenever a bomb is merely nearby.
//
// That overstatement is what burns repels. The repel gate fires when estimated incoming damage
// exceeds current energy, so a pair of bombs that would really have clipped us for a fraction each
// used to read as instantly lethal and spend a repel - and a wasted repel is close to a wasted
// life, since it is the only thing that stops damage outright. Measured over the 4v4 bot replays,
// individual damage events taken by the losing bots reach 400-900 raw energy (p90 396-487) against
// the human's 419 maximum and a 222 p90, i.e. the bots were repeatedly caught near blast centres
// while the human clipped edges - so the bots have both problems at once: they eat centre hits, and
// they spend repels on hits that would not have killed them.
//
// Closest approach is computed in the weapon's frame relative to us, so a bomb that will pass wide
// is correctly scored as harmless rather than as a full hit that happens to be aimed near us.
struct IncomingBlastDamageNode : public behavior::BehaviorNode {
  IncomingBlastDamageNode(float check_distance, const char* output_key)
      : check_distance(check_distance), output_key(output_key) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    Game& game = *ctx.bot->game;
    auto& settings = game.connection.settings;

    float ship_radius = settings.ShipSettings[self->ship].GetRadius();
    float distance_sq = check_distance * check_distance;
    float total_damage = 0.0f;

    links.clear();

    auto& weapon_man = game.weapon_manager;
    for (size_t i = 0; i < weapon_man.weapon_count; ++i) {
      Weapon& weapon = weapon_man.weapons[i];

      if (weapon.frequency == self->frequency) continue;
      if (weapon.data.type == WeaponType::Repel || weapon.data.type == WeaponType::Decoy) continue;
      if (weapon.data.type == WeaponType::Burst && !(weapon.flags & WEAPON_FLAG_BURST_ACTIVE)) continue;
      if (weapon.position.DistanceSq(self->position) > distance_sq) continue;

      // Multifire/shrapnel share a link id; counting every fragment separately would inflate the
      // estimate the same way flat bomb damage does.
      if (weapon.link_id != kInvalidLink && links.find(weapon.link_id) != links.end()) continue;

      float base_damage = (float)GetEstimatedWeaponDamage(weapon, game.connection);
      if (base_damage <= 0.0f) continue;

      float closest = ClosestApproach(*self, weapon);
      if (closest < 0.0f) continue;  // travelling away from us, or already past

      bool is_blast = weapon.data.type == WeaponType::Bomb || weapon.data.type == WeaponType::ProximityBomb ||
                      weapon.data.type == WeaponType::Thor;

      float damage = 0.0f;

      if (is_blast) {
        float explode_pixels = (float)(settings.BombExplodePixels + settings.BombExplodePixels * weapon.data.level);
        if (explode_pixels <= 0.0f) continue;

        // Mirror ShipController's falloff exactly, in pixels.
        constexpr float kBombSize = 2.0f;
        float distance_pixels = closest * 16.0f - kBombSize;
        if (distance_pixels < 0.0f) distance_pixels = 0.0f;

        if (distance_pixels >= explode_pixels) continue;  // outside the blast entirely

        damage = (explode_pixels - distance_pixels) * (base_damage / explode_pixels);
      } else {
        // Bullets are all-or-nothing: they either intersect our hull or they don't.
        if (closest > ship_radius) continue;
        damage = base_damage;
      }

      total_damage += damage;

      if (weapon.link_id != kInvalidLink) links.insert(weapon.link_id);
    }

    ctx.blackboard.Set<float>(output_key, total_damage);

    return behavior::ExecuteResult::Success;
  }

 private:
  // Distance of the weapon's nearest future approach to us, in tiles. Negative when it is already
  // moving away, so the caller can discard it.
  float ClosestApproach(Player& self, Weapon& weapon) {
    Vector2f delta = weapon.position - self.position;
    Vector2f relative_velocity = weapon.velocity - self.velocity;

    float speed_sq = relative_velocity.LengthSq();

    // A stationary weapon relative to us (a mine, or matched velocity) stays exactly where it is.
    if (speed_sq < 0.0001f) return delta.Length();

    float t = -delta.Dot(relative_velocity) / speed_sq;
    if (t < 0.0f) return -1.0f;

    // Don't credit a weapon with reaching a point it will expire before arriving at.
    float remaining_seconds = TICK_DIFF(weapon.end_tick, GetCurrentTick()) / 100.0f;
    if (remaining_seconds <= 0.0f) return -1.0f;
    if (t > remaining_seconds) t = remaining_seconds;

    return (delta + relative_velocity * t).Length();
  }

  float check_distance = 0.0f;
  const char* output_key = nullptr;

  std::unordered_set<u32> links;
};

}  // namespace nexus
}  // namespace zero
