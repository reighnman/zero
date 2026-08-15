#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/behavior/nodes/AimNode.h>
#include <zero/game/Game.h>
#include <zero/zones/teamversus/nodes/CombatMath.h>

namespace zero {
namespace teamversus {

// Answers one question: if we pull the trigger right now, does the shot connect and is it safe to
// take? Success means fire.
//
// This replaces the usual four-node chain (bounding box, move rectangle, ray, ray-rectangle
// intercept) plus a separate scattering of distance checks, because those checks are not actually
// independent - they all depend on which weapon is being fired, and getting the interactions wrong
// is subtle. Three specific interactions this gets right:
//
//  1. Nothing in a ray-versus-bounding-box test knows terrain exists. A target on the far side of a
//     wall produces a perfectly valid-looking intercept and the bot cheerfully fires into solid
//     tiles. Bullets and bombs do not pass through walls; thors do.
//
//  2. A bomb's line of sight only has to be clear up to *proximity fuse range* of the target, not
//     all the way to the aim point. The fuse triggers on ships only and never on terrain, so
//     terrain in that last stretch cannot stop the bomb - and must not veto the shot, or a target
//     standing against a wall becomes permanently un-bombable.
//
//  3. A bomb does not detonate on a teammate; WeaponManager skips same-frequency players in its
//     collision scan, so a friendly in the flight path is passed straight through. Team damage
//     comes purely from blast splash wherever the bomb *does* go off. So the right safety check is
//     "will a friendly be near the detonation point", not "is a friendly in the lane" - and the same
//     applies to us, since the blast radius is far larger than the fuse radius and a close bomb
//     catches its own shooter.
//
// Two details about which vectors go into the test:
//
//  - The fire direction is the *current heading*, not the aim direction. Weapons fire along the
//    hull's orientation whatever the aim solver wanted, so testing the aim direction would approve
//    shots the ship is not actually going to take.
//  - Everything is done in world space, against the target's predicted *world* position - not
//    against the shooter-frame lead point. The projectile's world velocity includes our own, and
//    terrain and blast radii are world-space facts. Mixing the two frames introduces an error
//    proportional to our speed times the flight time, which at 15 tiles/sec over a 1.5 second
//    bullet flight is over twenty tiles - larger than the thing being tested.
//
// The hit tolerance grows with range, and that is not slop for its own sake. Weapons fire on the
// quantized 40-step orientation, so the fire direction snaps to 9 degree increments and carries an
// unavoidable +-4.5 degrees no matter how good the aim solution is. That is about 2.3 tiles of
// spread at the 29 tile median firing range. A tolerance tighter than the hull can physically be
// aimed does not make the bot more accurate; it just makes it refuse to fire at range, and a bot
// that goes quiet in front of an enemy loses fights it would otherwise win.
struct ShotClearanceNode : public behavior::BehaviorNode {
  ShotClearanceNode(WeaponType weapon_type, const char* predicted_key, float hit_radius_multiplier)
      : weapon_type(weapon_type), predicted_key(predicted_key), hit_radius_multiplier(hit_radius_multiplier) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();
    if (!self || self->ship >= 8) return behavior::ExecuteResult::Failure;

    auto opt_predicted = ctx.blackboard.Value<Vector2f>(predicted_key);
    if (!opt_predicted) return behavior::ExecuteResult::Failure;

    Vector2f predicted = *opt_predicted;
    auto& game = *ctx.bot->game;

    float weapon_speed = behavior::GetWeaponSpeed(game, *self, weapon_type);
    Vector2f shot_velocity = self->velocity + self->GetHeading() * weapon_speed;

    if (shot_velocity.LengthSq() <= 0.0f) return behavior::ExecuteResult::Failure;

    Vector2f shot_direction = Normalize(shot_velocity);

    // --- does the trajectory actually cross the target? ----------------------------------------
    float ship_radius = game.connection.settings.ShipSettings[self->ship].GetRadius();
    float range = predicted.Distance(self->position);
    float half_extent = ship_radius * hit_radius_multiplier + range * kOrientationQuantizationTangent;

    Rectangle target_bounds(predicted - Vector2f(half_extent, half_extent),
                            predicted + Vector2f(half_extent, half_extent));

    float intercept_distance = 0.0f;
    if (!RayBoxIntersect(Ray(self->position, shot_direction), target_bounds, &intercept_distance, nullptr)) {
      return behavior::ExecuteResult::Failure;
    }

    bool is_bomb = weapon_type == WeaponType::Bomb || weapon_type == WeaponType::ProximityBomb;

    // --- line of sight -------------------------------------------------------------------------
    // Thors travel through walls, so a terrain check would only ever wrongly veto them.
    if (weapon_type != WeaponType::Thor) {
      float cast_distance = intercept_distance;

      if (is_bomb) {
        // Stop short by fuse range - see interaction (2) above.
        cast_distance -= GetProximityRadius(game, GetBombLevel(game));
      }

      if (cast_distance > 0.0f) {
        Vector2f cast_start = self->position + shot_direction * ship_radius;

        CastResult result = game.GetMap().Cast(cast_start, shot_direction, cast_distance, self->frequency);
        if (result.hit) return behavior::ExecuteResult::Failure;
      }
    }

    // --- blast safety --------------------------------------------------------------------------
    if (is_bomb || weapon_type == WeaponType::Thor) {
      Vector2f detonation = self->position + shot_direction * intercept_distance;
      float blast_radius = GetBlastRadius(game, GetBombLevel(game));

      // Us first. The blast is far bigger than the fuse, so a bomb thrown at something too close
      // detonates inside our own radius.
      if (detonation.DistanceSq(self->position) < blast_radius * blast_radius) {
        return behavior::ExecuteResult::Failure;
      }

      auto& pm = game.player_manager;
      for (size_t i = 0; i < pm.player_count; ++i) {
        Player* mate = pm.players + i;

        if (!IsLiveTeammate(game, *self, *mate)) continue;

        if (mate->position.DistanceSq(detonation) < blast_radius * blast_radius) {
          return behavior::ExecuteResult::Failure;
        }
      }
    }

    return behavior::ExecuteResult::Success;
  }

  WeaponType weapon_type;
  const char* predicted_key = nullptr;
  // Slack around the target's hull, on top of the range-scaled quantization term above. Bullets
  // want this tight because they need a real hit; bombs want it loose because a near miss still
  // delivers blast damage, which is how a bomb is actually used - as area denial rather than a
  // precision shot.
  float hit_radius_multiplier = 1.0f;

 private:
  // tan(4.5 degrees) - half of the 9 degree step the ship's orientation is quantized to, and
  // therefore the irreducible angular error on every shot.
  static constexpr float kOrientationQuantizationTangent = 0.0787f;

  // Our own bomb level, which sets both the fuse radius and the blast radius.
  static u16 GetBombLevel(Game& game) {
    u32 bombs = game.ship_controller.ship.bombs;
    return bombs > 0 ? (u16)(bombs - 1) : (u16)0;
  }
};

}  // namespace teamversus
}  // namespace zero
