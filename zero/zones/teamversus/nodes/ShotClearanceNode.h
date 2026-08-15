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
//     tiles. Bullets and bombs do not pass through walls; thors do - but a thor still explodes with
//     the same splash when it finally stops, so passing through walls exempts it from the
//     line-of-sight veto and from nothing else. Its blast is checked exactly like a bomb's, and the
//     blast itself does not respect walls either, so a teammate sheltering behind terrain near the
//     detonation is still hit.
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
  ShotClearanceNode(WeaponType weapon_type, const char* predicted_key, float hit_radius_multiplier,
                    float max_flight_time)
      : weapon_type(weapon_type),
        predicted_key(predicted_key),
        hit_radius_multiplier(hit_radius_multiplier),
        max_flight_time(max_flight_time) {}

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

    // --- will it arrive before they can move? --------------------------------------------------
    // A projectile leaves at our velocity plus the muzzle velocity, so our own motion is part of how
    // fast the shot crosses the ground - and ground speed is what decides whether the target has
    // time to dodge. Standing still, the shot travels at the bare muzzle speed of 12.5 tiles/sec,
    // which is *slower than the ship it is chasing* (20.3 top speed): at 30 tiles that is a 2.4
    // second flight, and the target gets to make several independent decisions inside it.
    //
    // Expressing this as a flight-time cap rather than a minimum speed makes it scale with range on
    // its own, which is the behaviour we want. Up close the muzzle speed alone satisfies it and our
    // velocity is irrelevant; the further out the target is, the more of our own speed has to be
    // going into the shot before it is worth taking. It also falls out naturally in an orbit, where
    // the in-out pump means the closing half of each cycle can take the shot and the receding half
    // holds - which is the same accelerate-fire-reverse pattern real players volley with.
    //
    // Note the frame. This is a *world*-frame flight time, measured along the actual trajectory to
    // the intercept point, and is a different quantity from InterceptAimNode's shooter-frame flight
    // time - that one asks how long the solver has to extrapolate, this one asks how long the target
    // has to react.
    // How long the target is allowed to be given also depends on how fast *they* are travelling,
    // which is a separate fact from which way their hull points and has to be read separately. A
    // ship sitting still cannot exploit a long flight no matter how long it is; one already moving
    // at 20 tiles/sec displaces a full ship length every twentieth of a second, so the same flight
    // time is a far weaker shot against them. Scale the window down with their speed rather than
    // treating every target as equally hard to lead.
    float target_speed = ctx.blackboard.ValueOr<float>("target_speed", 0.0f);
    float ship_max_speed = game.connection.settings.ShipSettings[self->ship].MaximumSpeed / 16.0f / 10.0f;

    float mobility = ship_max_speed > 0.0f ? target_speed / ship_max_speed : 0.0f;
    if (mobility > 1.0f) mobility = 1.0f;

    float allowed_flight_time = max_flight_time * (1.0f - mobility * mobility_penalty);

    float shot_speed = shot_velocity.Length();
    if (intercept_distance / shot_speed > allowed_flight_time) return behavior::ExecuteResult::Failure;

    bool is_bomb = weapon_type == WeaponType::Bomb || weapon_type == WeaponType::ProximityBomb;

    Vector2f cast_start = self->position + shot_direction * ship_radius;

    // --- line of sight -------------------------------------------------------------------------
    // Thors travel through walls, so a terrain check would only ever wrongly veto them.
    if (weapon_type != WeaponType::Thor) {
      float cast_distance = intercept_distance;

      if (is_bomb) {
        // Stop short by fuse range - see interaction (2) above.
        cast_distance -= GetProximityRadius(game, GetBombLevel(game));
      }

      if (cast_distance > 0.0f) {
        CastResult result = game.GetMap().Cast(cast_start, shot_direction, cast_distance, self->frequency);
        if (result.hit) return behavior::ExecuteResult::Failure;
      }
    }

    // --- blast safety --------------------------------------------------------------------------
    // A bomb explodes on whatever stops it first - an enemy's proximity fuse or a wall - and the
    // blast is indiscriminate. It does not detonate *on* a teammate (WeaponManager skips
    // same-frequency players in its collision scan, so a friendly in the lane is passed straight
    // through), which is exactly why the question to ask is "who is near where it goes off", never
    // "who is in the way".
    if (is_bomb || weapon_type == WeaponType::Thor) {
      // Where does it actually detonate? The fuse trips at the target, but a wall short of them
      // stops it sooner - and for bombs the line-of-sight test above deliberately casts *past* any
      // terrain inside fuse range, so that case gets through the veto and has to be modelled here
      // rather than assumed away. Thors ignore walls entirely.
      float detonation_distance = intercept_distance;

      if (weapon_type != WeaponType::Thor) {
        CastResult terrain = game.GetMap().Cast(cast_start, shot_direction, intercept_distance, self->frequency);

        if (terrain.hit && terrain.distance < detonation_distance) {
          detonation_distance = terrain.distance;
        }
      }

      Vector2f detonation = cast_start + shot_direction * detonation_distance;
      float detonation_time = detonation_distance / shot_speed;

      u16 bomb_level = GetBombLevel(game);

      // Everyone is checked at where they are now *and* where they will be when it goes off. The
      // blast happens in the future, and at 20 tiles/sec a ship covers two blast radii during a
      // one-and-a-half second flight - so testing current positions alone clears bombs that our own
      // team then flies into, which is the same frame-mixing mistake as aiming at a lead point in
      // world space. Neither sample is authoritative on its own, so the worse of the two decides.
      if (WorstBlastFraction(game, bomb_level, self->position, self->velocity, detonation, detonation_time) >=
          self_blast_limit) {
        return behavior::ExecuteResult::Failure;
      }

      auto& pm = game.player_manager;
      for (size_t i = 0; i < pm.player_count; ++i) {
        Player* mate = pm.players + i;

        if (!IsLiveTeammate(game, *self, *mate)) continue;

        if (WorstBlastFraction(game, bomb_level, mate->position, mate->velocity, detonation, detonation_time) >=
            team_blast_limit) {
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

  // Longest world-frame flight this weapon is allowed to accept, in seconds - i.e. how much warning
  // we are willing to give the target. Tight for bullets, which need a real hit and are cheap enough
  // to wait for a better one. Looser for bombs and thors, because a bomb that arrives late still
  // denies the space it lands in and guiding an opponent is a legitimate use even when the shot was
  // never going to connect.
  float max_flight_time = 1.5f;

  // How much of that window a target moving at full speed gives up. At 0.25 a stationary target may
  // be shot at across the full flight time and one at the speed cap gets 75% of it.
  //
  // Deliberately mild, because it multiplies with the cap above and the two together are what
  // decides whether the bot fires at all. Most shots in a real fight are at a moving target, so a
  // heavy penalty here is really just a lower cap wearing a disguise - and the corpus says humans
  // take those shots.
  float mobility_penalty = 0.25f;

  // Blast tolerances, as a fraction of the bomb's maximum damage (750 here, 44% of a full tank).
  //
  // Expressed as damage rather than as "inside the blast radius" because blast damage falls off
  // linearly from the centre - a bomb at the rim of its blast does literally nothing. A hard radius
  // veto treats a graze at 9 tiles the same as a direct hit, and since the radius is 10 tiles while
  // teammates sit at a p10 spacing of 8, that would refuse most of the bombs worth throwing.
  //
  // Ours is tighter than theirs: we are the one already committed to this fight, and self-inflicted
  // damage lands on top of whatever the enemy is doing to us at the same moment.
  float self_blast_limit = 0.15f;
  float team_blast_limit = 0.25f;

 private:
  // Blast damage fraction at whichever of a player's current and predicted positions is worse.
  static float WorstBlastFraction(Game& game, u16 level, const Vector2f& position, const Vector2f& velocity,
                                  const Vector2f& detonation, float seconds) {
    float now = GetBlastDamageFraction(game, level, position.Distance(detonation));
    float later = GetBlastDamageFraction(game, level, (position + velocity * seconds).Distance(detonation));

    return now > later ? now : later;
  }

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
