#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

#include <limits>

namespace zero {
namespace teamversus {

// Pure helpers shared by more than one team-versus node. Everything here is `inline` and stateless
// on purpose - the one-node-per-file rule exists so node *types* have a single definition, and free
// functions that several nodes need would otherwise get copy-pasted into each of them and drift
// apart (exactly the failure mode that produced the ShotSpreadNode ODR bug in the nexus zone).

// ---------------------------------------------------------------------------------------------
// Player filtering
// ---------------------------------------------------------------------------------------------

// True if `player` is someone we could actually interact with right now: in a ship, alive, not
// sitting in safe, and reporting a position we have any reason to trust.
//
// The synchronization check matters more than it looks. PlayerManager keeps the last known
// position of everyone, so a player who left our radar view keeps a stale position forever. Inside
// radar view we require fresh packets (if they're supposedly right there but silent, they've
// actually moved and we'd be aiming at a ghost); outside radar view we accept the stale position,
// because "where we last saw them" is genuinely the best information available and is what a human
// would work from too.
inline bool IsEngageablePlayer(Game& game, Player& player) {
  if (player.ship >= 8) return false;
  if (player.IsRespawning()) return false;
  if (player.position == Vector2f(0, 0)) return false;
  if (game.connection.map.GetTileId(player.position) == kTileIdSafe) return false;

  if (game.radar.InRadarView(player.position)) {
    return game.player_manager.IsSynchronized(player);
  }

  return true;
}

inline bool IsLiveTeammate(Game& game, const Player& self, Player& player) {
  if (player.id == self.id) return false;
  if (player.frequency != self.frequency) return false;

  return IsEngageablePlayer(game, player);
}

inline bool IsLiveEnemy(Game& game, const Player& self, Player& player) {
  if (player.frequency == self.frequency) return false;

  return IsEngageablePlayer(game, player);
}

// Returned by GetSupportDistance when a player has no living teammates at all. Deliberately a
// finite number rather than infinity so callers can do arithmetic on it without producing NaN.
inline constexpr float kNoSupportDistance = 1024.0f;

// Distance from `player` to their own closest living teammate, or kNoSupportDistance if they have
// none left. This is the single strongest lead indicator of who dies next in real matches: the
// median victim was 41 tiles from their nearest teammate two full seconds before dying, against a
// 27 tile baseline for everyone alive. It's used both to pick who to hunt and to decide whether
// we're the one who has drifted out on a limb.
inline float GetSupportDistance(Game& game, const Player& player) {
  float closest_sq = std::numeric_limits<float>::max();
  auto& pm = game.player_manager;

  for (size_t i = 0; i < pm.player_count; ++i) {
    Player* other = pm.players + i;

    if (other->id == player.id) continue;
    if (other->frequency != player.frequency) continue;
    if (!IsEngageablePlayer(game, *other)) continue;

    float dist_sq = other->position.DistanceSq(player.position);
    if (dist_sq < closest_sq) closest_sq = dist_sq;
  }

  if (closest_sq == std::numeric_limits<float>::max()) return kNoSupportDistance;

  return sqrtf(closest_sq);
}

// ---------------------------------------------------------------------------------------------
// Self energy
// ---------------------------------------------------------------------------------------------

// `ship_controller.ship.energy` is the ship's MAXIMUM energy - `Ship` is the stat block, not live
// state. Current energy lives on the Player. So this ratio reads like a tautology and isn't; every
// energy threshold in this zone goes through here to avoid getting it backwards.
inline float GetSelfEnergyPercent(Game& game, const Player& self) {
  u32 max_energy = game.ship_controller.ship.energy;
  if (max_energy == 0) return 0.0f;

  return self.energy / (float)max_energy;
}

// ---------------------------------------------------------------------------------------------
// Bomb blast
// ---------------------------------------------------------------------------------------------

// Blast radius in tiles. ShipController computes `BombExplodePixels + BombExplodePixels * level`,
// i.e. the radius scales with bomb level, and converts to tiles at 16 px/tile. At the common level
// this is around 10 tiles - much larger than the proximity fuse, which is why "will this bomb catch
// a friendly" is a question about the detonation point rather than about the flight path.
inline float GetBlastRadius(Game& game, u16 level) {
  float explode_pixels =
      (float)(game.connection.settings.BombExplodePixels + game.connection.settings.BombExplodePixels * (int)level);

  return explode_pixels / 16.0f;
}

// Fraction of a bomb's maximum damage actually delivered `distance` tiles from the detonation.
//
// This exists because GetEstimatedWeaponDamage returns the *maximum* bomb damage with no falloff at
// all, while the real formula in ShipController falls off linearly from the centre:
//
//     damage = (explode_pixels - max(0, dist_px - bomb_size)) * bomb_damage / explode_pixels
//
// A bomb at the rim of its blast does nothing; only a near-centre hit does full damage. Estimating
// incoming bomb damage without this overstates it several-fold, which is how a dodge system ends up
// panicking at every bomb on the screen and a repel system ends up spending repels on grazes.
inline float GetBlastDamageFraction(Game& game, u16 level, float distance) {
  float explode_pixels =
      (float)(game.connection.settings.BombExplodePixels + game.connection.settings.BombExplodePixels * (int)level);

  if (explode_pixels <= 0.0f) return 0.0f;

  // ShipController subtracts a 2px bomb size before applying falloff.
  constexpr float kBombSizePixels = 2.0f;

  float distance_pixels = distance * 16.0f - kBombSizePixels;
  if (distance_pixels < 0.0f) distance_pixels = 0.0f;
  if (distance_pixels >= explode_pixels) return 0.0f;

  return (explode_pixels - distance_pixels) / explode_pixels;
}

// Proximity fuse radius in tiles: `((ProximityDistance + level) * 18 - 14) / 16`.
//
// The fuse triggers on *ships only*, never on terrain - WeaponManager tests a single point against
// the map for wall collision and uses the proximity radius exclusively in the player scan. So a
// bomb only needs a clear lane up to fuse range of its target, not all the way to the aim point; a
// target standing against a wall is still bombable and terrain in that last stretch must not veto
// the shot.
inline float GetProximityRadius(Game& game, u16 level) {
  float radius_pixels = ((float)game.connection.settings.ProximityDistance + (float)level) * 18.0f - 14.0f;
  if (radius_pixels < 0.0f) radius_pixels = 0.0f;

  return radius_pixels / 16.0f;
}

// ---------------------------------------------------------------------------------------------
// Dodge capability
// ---------------------------------------------------------------------------------------------

// How far we can move *across* an incoming shot's path within `seconds`, in tiles.
//
// This is what turns "is there still time to dodge" from a guessed constant into something computed.
// A fixed reaction horizon cannot answer it, because the answer depends on where our nose currently
// points: thrust in this game acts only along the heading, so a ship already broadside to the shot
// starts displacing immediately, while one pointed straight down the shot's path has to spend most
// of the available time turning before any of its thrust does anything useful. Those two cases can
// differ by a factor of several, and treating them the same is what makes a bot burn a repel on
// something it could have simply flown out of.
//
// Two strategies are available and we get to take whichever is better:
//
//   - Thrust immediately along the current heading, forward or backward. Only the component of the
//     heading that lies across the shot contributes, but it starts working on tick one.
//   - Turn onto the escape axis first, then thrust with everything. Strictly better given enough
//     time, and strictly worse when there isn't enough left to complete the turn.
//
// Both use our own upgraded thrust and rotation stats, which - unlike an enemy's - we can read
// directly. Displacement is the usual (1/2)at^2; the shot's own travel is already accounted for by
// the caller, which measures closest approach in the relative frame.
inline float GetDodgeDistance(Game& game, const Player& self, const Vector2f& threat_direction, float seconds) {
  if (seconds <= 0.0f) return 0.0f;

  auto& ship = game.ship_controller.ship;

  float thrust = ship.thrust * (10.0f / 16.0f);
  if (thrust <= 0.0f) return 0.0f;

  // Rotation is in units where 400 is a full revolution per second.
  float rotation_rate = (ship.rotation / 400.0f) * 2.0f * 3.14159265f;

  Vector2f across = Perpendicular(threat_direction);

  // Fraction of our thrust that currently pushes across the shot rather than along it. Absolute
  // value because reverse thrust is just as good as forward for getting out of the way, and either
  // side of the shot's path counts as a miss.
  float alignment = fabsf(self.GetHeading().Dot(across));
  if (alignment > 1.0f) alignment = 1.0f;

  float immediate = 0.5f * thrust * alignment * seconds * seconds;

  float best = immediate;

  if (rotation_rate > 0.0f) {
    float turn_seconds = acosf(alignment) / rotation_rate;

    if (seconds > turn_seconds) {
      float remaining = seconds - turn_seconds;
      float aligned = 0.5f * thrust * remaining * remaining;

      if (aligned > best) best = aligned;
    }
  }

  return best;
}

// ---------------------------------------------------------------------------------------------
// Ship motion model
// ---------------------------------------------------------------------------------------------

// The observable constraints on any Subspace ship's motion, read out of arena settings rather than
// guessed. Used to forward-integrate a target instead of extrapolating its velocity in a straight
// line.
//
// An enemy's *upgraded* stats aren't visible to us - only their own ShipController knows those - so
// these are the arena ceilings for that ship type. That makes the model an upper bound on how
// sharply they can maneuver, which is the right side to err on for a prediction that gets clamped
// anyway.
struct ShipMotionLimits {
  float max_speed = 0.0f;     // tiles/sec
  float thrust = 0.0f;        // tiles/sec^2
  float max_rotation = 0.0f;  // radians/sec
};

inline ShipMotionLimits GetShipMotionLimits(Game& game, u8 ship) {
  ShipMotionLimits limits;

  if (ship >= 8) return limits;

  auto& settings = game.connection.settings.ShipSettings[ship];

  // Speed settings are pixels/second/10, and there are 16 pixels per tile.
  limits.max_speed = settings.MaximumSpeed / 16.0f / 10.0f;
  // Thrust is applied by ShipController as `thrust * 10 / 16` tiles/sec per second.
  limits.thrust = settings.MaximumThrust * (10.0f / 16.0f);
  // Rotation is in units where 400 is a full revolution per second.
  limits.max_rotation = (settings.MaximumRotation / 400.0f) * 2.0f * 3.14159265f;

  return limits;
}

// One step of the constrained motion model: rotate the heading at `turn_rate`, apply thrust along
// the (now rotated) heading, clamp to top speed, and integrate position.
//
// Thrust acts *only* along the heading - that is the entire reason this model beats free-vector
// extrapolation. Turning does not rotate existing velocity; it only changes where future thrust
// will point, which is why heading and velocity are tracked separately here rather than collapsed
// into a single acceleration estimate.
struct ShipMotionSample {
  Vector2f position;
  Vector2f velocity;
  Vector2f heading;
};

inline void StepShipMotion(ShipMotionSample& sample, const ShipMotionLimits& limits, float turn_rate,
                           float thrust_sign, float dt) {
  sample.heading = Rotate(sample.heading, turn_rate * dt);

  if (thrust_sign != 0.0f) {
    sample.velocity += sample.heading * (limits.thrust * thrust_sign * dt);

    float speed_sq = sample.velocity.LengthSq();
    if (limits.max_speed > 0.0f && speed_sq > limits.max_speed * limits.max_speed) {
      sample.velocity = Normalize(sample.velocity) * limits.max_speed;
    }
  }

  sample.position += sample.velocity * dt;
}

}  // namespace teamversus
}  // namespace zero
