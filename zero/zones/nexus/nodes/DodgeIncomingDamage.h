#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>
#include <zero/game/Logger.h>
#include <zero/zones/nexus/nodes/IncomingDamageQuery.h>

namespace zero {
namespace nexus {

// Nexus-specific fork of behavior::DodgeIncomingDamage (zero/behavior/nodes/ThreatNode.h).
//
// The shared version unconditionally overwrites steering.force every tick it runs, even when
// GetIncomingDamage finds no actual threat. In that case 'side' is computed from a degenerate
// zero-length ray (report.average_direction/average_origin are both (0,0) when weapon_count is 0),
// so Ray::GetClosestPosition collapses to the ray's origin and 'side' becomes
// Normalize(self->position) - a direction with no relation to any real danger, just self's
// absolute position on the map. Applying that as a hard force override fights whatever legitimate
// movement force Flee/aim-and-shoot already set that tick, which is what made bots appear to stall
// out whenever a bullet or bomb passed nearby without actually being on a collision course. This
// version leaves steering untouched when there's nothing to dodge, and blends its avoidance force
// in additively like every other steering method instead of overwriting.
struct DodgeIncomingDamage : public behavior::BehaviorNode {
  DodgeIncomingDamage(float damage_percent_threshold, float distance, float minimum_force = 2.0f)
      : damage_percent_threshold(damage_percent_threshold), distance(distance), minimum_force(minimum_force) {}
  DodgeIncomingDamage(float damage_percent_threshold, const char* distance_key, float minimum_force = 2.0f)
      : damage_percent_threshold(damage_percent_threshold), distance_key(distance_key), minimum_force(minimum_force) {}
  DodgeIncomingDamage(const char* damage_percent_threshold_key, const char* distance_key, float minimum_force = 2.0f)
      : damage_percent_threshold_key(damage_percent_threshold_key),
        distance_key(distance_key),
        minimum_force(minimum_force) {}

  behavior::ExecuteResult Execute(behavior::ExecuteContext& ctx) override {
    Player* self = ctx.bot->game->player_manager.GetSelf();

    if (!self) return behavior::ExecuteResult::Failure;
    if (self->ship >= 8) return behavior::ExecuteResult::Failure;

    float check_distance = distance;
    if (distance_key) {
      auto opt_distance = ctx.blackboard.Value<float>(distance_key);
      if (!opt_distance) return behavior::ExecuteResult::Failure;

      check_distance = *opt_distance;
    }

    float damage_percent_threshold = this->damage_percent_threshold;
    if (damage_percent_threshold_key) {
      auto opt_threshold = ctx.blackboard.Value<float>(damage_percent_threshold_key);
      if (!opt_threshold) return behavior::ExecuteResult::Failure;

      damage_percent_threshold = *opt_threshold;
    }

    IncomingDamageReport report = GetIncomingDamage(ctx, self, check_distance);

    // Nothing is actually on a collision course - leave steering alone instead of applying a
    // force derived from degenerate ray math that has no relation to any real threat.
    if (report.weapon_count == 0) return behavior::ExecuteResult::Failure;

    float est_damage = report.weapon_count * report.average_damage;
    float new_energy = self->energy - est_damage;
    float damage_percent = est_damage / (float)ctx.bot->game->ship_controller.ship.energy;

    Vector2f incoming_direction = Normalize(report.average_direction);
    Ray ray(report.average_origin, incoming_direction);
    Vector2f closest_hit = ray.GetClosestPosition(self->position);

    // The escape direction is "away from the threat line". That is undefined exactly when the
    // threat line runs through us - and a shot aimed straight at us is the most common case there
    // is. Normalize() returns a zero-length vector unchanged rather than NaN, so this used to
    // silently produce side = (0,0), and then `steering.force += side * 10000` added *nothing*
    // while this node still returned Success. Success short-circuits the whole fight Selector, so
    // the flee, the orbit and the aim-and-shoot branch below all got skipped too, leaving zero
    // steering force for the tick. Actuator treats no force as "release thrust", so the ship simply
    // stopped - and kept stopping for as long as the shot stayed in scan range. That is the
    // freeze-up: measured across the bot replays, stalls of 3, 5 and even 12 seconds, 80-100% of
    // them beginning within a second of an enemy firing, against a human maximum of 0.6s.
    //
    // Dodging perpendicular to the incoming line is the right answer for a head-on shot anyway, so
    // that's the fallback rather than giving up.
    constexpr float kMinOffsetSq = 0.01f;

    Vector2f offset = self->position - closest_hit;
    Vector2f side;

    // Threats that disagree about where they are coming from cannot be averaged, and this is the
    // second distinct way this node used to lock up. The head-on case above is one shot whose line
    // runs through us; THIS is several shots pointing opposite ways - the reported case being a mine
    // on one side and a bomb on the other.
    //
    // Their direction vectors cancel, so average_direction is left holding floating point residue
    // that normalizes to an essentially random unit vector, and average_origin lands midway between
    // them where nothing actually is. The escape derived from that is noise, and because the threats
    // keep moving it is DIFFERENT noise every tick: the bot shoves one way, then the other, and nets
    // no movement while looking like it cannot make up its mind. That is the "struggling to decide"
    // symptom exactly - not a stall this time but a dither, which is why the earlier freeze fix did
    // not catch it.
    //
    // There is also no averaged answer to find. Running from either threat runs into the other; the
    // only way out from between them is SIDEWAYS. So when coherence is low we ignore the average
    // entirely and slip perpendicular to the bearing of the biggest threat, picking whichever
    // perpendicular we are already moving toward so the dodge keeps our momentum instead of
    // fighting it.
    bool incoherent = report.weapon_count > 1 && report.direction_coherence < kMinDirectionCoherence;

    if (incoherent && report.strongest_bearing.LengthSq() > 0.0f) {
      side = Perpendicular(report.strongest_bearing);

      float velocity_alignment = side.Dot(self->velocity);
      if (velocity_alignment < 0.0f) {
        side = side * -1.0f;
      } else if (velocity_alignment == 0.0f && side.Dot(self->GetHeading()) < 0.0f) {
        side = side * -1.0f;
      }
    } else if (offset.LengthSq() > kMinOffsetSq) {
      side = Normalize(offset);
    } else if (incoming_direction.LengthSq() > 0.0f) {
      side = Perpendicular(incoming_direction);
    } else {
      // No coherent threat line at all. Let the rest of the tree drive rather than holding the
      // Selector open while contributing nothing.
      return behavior::ExecuteResult::Failure;
    }

    if (est_damage > 0) {
      Vector3f color = Vector3f(1, 1, 0);
      if (damage_percent >= damage_percent_threshold || new_energy <= 0) {
        color = Vector3f(0, 1, 0);
      }

      ctx.bot->game->line_renderer.PushLine(self->position, Vector3f(1, 1, 0), self->position + side * 5.0f, color);
      ctx.bot->game->line_renderer.Render(ctx.bot->game->camera);
    }

    float force = 10000.0f;
    auto result = behavior::ExecuteResult::Success;

    // If we won't die then we should let the rest of the behavior tree run and apply only a small amount of force.
    if (damage_percent < damage_percent_threshold && new_energy > 0) {
      force = minimum_force + damage_percent * 10.0f;
      result = behavior::ExecuteResult::Failure;
    }

    // We are going to die or taking too much damage. Make the force very strong.
    ctx.bot->bot_controller->steering.force += side * force;

    return result;
  }

  // Below this level of agreement between incoming threats, the average direction is not a
  // description of anything and we sidestep instead. 0.5 is the resultant length of two unit vectors
  // 120 degrees apart, so anything from a wide spread up to directly opposing counts as incoherent,
  // while a couple of shots from broadly the same side still averages normally.
  static constexpr float kMinDirectionCoherence = 0.5f;

  float damage_percent_threshold = 0.0f;
  float distance = 0.0f;
  float minimum_force = 2.0f;
  const char* distance_key = nullptr;
  const char* damage_percent_threshold_key = nullptr;
};

}  // namespace nexus
}  // namespace zero
