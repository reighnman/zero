#pragma once

#include <zero/BotController.h>
#include <zero/ZeroBot.h>
#include <zero/behavior/BehaviorTree.h>
#include <zero/game/Game.h>

namespace zero {
namespace nexus {

// Describes how boxed-in a position is, rather than merely whether a wall happens to be near it.
//
// Proximity to a wall is a bad danger signal on its own: hugging a single flat wall in open space
// is perfectly safe (and useful - it's cover, and bullets bounce off it), while sitting in the
// mouth of a dead end is lethal even though the nearest wall may be no closer. What actually
// matters is how many directions remain available and whether the ones that do remain form a
// single narrow escape corridor.
struct EnclosureReport {
  // Fraction of probed directions that stayed clear all the way out to the probe distance.
  // 1.0 = open field, 0.0 = fully walled in.
  float open_fraction = 1.0f;

  // Width (radians) of the widest unbroken span of clear directions. A corner has a small value
  // even when open_fraction is moderate, because its clear directions are split across two
  // disconnected arcs.
  float escape_arc = 6.28318f;

  // Center of that widest clear span - the direction to move to get out.
  Vector2f escape_direction;

  // Mean clear distance over all probes, useful as a soft "how much room is there" measure.
  float average_clearance = 0.0f;
};

// Casts a ring of rays from `position` and summarizes the room available.
//
// `probe_distance` sets what counts as "open" - it should be a few ship-lengths, large enough that
// a shallow alcove reads as closed but small enough that ordinary map geometry doesn't.
inline EnclosureReport GetEnclosure(Game& game, const Vector2f& position, u16 frequency, float radius,
                                    float probe_distance, size_t sample_count = 16) {
  constexpr float kTwoPi = 6.28318f;

  EnclosureReport report;

  if (sample_count == 0) return report;

  // Cast the ring, recording which directions stayed clear.
  bool open[64] = {};
  if (sample_count > 64) sample_count = 64;

  size_t open_count = 0;
  float clearance_total = 0.0f;

  for (size_t i = 0; i < sample_count; ++i) {
    float angle = (kTwoPi / sample_count) * i;
    Vector2f direction = Rotate(Vector2f(1, 0), angle);
    Vector2f start = position + direction * radius;

    CastResult result = game.GetMap().Cast(start, direction, probe_distance, frequency);

    float clearance = result.hit ? result.distance : probe_distance;
    clearance_total += clearance;

    open[i] = !result.hit;
    if (open[i]) ++open_count;
  }

  report.open_fraction = (float)open_count / (float)sample_count;
  report.average_clearance = clearance_total / (float)sample_count;

  if (open_count == 0) {
    // Fully enclosed at this probe distance - fall back to whichever direction had the most room
    // so a caller still has something better than nothing to steer along.
    float best_clearance = -1.0f;

    for (size_t i = 0; i < sample_count; ++i) {
      float angle = (kTwoPi / sample_count) * i;
      Vector2f direction = Rotate(Vector2f(1, 0), angle);
      Vector2f start = position + direction * radius;

      CastResult result = game.GetMap().Cast(start, direction, probe_distance, frequency);
      float clearance = result.hit ? result.distance : probe_distance;

      if (clearance > best_clearance) {
        best_clearance = clearance;
        report.escape_direction = direction;
      }
    }

    report.escape_arc = 0.0f;
    return report;
  }

  if (open_count == sample_count) {
    report.escape_arc = kTwoPi;
    report.escape_direction = Vector2f(1, 0);
    return report;
  }

  // Find the longest circular run of open directions. Starting from a closed index guarantees the
  // scan never splits a run across the array boundary.
  size_t start_index = 0;
  while (start_index < sample_count && open[start_index]) ++start_index;

  size_t best_run_start = 0;
  size_t best_run_length = 0;
  size_t run_start = 0;
  size_t run_length = 0;

  for (size_t offset = 0; offset < sample_count; ++offset) {
    size_t i = (start_index + offset) % sample_count;

    if (open[i]) {
      if (run_length == 0) run_start = i;
      ++run_length;

      if (run_length > best_run_length) {
        best_run_length = run_length;
        best_run_start = run_start;
      }
    } else {
      run_length = 0;
    }
  }

  report.escape_arc = (kTwoPi / sample_count) * best_run_length;

  float center_angle = (kTwoPi / sample_count) * (best_run_start + (best_run_length - 1) * 0.5f);
  report.escape_direction = Rotate(Vector2f(1, 0), center_angle);

  return report;
}

}  // namespace nexus
}  // namespace zero
