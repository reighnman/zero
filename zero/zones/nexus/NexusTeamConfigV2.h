#pragma once

#include <zero/Types.h>

namespace zero {
namespace nexus {

// Every tunable the shared V2 team tree uses, so twos/threes/fours can differ in more than which
// queue they join.
//
// The modes are not the same game at a different headcount. The arena map is sized to the roster,
// so anything expressed as a map distance (regroup range, break-off distance, how far away still
// counts as "the fight we're in") has to shrink for smaller modes or the bot will wander off
// looking for space that doesn't exist. Roster size also changes the logic itself, not just the
// numbers: a twos player has exactly one teammate, so "fall back toward the second-nearest
// teammate" is meaningless there and has to collapse to the only teammate.
//
// What deliberately does NOT scale is engagement range. Weapon accuracy versus distance is a
// property of the weapons, not the arena - a bullet fired from 30 tiles misses just as often on a
// small map as a large one - so orbit distance and the weapon bands stay fixed across modes.
//
// Provenance: the Fours() values are measured (ZeroReplayAnalyzer over 21 real 4v4 SVS league
// matches). Every 4v4 replay in that corpus was a fours match, so there is NO threes or twos data
// behind Threes()/Twos() - their map-scaled distances are estimates derived from the fours numbers
// and flagged below. They are the first thing to revisit if a smaller mode plays badly.
struct NexusTeamConfigV2 {
  // Matchmaking queue for this mode. ChatMessageNode copies this into a std::string, so a literal
  // is fine.
  const char* queue_command = "?next 4v4pub";

  // Players per side. Drives the roster-size-dependent branches rather than any distance.
  u32 team_size = 4;

  // Which teammate to fall back toward when regrouping with the wider team: 1 = nearest,
  // 2 = second-nearest (stay with the group rather than trailing one straggler). Must never exceed
  // team_size - 1 or the lookup can never succeed and the regroup branch silently dies.
  u32 regroup_teammate_factor = 2;

  // --- Target selection.
  float low_energy_distance_threshold = 15.0f;
  float low_energy_target_percent = 0.35f;

  // --- Engagement phases (EngagementPhaseNode).
  // Killed players sat at a median 10% (p75 20%) of their own max energy at death, at a median
  // killer distance of ~11 tiles.
  float finish_target_percent = 0.20f;
  float finish_distance = 12.0f;
  // Must still be a fight we're actually in rather than one across the map - scales with the arena.
  float press_distance = 36.0f;
  u32 press_ticks = 300;  // ~3s
  float disadvantage_enter_ratio = 0.65f;
  float disadvantage_exit_ratio = 0.9f;
  float critical_energy_percent = 0.10f;
  // Killer's own energy at a real kill: median 50%, p25 30%.
  float finish_min_self_energy_percent = 0.4f;
  u32 finish_repel_threshold = 1;

  // --- Engagement range. Fixed across modes; see the class comment.
  //
  // Reconstructing damage from the position stream (energy drops on samples where the player did
  // not themselves fire) gives an empirical bullet hit rate per firing range over 93k shots:
  //
  //     0-9 tiles : 52.6%      30-39 tiles : 8.0%
  //   10-19 tiles : 23.1%      40-49 tiles : 6.9%
  //   20-29 tiles :  9.9%      50+ tiles   : ~5-6%  (at/near the false-attribution noise floor)
  //
  // These players habitually fired from a median of 30 tiles, where fewer than one shot in ten
  // connects. Copying their observed range would faithfully reproduce their ineffectiveness, so the
  // bot deliberately fights closer than they do - it has an exact firing solution and tick-rate
  // reactions, so it can hold a range humans find uncomfortable.
  float orbit_distance = 15.0f;

  // --- Map-scaled positioning.
  //
  // CURRENTLY the twos values are used for every mode (see the factories below), so these defaults
  // are the twos ones rather than the fours ones. The measured/estimated per-mode values are kept
  // in the factory comments so they can be restored without re-deriving them.
  float leash_distance = 26.0f;
  float team_range = 26.0f;
  float avoid_team_distance = 7.5f;  // teammate spacing p10 (tight end) was ~7.6

  // --- Local numbers. Bucketing every player-snapshot by (friendly - enemy) head count within 25
  // tiles, the fraction actively closing runs 40% at -2 or worse, 48% at -1, then 59-60% at 0/+1/+2
  // - players flip from backing off to committing right at parity. Two seconds before dying the
  // median victim had ZERO teammates within that radius while carrying 1-2 attackers.
  float swarm_radius = 18.0f;
  float commit_advantage = 0.0f;      // push at parity or better
  float break_off_advantage = -1.0f;  // disengage when outnumbered

  // --- Defence and aiming geometry (mode-independent).
  // Repel was used at a median distance of ~13.6 tiles (p25 ~9.2), so the incoming-damage check
  // needs room to see a lethal shot coming rather than reacting point-blank.
  float repel_detection_distance = 10.0f;
  float bomb_required_damage_overlap = 300.0f;
  float shot_spread_distance_threshold = 40.0f;
  float aim_lead_bias_seconds = 0.2f;
  float shot_spread_maneuvering_normalizer = 4.0f;

  // --- Terrain. Walls are non-destructive (contact bounces you), so proximity is not the danger -
  // losing escape options is. These feed EnclosureQuery.h rather than a nearest-wall check, which
  // lets the bot fight alongside obstacles and bail out only when actually being funnelled.
  float enclosure_probe_distance = 14.0f;
  float self_min_open_fraction = 0.35f;
  float self_min_escape_arc = 1.2f;  // radians, ~70 degrees
  float trap_target_open_fraction = 0.4f;
  float trap_target_escape_arc = 1.6f;  // radians, ~92 degrees
  float trap_block_distance = 8.0f;

  // --- Weapon bands (WeaponEnergyBandNode), from the per-weapon (self energy%, distance)
  // distributions at the moment each weapon was actually fired. Fixed across modes.
  float bullet_min_energy_percent = 0.4f;
  float bomb_min_energy_percent = 0.5f;
  // Bomb hit rate collapses like bullets: 13.8% at 10-19 tiles, 6.5% at 20-29, ~5% beyond. Lobbing
  // from the observed p90 of 47 tiles is close to pure energy waste.
  float bomb_max_distance = 24.0f;
  float bomb_self_guard_distance = 6.0f;
  float thor_min_energy_percent = 0.5f;
  float thor_max_distance = 25.0f;
  u32 thor_cooldown_ticks = 1500;  // only ~2.5 thors per player per match were actually spent
  float decoy_min_energy_percent = 0.45f;

  // Bombs herd rather than snipe: across 18k bomb shots the launch heading sat a median +12 degrees
  // (p75 +39) toward the side the target was sliding, against targets moving laterally at a median
  // 6.9 tiles/sec.
  float bomb_herd_lead_seconds = 0.6f;
  float bomb_herd_min_lateral_speed = 3.0f;

  u32 fire_lull_ticks = 130;

  // NOTE: all three modes currently share the twos tuning (the struct defaults above) while the
  // behavior is being tested. Only the queue, roster size and the roster-derived regroup factor
  // differ right now. Restoring per-mode distances means re-adding the overrides recorded in each
  // factory's comment.
  //
  // The fours numbers are the only measured ones - every replay analysed was a 4v4 league match -
  // so the threes values were always estimates anyway.

  // Per-mode distances when restored (measured from 4v4 league play):
  //   leash 40, team_range 43, press 56, swarm 25, low_energy_distance 22
  static NexusTeamConfigV2 Fours() {
    NexusTeamConfigV2 config;

    config.queue_command = "?next 4v4pub";
    config.team_size = 4;
    config.regroup_teammate_factor = 2;

    return config;
  }

  // Per-mode distances when restored (estimated by scaling the fours values down for the smaller
  // arena, never measured):
  //   leash 32, team_range 34, press 45, swarm 21, low_energy_distance 18
  static NexusTeamConfigV2 Threes() {
    NexusTeamConfigV2 config;

    config.queue_command = "?next 3v3pub";
    config.team_size = 3;
    config.regroup_teammate_factor = 2;

    return config;
  }

  // Smallest arena, and with a single teammate the "regroup with the wider group" idea collapses -
  // there is only ever one other player to fall back to. Its distances are the struct defaults.
  static NexusTeamConfigV2 Twos() {
    NexusTeamConfigV2 config;

    config.queue_command = "?next 2v2pub";
    config.team_size = 2;
    config.regroup_teammate_factor = 1;

    return config;
  }
};

}  // namespace nexus
}  // namespace zero
