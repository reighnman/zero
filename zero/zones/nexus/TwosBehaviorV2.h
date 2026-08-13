#pragma once

#include <zero/Math.h>
#include <zero/behavior/Behavior.h>
#include <zero/behavior/BehaviorBuilder.h>

namespace zero {
namespace nexus {

// 2v2 variant of the V2 team behavior. Like ThreesBehaviorV2 it is a thin wrapper over the shared
// tree in NexusTeamBehaviorV2.cpp, so tuning changes apply to every mode rather than drifting apart
// between copies - the mode differences live in NexusTeamConfigV2::Twos().
struct TwosBehaviorV2 : public behavior::Behavior {
  void OnInitialize(behavior::ExecuteContext& ctx) override {
    ctx.blackboard.Set("request_ship", 4);
    ctx.blackboard.Set("startup_damage_trigger", "50");

    std::vector<Vector2f> waypoints{
        Vector2f(410, 415), Vector2f(615, 395), Vector2f(515, 545), Vector2f(505, 680), Vector2f(355, 545),
    };

    ctx.blackboard.Set("waypoints", waypoints);
  }

  std::unique_ptr<behavior::BehaviorNode> CreateTree(behavior::ExecuteContext& ctx) override;
};

}  // namespace nexus
}  // namespace zero
