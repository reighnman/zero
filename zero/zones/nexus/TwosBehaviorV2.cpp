#include "TwosBehaviorV2.h"

#include <zero/zones/nexus/NexusTeamBehaviorV2.h>

namespace zero {
namespace nexus {

// The 2v2 variant of the shared V2 team tree - see NexusTeamBehaviorV2.cpp for the behavior itself
// and the replay analysis behind its tuning. Twos is the mode where roster size changes the logic
// rather than just the numbers: with a single teammate the regroup lookup collapses to that one
// player (NexusTeamConfigV2::Twos sets regroup_teammate_factor to 1).
std::unique_ptr<behavior::BehaviorNode> TwosBehaviorV2::CreateTree(behavior::ExecuteContext& ctx) {
  return CreateNexusTeamTreeV2(ctx, NexusTeamConfigV2::Twos());
}

}  // namespace nexus
}  // namespace zero
