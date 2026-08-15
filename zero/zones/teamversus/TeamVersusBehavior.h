#pragma once

#include <zero/behavior/Behavior.h>

namespace zero {
namespace teamversus {

// The team-versus knockout behavior. Built for elimination matches of any size (2v2 through
// many-v-many) where each player has a fixed number of lives.
//
// See TeamVersusBehavior.cpp for the tree itself and for the reasoning behind its shape.
struct TeamVersusBehavior : public behavior::Behavior {
  void OnInitialize(behavior::ExecuteContext& ctx) override;
  std::unique_ptr<behavior::BehaviorNode> CreateTree(behavior::ExecuteContext& ctx) override;
};

}  // namespace teamversus
}  // namespace zero
