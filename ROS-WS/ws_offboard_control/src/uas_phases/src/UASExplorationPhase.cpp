#include "uas_phases/UASExplorationPhase.h"


UASExplorationPhase::UASExplorationPhase(std::vector<UASState> waypoints) : waypoints_(waypoints)
{
    phaseName_ = "Exploration";
    waypointIndex_ = 0;
}

UASState UASExplorationPhase::generateDesiredState(const UASState& uasState)
{   
    if(distance(uasState, waypoints_[waypointIndex_]) < 0.5) {
        waypointIndex_++;
        if(waypointIndex_ == waypoints_.size()) {
            waypointIndex_ = 0;
        }
    }

    return waypoints_[waypointIndex_];
}