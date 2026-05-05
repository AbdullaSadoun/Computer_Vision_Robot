/*
GridPlanner.h
- A* occupancy-grid path planner for the Mode 5 map-based attack pipeline (USE_VISION)
- Builds a coarse grid inside the arena polygon, inflates obstacles by (robot_radius + clearance_px)
- Finds the nearest grid cell that has clear LOS to the enemy position as the A* goal
- Returns a polyline of warped-arena pixel waypoints for the Mode5AttackPlanner to follow
by: Abdulla Sadoun
Date: April 1, 2026
*/
#pragma once

#include <utility>
#include <vector>

#include "../../CoreTypes.h"
#include "../../Simulation/Safety.h"

// Builds a coarse occupancy grid inside the arena polygon, runs A* to a cell with
// clear line-of-sight to the enemy, returns a polyline in warped-arena pixels.
struct GridPlanResult {
    bool ok = false;
    std::vector<std::pair<float, float>> waypoints;
};

class GridPlanner {
public:
    // `clearance_px` is extra radius beyond robot (maps ~1.5–2 in via px/in upstream).
    static GridPlanResult plan(const ArenaInfo& arena, const std::vector<Obstacle>& obstacles,
                               float start_x, float start_y, float enemy_x, float enemy_y,
                               float cell_px, float robot_radius_px, float clearance_px,
                               float shoot_standoff_px, float los_step,
                               float polygon_edge_extra_px);
};
