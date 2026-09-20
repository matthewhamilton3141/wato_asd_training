#include "planner_core.hpp"

#include <cmath>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <algorithm>

namespace robot
{

PlannerCore::PlannerCore(const rclcpp::Logger& logger) 
: logger_(logger) {}

void PlannerCore::configure(int obstacle_threshold, double cost_weight, double snap_radius)
{
  obstacle_threshold_ = obstacle_threshold;
  cost_weight_ = cost_weight;
  snap_radius_ = snap_radius;
}

bool PlannerCore::worldToCell(const nav_msgs::msg::OccupancyGrid& map, double wx, double wy, CellIndex& out) const
{
  const double res = map.info.resolution;
  const int x = static_cast<int>(std::floor((wx - map.info.origin.position.x) / res));
  const int y = static_cast<int>(std::floor((wy - map.info.origin.position.y) / res));
  if (x < 0 || y < 0 || x >= static_cast<int>(map.info.width) || y >= static_cast<int>(map.info.height)) {
    return false;
  }
  out = CellIndex(x, y);
  return true;
}

void PlannerCore::cellToWorld(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& c, double& wx, double& wy) const
{
  const double res = map.info.resolution;
  wx = map.info.origin.position.x + (c.x + 0.5) * res;
  wy = map.info.origin.position.y + (c.y + 0.5) * res;
}

int8_t PlannerCore::costAt(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& c) const
{
  if (c.x < 0 || c.y < 0 || c.x >= static_cast<int>(map.info.width) || c.y >= static_cast<int>(map.info.height)) {
    return 100;
  }
  const int8_t v = map.data[c.y * map.info.width + c.x];
  return v < 0 ? 0 : v;  // unknown space is treated as free
}

bool PlannerCore::isFree(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& c) const
{
  return costAt(map, c) < obstacle_threshold_;
}

std::optional<CellIndex> PlannerCore::nearestFree(const nav_msgs::msg::OccupancyGrid& map, const CellIndex& c) const
{
  if (isFree(map, c)) return c;

  const int r = static_cast<int>(std::ceil(snap_radius_ / map.info.resolution));
  std::queue<CellIndex> q;
  std::unordered_set<CellIndex, CellIndexHash> seen;
  q.push(c);
  seen.insert(c);
  while (!q.empty()) {
    const CellIndex cur = q.front();
    q.pop();
    for (int dy = -1; dy <= 1; ++dy) {
      for (int dx = -1; dx <= 1; ++dx) {
        if (!dx && !dy) continue;
        const CellIndex n(cur.x + dx, cur.y + dy);
        if (std::abs(n.x - c.x) > r || std::abs(n.y - c.y) > r) continue;
        if (n.x < 0 || n.y < 0 || n.x >= static_cast<int>(map.info.width) || n.y >= static_cast<int>(map.info.height)) continue;
        if (!seen.insert(n).second) continue;
        if (isFree(map, n)) return n;
        q.push(n);
      }
    }
  }
  return std::nullopt;
}

std::optional<nav_msgs::msg::Path> PlannerCore::plan(const nav_msgs::msg::OccupancyGrid& map,
                                                     double sx, double sy, double gx, double gy)
{
  CellIndex start_raw, goal_raw;
  if (!worldToCell(map, sx, sy, start_raw)) {
    RCLCPP_WARN(logger_, "Start (%.2f, %.2f) is outside the map", sx, sy);
    return std::nullopt;
  }
  if (!worldToCell(map, gx, gy, goal_raw)) {
    RCLCPP_WARN(logger_, "Goal (%.2f, %.2f) is outside the map", gx, gy);
    return std::nullopt;
  }

  // The robot may legitimately be sitting inside the inflation zone (it is
  // 2m long) and users click goals near walls, so nudge both onto free cells.
  const auto start = nearestFree(map, start_raw);
  const auto goal = nearestFree(map, goal_raw);
  if (!start) {
    RCLCPP_WARN(logger_, "No free cell within %.1fm of start", snap_radius_);
    return std::nullopt;
  }
  if (!goal) {
    RCLCPP_WARN(logger_, "No free cell within %.1fm of goal", snap_radius_);
    return std::nullopt;
  }
  if (*goal != goal_raw) {
    RCLCPP_INFO(logger_, "Goal was in an obstacle, snapped %d cells away",
                std::max(std::abs(goal->x - goal_raw.x), std::abs(goal->y - goal_raw.y)));
  }

  const double res = map.info.resolution;
  auto heuristic = [&](const CellIndex& c) {
    return std::hypot(c.x - goal->x, c.y - goal->y) * res;
  };

  std::priority_queue<AStarNode, std::vector<AStarNode>, CompareF> open;
  std::unordered_map<CellIndex, double, CellIndexHash> g_score;
  std::unordered_map<CellIndex, CellIndex, CellIndexHash> came_from;
  std::unordered_set<CellIndex, CellIndexHash> closed;

  g_score[*start] = 0.0;
  open.emplace(*start, heuristic(*start));

  static const int kDx[8] = {1, -1, 0, 0, 1, 1, -1, -1};
  static const int kDy[8] = {0, 0, 1, -1, 1, -1, 1, -1};

  bool found = false;
  size_t expanded = 0;
  while (!open.empty()) {
    const CellIndex cur = open.top().index;
    open.pop();
    if (closed.count(cur)) continue;   // stale duplicate entry
    closed.insert(cur);
    ++expanded;

    if (cur == *goal) { found = true; break; }

    const double g_cur = g_score[cur];
    for (int k = 0; k < 8; ++k) {
      const CellIndex n(cur.x + kDx[k], cur.y + kDy[k]);
      if (!isFree(map, n) || closed.count(n)) continue;

      const bool diagonal = kDx[k] != 0 && kDy[k] != 0;
      // Don't let a diagonal step squeeze between two blocked orthogonal cells.
      if (diagonal && (!isFree(map, CellIndex(cur.x + kDx[k], cur.y)) ||
                       !isFree(map, CellIndex(cur.x, cur.y + kDy[k])))) {
        continue;
      }

      const double step = (diagonal ? std::sqrt(2.0) : 1.0) * res;
      const double penalty = 1.0 + cost_weight_ * (costAt(map, n) / 100.0);
      const double g_new = g_cur + step * penalty;

      auto it = g_score.find(n);
      if (it == g_score.end() || g_new < it->second) {
        g_score[n] = g_new;
        came_from[n] = cur;
        open.emplace(n, g_new + heuristic(n));
      }
    }
  }

  if (!found) {
    RCLCPP_WARN(logger_, "A* found no path after expanding %zu cells", expanded);
    return std::nullopt;
  }

  // Walk back from goal to start, then reverse.
  std::vector<CellIndex> cells;
  for (CellIndex c = *goal; c != *start; c = came_from.at(c)) cells.push_back(c);
  cells.push_back(*start);
  std::reverse(cells.begin(), cells.end());

  nav_msgs::msg::Path path;
  path.header.frame_id = map.header.frame_id;

  auto add_pose = [&](double wx, double wy) {
    geometry_msgs::msg::PoseStamped p;
    p.header = path.header;
    p.pose.position.x = wx;
    p.pose.position.y = wy;
    p.pose.orientation.w = 1.0;
    path.poses.push_back(p);
  };

  add_pose(sx, sy);  // exact robot position first
  for (size_t i = 1; i + 1 < cells.size(); ++i) {
    double wx, wy;
    cellToWorld(map, cells[i], wx, wy);
    add_pose(wx, wy);
  }
  if (*goal == goal_raw) {
    add_pose(gx, gy);  // exact clicked goal last
  } else {
    double wx, wy;
    cellToWorld(map, *goal, wx, wy);
    add_pose(wx, wy);
  }

  RCLCPP_INFO(logger_, "A* path: %zu poses, %.1fm, expanded %zu cells",
              path.poses.size(), g_score[*goal], expanded);
  return path;
}

} 
