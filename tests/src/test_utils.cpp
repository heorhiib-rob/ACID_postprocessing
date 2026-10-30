/*
 * Author: Jan Chleboun
 * Date: 12-02-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#include "test_utils.h"

#include <gtest/gtest.h>

#include <climits>
#include <filesystem>

#include "SafeIntervalTable.h"


std::string get_base_path_tests()
{
  return std::filesystem::canonical("/proc/self/exe").parent_path().parent_path().parent_path().string();  // Linux only
}

TimePointPath generate_random_timepointpath(int len_wanted)
{
  int                      len             = len_wanted - 1;  // decrease the length as the last element has to be handled separately
  TimePointPath            tp_path         = {};
  int                      i               = 0;
  int                      cur_loc         = 0;  // start at (0, 0)
  const int                instance_width  = 5;  // lets say the instance has width 5
  const std::array<int, 4> possible_shifts = {-instance_width, -1, 1, instance_width};
  while (i < len)
  {
    // choose a random count of elements (maximum 10)
    int count = std::min(rand() % 10 + 1, len - i);

    // choose a random length of the interval
    int interval_len = i + count - 1 + rand() % (INT_MAX - i - count + 1);

    // add the timepoint to the path
    tp_path.emplace_back(cur_loc, TimeInterval(i, interval_len));

    // increment i
    i += count;

    // choose a random neighboring position
    cur_loc = cur_loc + possible_shifts[rand() % 4];

    // make sure the position is valid
    while (cur_loc < 0 || cur_loc > instance_width * instance_width) cur_loc = cur_loc + possible_shifts[rand() % 4];
  }
  // last interval
  tp_path.emplace_back(cur_loc, TimeInterval(i, INT_MAX));

  return tp_path;
}

auto filter_time_intervals(std::pair<std::list<TimeInterval>::const_iterator, std::list<TimeInterval>::const_iterator> start_goal,
                           TimeInterval interval, int from, int to, SafeIntervalTable& table) -> std::vector<TimeInterval>
{
  std::list<TimeInterval>::const_iterator start;
  std::list<TimeInterval>::const_iterator end;
  std::tie(start, end) = start_goal;
  std::vector<TimeInterval> ret;

  for (auto it = start; it != end; it++)
  {
    TimeInterval new_interval(it->t_min, it->t_max);
    // modify the interval to be valid
    if (new_interval.t_min < interval.t_min)
    {
      new_interval.t_min = interval.t_min;
    }
    assertm(new_interval.t_min <= new_interval.t_max, "Modifying the time interval made it invalid.");

    // check edge collision
    if (table.edge_constraint_table.get(to, from, new_interval.t_min))
    {
      continue;
    }

    // add to the covnerted intervals
    ret.push_back(new_interval);
  }
  return ret;
}

auto find_path_distance_gradient(int agent_num, const Instance& instance) -> Path
{
  Path ret;
  int  start = instance.get_start_locations()[agent_num];
  ret.push_back(start);
  int curr      = start;
  int curr_dist = instance.get_heuristic_distance(agent_num, curr);
  while (curr_dist > 0)
  {
    // find neighbor locations
    const std::vector<int>& neighbors = instance.get_neighbor_locations(curr);
    bool                    improved  = false;
    // iterate over the neighbors
    for (int neighbor : neighbors)
    {
      // check if the neighbor is closer to the goal
      if (instance.get_heuristic_distance(agent_num, neighbor) < curr_dist)
      {
        curr      = neighbor;
        curr_dist = instance.get_heuristic_distance(agent_num, curr);
        ret.push_back(curr);
        improved = true;
        break;
      }
    }
    assertm(improved, "No neighbor was closer to the goal.");
  }
  return ret;
}

