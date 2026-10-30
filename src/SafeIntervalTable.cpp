/*
 * Author: Jan Chleboun
 * Date: 14-02-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#include "SafeIntervalTable.h"

#include <boost/functional/hash.hpp>
#include <climits>
#include <iostream>

auto EdgeConstraint::operator==(const EdgeConstraint& other) const -> bool
{
  return from == other.from && to == other.to && t == other.t;
}

SafeIntervalTable::SafeIntervalTable(const Instance& instance_)
    : edge_constraint_table(instance_),
      instance(instance_),
      unlimited_safe_intervals(instance_.get_map_data().get_num_free_cells())
{
  const int num_free_cells = instance.get_num_free_cells();

  // initialize the safe interval vectors, cells with static obstacles dont need safe interval
  safe_intervals = std::vector<std::list<TimeInterval>>(num_free_cells, std::list<TimeInterval>(1, TimeInterval(0, INT_MAX)));
  ci_list_for_cell = std::vector<std::list<CIWithCount>>(num_free_cells, std::list<CIWithCount>(1, CIWithCount(0, INT_MAX)));
}

void SafeIntervalTable::add_constraint(const TimePoint& timepoint)
{
  // check the location is valid
  assertm(instance.get_map_data().is_in(timepoint.location), "Invalid location.");

  if(allow_soft_collisions){
    change_ci_count(timepoint.location, timepoint.interval.t_min, timepoint.interval.t_max, 1);
    return;
  }

  // update unlimited safe interval number
  if (timepoint.interval.t_max == INT_MAX)
  {
    unlimited_safe_intervals--;
    assertm(unlimited_safe_intervals >= 0, "Invalid unlimited safe interval count.");
  }
  else
  {
    // update latest constraint end
    latest_constraint_end = std::max(latest_constraint_end, timepoint.interval.t_max);
  }


  int                      free_location = instance.location_to_free_location(timepoint.location);
  std::list<TimeInterval>& cur_TI_list   = safe_intervals[free_location];

  // modify intervals that have some intersections with the time range
  auto it = cur_TI_list.begin();
  while (it != cur_TI_list.end())
  {
    // skip irrelevant time intervals (e.g., end before the constraint begins)
    if (it->t_max < timepoint.interval.t_min)
    {
      it++;
      continue;
    }


    // check that there are still intervals that could be reduced
    // yeah if this doesn't hold, the constraint goes over the safe interval.
    // however, what if we continue
    assertm(it->t_min <= timepoint.interval.t_max, "Can not add an overlapping constraint.");

    // shorten intervals starting before the range
    if (it->t_min < timepoint.interval.t_min)
    {
      assertm(it->t_max >= timepoint.interval.t_max, "Can not add an overlapping constraint.");
      const int new_tmax = timepoint.interval.t_min - 1;


      // create new interval if there is still some time in the safe interval after the constraint
      if (it->t_max > timepoint.interval.t_max)
      {
        cur_TI_list.insert(std::next(it, 1), TimeInterval(timepoint.interval.t_max + 1, it->t_max));
        it->t_max = new_tmax;
        break;
      }
      const int tmax = it->t_max;

      it->t_max = new_tmax;

      // check whether there might be any other affected intervals
      if (tmax == timepoint.interval.t_max)
      {
        break;
      }
      it++;
    }
    // shorten intervals ending after the range
    else if (it->t_max > timepoint.interval.t_max)
    {
      assertm(it->t_min == timepoint.interval.t_min, "Can not add an overlapping constraint.");
      it->t_min = timepoint.interval.t_max + 1;
      break;
    }
    // delete intervals that are fully overlapped by the constraint
    else
    {
      const int tmax = it->t_max;
      it             = cur_TI_list.erase(it);

      // check whether there can be any other affected ontervals
      if (tmax == timepoint.interval.t_max)
      {
        break;
      }
    }
  }
}


void SafeIntervalTable::remove_constraint(const TimePoint& timepoint)
{
  // check the location is valid
  assertm(instance.get_map_data().is_in(timepoint.location), "Invalid location.");

  if(allow_soft_collisions){
    change_ci_count(timepoint.location, timepoint.interval.t_min, timepoint.interval.t_max, -1);
    return;
  }

  // update unlimited safe interval number
  if (timepoint.interval.t_max == INT_MAX)
  {
    unlimited_safe_intervals++;
    assertm(unlimited_safe_intervals <= instance.get_map_data().get_num_free_cells(), "Invalid unlimited safe interval count.");

    // update latest constraint done in remove constraints
  }

  int                      free_location = instance.location_to_free_location(timepoint.location);
  std::list<TimeInterval>& cur_TI_list   = safe_intervals[free_location];

  // check whether there are any safe intervals
  if (cur_TI_list.empty())
  {
    cur_TI_list.push_back(timepoint.interval);
    return;
  }

  // firstly handle constraints after the last safe interval (there is no safe interval after the constraint)
  assertm(timepoint.interval.t_min != cur_TI_list.back().t_max, "Constraint start overlaps with last safe interval end.");

  if (timepoint.interval.t_min > cur_TI_list.back().t_max)
  {
    // if the last safe interval precedes directly, extend it
    if (cur_TI_list.back().t_max == timepoint.interval.t_min - 1)
    {
      cur_TI_list.back().t_max = timepoint.interval.t_max;
      return;
    }
    // otherwise add a new safe interval
    cur_TI_list.push_back(timepoint.interval);
    return;
  }


  // find interval after the constraint
  auto it = cur_TI_list.begin();
  while (it != cur_TI_list.end())
  {

    // find the interval after the constraint
    if (it->t_min > timepoint.interval.t_max)
    {
      // check whether there was an interval before the constraint
      if (it != cur_TI_list.begin())
      {
        // check whether the previous timeinterval can be extended
        const auto prev = std::prev(it, 1);
        if (prev->t_max == timepoint.interval.t_min - 1)
        {
          // check whether prev should be merged with next interval
          if (timepoint.interval.t_max != INT_MAX && it->t_min == timepoint.interval.t_max + 1)
          {
            prev->t_max = it->t_max;
            it          = cur_TI_list.erase(it);
          }
          else  // extend prev
          {
            prev->t_max = timepoint.interval.t_max;
          }
          return;
        }
      }

      // previous can not be extended, check whether next can be extended
      if (timepoint.interval.t_max != INT_MAX && it->t_min == timepoint.interval.t_max + 1)
      {
        it->t_min = timepoint.interval.t_min;
        return;
      }

      // no interval can be extended, therefore insert a new interval
      cur_TI_list.insert(it, timepoint.interval);
      return;
    }
    it++;
  }
  throw std::runtime_error("SafeIntervalTable Remove Constraints: Did not remove any constraint.");
}

void SafeIntervalTable::add_constraints(const TimePointPath& path)
{
  for (int i = 0; i < (int)path.size(); i++)
  {
    // add the point constraint
    add_constraint(path[i]);

    // add the edge constraint
    if (i > 0)  // skip the first point as there is no edge
    {
      // the time when the edge is used is the start time of the later interval
      edge_constraint_table.add(path[i - 1].location, path[i].location, path[i].interval.t_min);
    }
  }
}


void SafeIntervalTable::remove_constraints(const TimePointPath& path)
{
  for (int i = 0; i < (int)path.size(); i++)
  {
    // remove the point constraint
    remove_constraint(path[i]);

    // remove the edge constraint
    if (i > 0)  // skip the first point as there is no edge
    {
      edge_constraint_table.remove(path[i - 1].location, path[i].location, path[i].interval.t_min);
    }
  }

  // remember that it will be needed to update the latest constraint estimate
  latest_constraint_end_updated = false;
  // update_latest_constraint_end_estimate();
}


auto SafeIntervalTable::get_first_safe_interval(int location) const -> std::list<TimeInterval>::const_iterator
{
  // check the location is valid
  assertm(instance.get_map_data().is_in(location), "Invalid location.");

  int                            free_location = instance.location_to_free_location(location);
  const std::list<TimeInterval>& cur_TI_list   = safe_intervals[free_location];

  // there is no safe interval
  if (cur_TI_list.empty())
  {
    return cur_TI_list.cend();
  }

  return cur_TI_list.cbegin();
}



// SIPPS: get safe intervals allowing soft collisions
void SafeIntervalTable::get_soft_safe_intervals(int from, int location, const TimeInterval& window, std::vector<TimeInterval>& output_si) const
{
  output_si.clear();
  const std::list<CIWithCount>& collision_intervals = ci_list_for_cell[instance.location_to_free_location(location)];

  for (const CIWithCount& ci : collision_intervals){
    // extend the previous collision interval with an adjacent occupied segment
    if (!output_si.empty() && (ci.agent_count > 0) && output_si.back().col){
      output_si.back().t_max = ci.t_max;
      continue;
    }

    // the previous interval is complete; drop it if it ends before the window
    if (!output_si.empty() && output_si.back().t_max < window.t_min){
      output_si.pop_back();
    }

    // nothing further can overlap the window
    if (ci.t_min > window.t_max){
      break;
    }

    output_si.push_back(TimeInterval(ci.t_min, ci.t_max, (ci.agent_count > 0)));
  }

  if (!output_si.empty() && output_si.back().t_max < window.t_min){
    output_si.pop_back();
  }

  // an agent that does not move can not swap places with another agent
  if (from == location){
    return;
  }

  // handling edge collisions
  for (int i = 0; i < (int)output_si.size(); i++)
  {
    // arriving in a colliding interval already counts one collision, a swap is not counted on top
    if (output_si[i].col){
      continue;
    }

    // the arrival times into this interval allowed by the window
    const int arrival_min = std::max(output_si[i].t_min, window.t_min);
    const int arrival_max = std::min(output_si[i].t_max, window.t_max);

    // find the first arrival time without a swap
    int arrival_no_edge_col = -1;
    for (int t = arrival_min; t <= arrival_max; t++){
      if (!edge_constraint_table.get(location, from, t)){
        arrival_no_edge_col = t;
        break;
      }
    }

    // no swap at the earliest arrival: the interval stays as it is
    if (arrival_no_edge_col == arrival_min){
      continue;
    }

    // the earliest arrival swaps places with another agent
    output_si[i].edge_col = true;

    // every possible arrival swaps: there is no other option
    if (arrival_no_edge_col == -1){
      continue;
    }

    // otherwise also add the option to arrive later without the swap (its edge_col stays false).
    TimeInterval no_swap(arrival_no_edge_col, output_si[i].t_max);
    output_si.insert(output_si.begin() + i + 1, no_swap);
    i++;
  }
}

// SIPPS: number of collisions if an agent stayed at the location forever after the given time,
auto SafeIntervalTable::get_future_collisions(int location, int time) const -> int
{
  const std::list<CIWithCount>& collision_intervals = ci_list_for_cell[instance.location_to_free_location(location)];

  int future_collisions = 0;
  for (const CIWithCount& ci : collision_intervals)
  {
    // skip intervals that end before the time and intervals without other agents
    if (ci.t_max <= time || ci.agent_count == 0)
    {
      continue;
    }
    assertm(ci.t_max != INT_MAX, "Vertex already forever occupied by another agent.");

    const int first_step = std::max(ci.t_min, time + 1);
    future_collisions += ci.agent_count * (ci.t_max - first_step + 1);
  }
  return future_collisions;
}

auto SafeIntervalTable::get_safe_intervals(int location, const TimeInterval& time_interval) const
    -> std::pair<std::list<TimeInterval>::const_iterator, std::list<TimeInterval>::const_iterator>
{
  // this implementation does not check edge collisions, so they must be handled elsewhere.
  assertm(instance.get_map_data().is_in(location), "Invalid location.");
  int free_location = instance.location_to_free_location(location);
  assertm(free_location < static_cast<int>(safe_intervals.size()), "Safe intervals not precomputed for this location.");

  // iterate over the list to find the first and last safe interval
  auto start = safe_intervals[free_location].cend();
  auto end   = safe_intervals[free_location].cend();
  for (auto it = safe_intervals[free_location].cbegin(); it != safe_intervals[free_location].cend(); it++)
  {
    // check whether the interval is inside the given time range
    if (it->t_max >= time_interval.t_min && it->t_min <= time_interval.t_max)
    {
      // remember the first interval in the range
      if (start == safe_intervals[free_location].end())
      {
        start = it;
      }
    }
    // if the interval is not inside the time range, check whether already found some intervals
    else if (start != safe_intervals[free_location].cend())
    {
      // if already found some intervals, return the found safe intervals
      end = it;
      return std::make_pair(start, end);
    }
  }
  return std::make_pair(start, end);
}

void SafeIntervalTable::update_latest_constraint_end_estimate()
{
  latest_constraint_end = 0;
  // TODO iterate only for last intervals
  for (const auto& safe_interval_list : safe_intervals)
  {
    const auto last = safe_interval_list.back();

    // safe interval reaches the end
    if (last.t_max == INT_MAX)
    {
      latest_constraint_end = std::max(latest_constraint_end, last.t_min - 1);
    }
  }

  latest_constraint_end_updated = true;
}

[[nodiscard]] auto SafeIntervalTable::get_max_path_len_estimate() -> int
{
  if (unlimited_safe_intervals == 0)
  {
    return INT_MAX;
  }

  if (!latest_constraint_end_updated)
  {
    update_latest_constraint_end_estimate();
  }
  // check for overflow
  if (latest_constraint_end <= INT_MAX - unlimited_safe_intervals)
  {
    return latest_constraint_end + unlimited_safe_intervals;
  }
  return INT_MAX;
}

EdgeConstraintTable::EdgeConstraintTable(const Instance& instance_) : instance(instance_)
{
  // initialize the edge constraints
  edge_constraints.resize(NUM_DIRECTIONS);
  for (int i = 0; i < NUM_DIRECTIONS; i++)
  {
    edge_constraints[i].resize(instance.get_num_free_cells());
  }
}

void EdgeConstraintTable::add(int loc1, int loc2, int time)
{
  assertm(loc1 != loc2, "Invalid edge constraint.");
  const int dir = static_cast<int>(find_direction(loc1, loc2)) - 1;
  assertm(dir >= 0 && dir <= NUM_DIRECTIONS, "Invalid direction.");

  int loc2_free = instance.location_to_free_location(loc2);
  // TODO: This should ideally still apply when we don't have soft collisions allowed
  // assertm(std::find(edge_constraints[dir][loc2_free].begin(), edge_constraints[dir][loc2_free].end(), time) ==
  //             edge_constraints[dir][loc2_free].end(),
  //         "Edge constraint already exists.");
 
  edge_constraints[dir][loc2_free].push_back(time);  // vector implementation
}

void EdgeConstraintTable::remove(int loc1, int loc2, int time)
{
  const int dir = static_cast<int>(find_direction(loc1, loc2)) - 1;
  assertm(dir >= 0 && dir <= NUM_DIRECTIONS, "Invalid direction.");

  int loc2_free = instance.location_to_free_location(loc2);
  std::vector<int> &edge_cons_for_loc2 = edge_constraints[dir][loc2_free];
  auto edge_cons_to_remove = std::find(edge_cons_for_loc2.begin(), edge_cons_for_loc2.end(), time);
  assertm(edge_cons_to_remove != edge_cons_for_loc2.end(), "Did not find edge constraint in edge cons list!");
  edge_cons_for_loc2.erase(edge_cons_to_remove);
  // the following erases all:
  // edge_constraints[dir][loc2_free].erase(
  //     std::remove(edge_constraints[dir][loc2_free].begin(), edge_constraints[dir][loc2_free].end(), time),
  //     edge_constraints[dir][loc2_free].end());
}


void SafeIntervalTable::build_sequential(const std::vector<TimePointPath>& paths)
{
  assertm(static_cast<int>(paths.size()) > 0, "No paths to add.");
  for (const auto& path : paths)
  {
    add_constraints(path);
  }
}
