/*
 * Author: Jan Chleboun
 * Date: 14-02-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#include "ConstraintTable.h"

#include <boost/functional/hash.hpp>
#include <climits>
#include <iostream>

constexpr int NUM_DIRECTIONS = magic_enum::enum_count<Direction>() - 1;  // dont consider NONE direction
                                                                         //
ConstraintTable::ConstraintTable(const Instance& instance_)
    : edge_constraint_table(instance_),
      instance(instance_)
{
  // initialize the constraint vectors, static obstacles dont need constraint vectors
  const int num_free_cells = instance.get_num_free_cells();
  constraints   = std::vector<std::list<std::pair<TimeInterval, int>>>(num_free_cells, std::list<std::pair<TimeInterval, int>>());
  agents_counts = std::vector<std::unordered_map<int, int>>(num_free_cells, std::unordered_map<int, int>());
}

void ConstraintTable::add_constraint(const TimePoint& timepoint, int agent_num)
{
  // check the location is valid
  assertm(instance.get_map_data().is_in(timepoint.location), "Invalid location.");
  assertm(agent_num >= 0 && agent_num < instance.get_num_of_agents(), "Invalid agent number.");
  int                                      free_location = instance.location_to_free_location(timepoint.location);
  std::list<std::pair<TimeInterval, int>>& cur_TI_list   = constraints[free_location];

  bool inserted = false;
  // find where to insert the interval
  for (auto it = cur_TI_list.begin(); it != cur_TI_list.end(); it++)
  {
    // check that the intervals do not have any overlap
    // Special exception for SIPPS: overlap may exist if 

    assertm(!overlap((*it).first, timepoint.interval), "Cannot add overlapping constraints without enabling SIPPS (soft collisions).");

    // insert before the first interval that starts later
    if (it->first.t_min > timepoint.interval.t_min)
    {
      std::pair<TimeInterval, int> new_cons = {timepoint.interval, agent_num};
      cur_TI_list.insert(it, new_cons);
      inserted = true;
      break;
    }
  }

  // if no interval starts later, insert to the back of the list
  if (!inserted)
  {
    std::pair<TimeInterval, int> new_cons = {timepoint.interval, agent_num};
    cur_TI_list.emplace_back(new_cons);
  }

  // add to the set of visiting agents
  // if the agent is not yet in the set, add it with count 1
  if (agents_counts[free_location].find(agent_num) == agents_counts[free_location].end())
  {
    agents_counts[free_location][agent_num] = 1;
  }
  else
  {
    // else increase the count
    agents_counts[free_location][agent_num] += 1;
  }
}

void ConstraintTable::remove_constraint(const TimePoint& timepoint, int agent_num)
{
  // check the location is valid
  assertm(instance.get_map_data().is_in(timepoint.location), "Invalid location.");
  int free_location = instance.location_to_free_location(timepoint.location);


  std::list<std::pair<TimeInterval, int>>& cur_TI_list = constraints[free_location];

  // check that the list is not empty
  assertm(!cur_TI_list.empty(), "Trying to remove interval from an empty list.");

  assertm(agents_counts[free_location].find(agent_num) != agents_counts[free_location].end(),
          "Removing agent that is not in the agents counts");
  agents_counts[free_location][agent_num] -= 1;
  // if the count drops to 0, erase the agent number from keys
  if (agents_counts[free_location][agent_num] == 0)
  {
    agents_counts[free_location].erase(agent_num);
  }

  // remove the interval from the list
  for (auto it = cur_TI_list.begin(); it != cur_TI_list.end(); it++)
  {
    // diferent handling for SIPPS (soft collisions)
    if ((*it).first == timepoint.interval )
    {
      cur_TI_list.erase(it);
      return;
    }
  }
  // if the interval was not found, throw an error
  throw std::runtime_error("Trying to remove non-existing interval from the constraint table.");
}


void ConstraintTable::add_constraints(const TimePointPath& path, int agent_num)
{
  for (int i = 0; i < (int)path.size(); i++)
  {
    // add the point constraint
    // std::cout << "Adding: " << path[i].location << ", " << path[i].interval << std::endl;
    add_constraint(path[i], agent_num);

    // add the edge constraint
    if (i > 0)  // skip the first point as there is no edge
    {
      // the time when the edge is used is the start time of the later interval
      edge_constraint_table.add(path[i - 1].location, path[i].location, path[i].interval.t_min, agent_num);
    }
  }
}



void ConstraintTable::remove_constraints(const TimePointPath& path, int agent_num)
{
  for (int i = 0; i < (int)path.size(); i++)
  {
    // remove the point constraint
    // std::cout << "Removing: " << path[i].location << ", " << path[i].interval << std::endl;
    remove_constraint(path[i], agent_num);

    // remove the edge constraint
    if (i > 0)  // skip the first point as there is no edge
    {
      edge_constraint_table.remove(path[i - 1].location, path[i].location, path[i].interval.t_min);
    }
  }
}


auto ConstraintTable::get_blocking_agent(int from, int to, int time) const -> std::pair<int, int>
{
  // check the location is valid
  assertm(instance.get_map_data().is_in(from) && instance.get_map_data().is_in(to), "Invalid location.");

  // check the time is valid
  assertm(time >= 0, "Invalid time.");

  int to_free = instance.location_to_free_location(to);

  const std::list<std::pair<TimeInterval, int>>& cur_TI_list = constraints[to_free];

  // if there are no constraints, return -1 (no agent blocking the square)
  if (cur_TI_list.empty())
  {
    return std::make_pair(-1, -1);
  };

  // find the constraint
  int vertex_constraint = -1;
  for (const auto& it : cur_TI_list)
  {
    const TimeInterval& constraint_tp = it.first;
    if (constraint_tp.t_min <= time && constraint_tp.t_max >= time)
    {
      // return the agent that created the constraint
      vertex_constraint = it.second;
      break;
    }
    // skip constraints that are too late
    if (constraint_tp.t_min > time)
    {
      break;
    }
  }

  // find edge constraint
  int edge_constraint = -1;
  if (from != to)
  {
    // we want to look for opposite edge, because there can not be any edge ending in to location without location constraint
    edge_constraint = edge_constraint_table.get(to, from, time);
  }
  return std::make_pair(vertex_constraint, edge_constraint);
}

auto ConstraintTable::get_blocking_agents(int location, int time_min) const -> std::vector<int>
{
  // check the location is valid
  assertm(instance.get_map_data().is_in(location), "Invalid location.");

  // check the time is valid
  assertm(time_min >= 0, "Invalid time.");

  int to_free = instance.location_to_free_location(location);

  const std::list<std::pair<TimeInterval, int>>& cur_TI_list = constraints[to_free];

  // if there are no constraints, return -1 (no agent blocking the square)
  assertm(!cur_TI_list.empty(), "Trying to get blocking agents from an empty list.");

  std::unordered_set<int> blocking_agents     = {};
  std::vector<int>        blocking_agents_vec = {};
  assertm(cur_TI_list.back().first.t_max == INT_MAX, "Last interval should be infinite.");

  // check how many intervals are present
  if (cur_TI_list.size() < 2)
  {
    return {};
  }

  // iterate over the constraints from end to beginning (skip the last interval, as agent can not block itself)
  for (auto it = std::next(cur_TI_list.rbegin()); it != cur_TI_list.rend(); it++)
  {
    // std::cout << "Interval: " << it->first << ", agent: " << it->second << std::endl;
    if (it->first.t_max < time_min)
    {
      // there can not be any more blocking agents
      break;
    }

    // check whether the agent was not already added
    const int agent_num = it->second;
    if (blocking_agents.find(agent_num) == blocking_agents.end())
    {
      // add the agent to the set
      blocking_agents.insert(agent_num);
      blocking_agents_vec.push_back(agent_num);
    }
  }
  return blocking_agents_vec;
}


void ConstraintTable::build_sequential(const std::vector<TimePointPath>& paths)
{
  assertm(static_cast<int>(paths.size()) > 0, "No paths to add.");
  for (int i = 0; i < static_cast<int>(paths.size()); i++)
  {
    add_constraints(paths[i], i);
  }
}


EdgeConstraintTableWithAgentNums::EdgeConstraintTableWithAgentNums(const Instance& instance_) : instance(instance_)
{
  // initialize the edge constraints
  edge_constraints.resize(NUM_DIRECTIONS);
  for (int i = 0; i < NUM_DIRECTIONS; i++)
  {
    edge_constraints[i].resize(instance.get_num_free_cells());
  }
}

void EdgeConstraintTableWithAgentNums::add(int loc1, int loc2, int time, int agent_num)
{
  assertm(loc1 != loc2, "Invalid edge constraint.");
  const int dir = static_cast<int>(find_direction(loc1, loc2)) - 1;
  assertm(dir >= 0 && dir <= NUM_DIRECTIONS, "Invalid direction.");
  // assertm(edge_constraints[dir][loc2].find(time) == edge_constraints[dir][loc2].end(), "Edge constraint already exists.");
  int loc2_free = instance.location_to_free_location(loc2);
  assertm(std::find_if(edge_constraints[dir][loc2_free].begin(), edge_constraints[dir][loc2_free].end(),
                       [time](const std::pair<int, int>& p) { return p.first == time; }) == edge_constraints[dir][loc2_free].end(),
          "Edge constraint already exists.");
  // edge_constraints[dir][loc2].insert(time);     // unordered set implementation
  edge_constraints[dir][loc2_free].push_back({time, agent_num});  // vector implementation
}

void EdgeConstraintTableWithAgentNums::remove(int loc1, int loc2, int time)
{
  const int dir = static_cast<int>(find_direction(loc1, loc2)) - 1;
  assertm(dir >= 0 && dir <= NUM_DIRECTIONS, "Invalid direction.");
  int loc2_free = instance.location_to_free_location(loc2);
// int erased = edge_constraints[dir][loc2].erase(time);
// assertm(erased == 1, "Trying to erase an edge constraint that is not present.");
#ifndef NDEBUG
  int constraints_before = edge_constraints[dir][loc2_free].size();
  // std::cout << "Constraints before: " << constraints_before << std::endl;
#endif
  edge_constraints[dir][loc2_free].erase(std::remove_if(edge_constraints[dir][loc2_free].begin(), edge_constraints[dir][loc2_free].end(),
                                                        [time](const std::pair<int, int>& p) { return p.first == time; }),
                                         edge_constraints[dir][loc2_free].end());
  // std::cout << "Constraints after: " << edge_constraints[dir][loc2].size() << std::endl;
  assertm(constraints_before - edge_constraints[dir][loc2_free].size() == 1, "Should erase exactly one edge constraint.");
}


auto EdgeConstraintTableWithAgentNums::get(int loc1, int loc2, int time) const -> int
{
  const int dir = static_cast<int>(find_direction(loc1, loc2)) - 1;  // dont consider NONE direction
  assertm(dir >= 0 && dir < NUM_DIRECTIONS, "Invalid direction.");

  int loc2_free = instance.location_to_free_location(loc2);
  // return edge_constraints[dir][loc2].find(time) != edge_constraints[dir][loc2].end(); // unordered set implementation
  auto it = std::find_if(edge_constraints[dir][loc2_free].begin(), edge_constraints[dir][loc2_free].end(),
                         [time](const std::pair<int, int>& p) { return p.first == time; });
  // check whether found
  if (it == edge_constraints[dir][loc2_free].end())
  {
    return -1;
  }
  return it->second;
}

