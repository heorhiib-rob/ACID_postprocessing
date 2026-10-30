/*
 * Author: Jan Chleboun
 * Date: 14-02-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#pragma once

#include <climits>
#include <iostream>
#include <list>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "Instance.h"
#include "magic_enum/magic_enum.hpp"
#include "utils.h"


class EdgeConstraintWithAgentNum
{
public:
  int from;
  int to;
  int t;
  int agent_num;

  EdgeConstraintWithAgentNum(int from_, int to_, int t_, int agent_num_) : from(from_), to(to_), t(t_), agent_num(agent_num_)
  {
  }

  auto operator==(const EdgeConstraintWithAgentNum& other) const -> bool;
};

class EdgeConstraintTableWithAgentNums
{
public:
  explicit EdgeConstraintTableWithAgentNums(const Instance& instance);
  void add(int loc1, int loc2, int time, int agent_num);
  void remove(int loc1, int loc2, int time);
  // auto get(int loc1, int loc2, int time) const -> bool;

  [[nodiscard]] auto get(int loc1, int loc2, int time) const -> int;

private:
  // std::unordered_set<EdgeConstraint, EdgeConstraint::hash_edge_collision> edge_constraints;  // constraints for edges
  // std::vector<std::vector<std::unordered_set<int>>> edge_constraints; // unordered set implementation
  std::vector<std::vector<std::vector<std::pair<int, int>>>> edge_constraints;  // vector implementation TODO try set or something log
  const Instance&                                            instance;
};
class ConstraintTable
{
public:
  explicit ConstraintTable(const Instance& instance_);
  EdgeConstraintTableWithAgentNums edge_constraint_table;

  void build_sequential(const std::vector<TimePointPath>& paths); // timepoint is actually a location + interval
  void               add_constraint(const TimePoint& timepoint, int agent_num);
  void               add_constraints(const TimePointPath& path, int agent_num);
  void               remove_constraint(const TimePoint& timepoint, int agent_num);
  void               remove_constraints(const TimePointPath& path, int agent_num);
  [[nodiscard]] auto get_blocking_agent(int from, int to, int time) const -> std::pair<int, int>;
  [[nodiscard]] auto get_blocking_agents(int location, int time_min) const -> std::vector<int>;
  [[nodiscard]] auto get_agents_counts(int location) const -> const std::unordered_map<int, int>&
  {
    assertm(instance.get_map_data().is_in(location), "Invalid location.");
    return agents_counts[instance.location_to_free_location(location)];
  }

  [[nodiscard]] auto get_agents_counts_free(int free_location) const -> const std::unordered_map<int, int>&
  {
    assertm(instance.get_map_data().is_in(instance.free_location_to_location(free_location)), "Invalid location.");
    return agents_counts[free_location];
  }

  [[nodiscard]] auto get_last_constraint_start(int location) const -> int
  {
    return constraints[instance.location_to_free_location(location)].back().first.t_min;
  }

private:
  std::vector<std::list<std::pair<TimeInterval, int>>> constraints;  // constraints for each location
  bool                                                 allow_soft_collisions = true; 
  const Instance&                                      instance;
  std::vector<std::unordered_map<int, int>>            agents_counts;  // unordered map of times the agents visit each location
};
