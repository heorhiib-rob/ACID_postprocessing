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
#include <unordered_set>
#include <vector>

#include "Instance.h"
#include "magic_enum/magic_enum.hpp"
#include "utils.h"

constexpr int NUM_DIRECTIONS = magic_enum::enum_count<Direction>() - 1;  // dont consider NONE direction

class EdgeConstraint
{
public:
  int from;
  int to;
  int t;

  EdgeConstraint(int from_, int to_, int t_) : from(from_), to(to_), t(t_)
  {
  }

  auto operator==(const EdgeConstraint& other) const -> bool;
};

class EdgeConstraintTable
{
public:
  bool allow_soft_collisions = true;
  explicit EdgeConstraintTable(const Instance& instance_);
  void add(int loc1, int loc2, int time);
  void remove(int loc1, int loc2, int time);
  // auto get(int loc1, int loc2, int time) const -> bool;

  [[nodiscard]] auto get(int loc1, int loc2, int time) const -> bool
  {
    const int dir = static_cast<int>(find_direction(loc1, loc2)) - 1;  // dont consider NONE direction
    assertm(dir >= 0 && dir < NUM_DIRECTIONS, "Invalid direction.");
    int loc2_free = instance.location_to_free_location(loc2);
    // return edge_constraints[dir][loc2].find(time) != edge_constraints[dir][loc2].end(); // unordered set implementation
    return std::find(edge_constraints[dir][loc2_free].begin(), edge_constraints[dir][loc2_free].end(), time) !=
           edge_constraints[dir][loc2_free].end();  // vector implementation
  }
  void reset()
  {
    edge_constraints.clear();
    edge_constraints.resize(NUM_DIRECTIONS);
    for (int i = 0; i < NUM_DIRECTIONS; i++)
    {
      edge_constraints[i].resize(instance.get_num_free_cells());
    }
  }

private:
  // std::unordered_set<EdgeConstraint, EdgeConstraint::hash_edge_collision> edge_constraints;  // constraints for edges
  // std::vector<std::vector<std::unordered_set<int>>> edge_constraints; // unordered set implementation
  std::vector<std::vector<std::vector<int>>> edge_constraints;  // vector implementation
  const Instance&                            instance;
};

struct CIWithCount {
  int t_min;
  int t_max;
  int agent_count;

  CIWithCount(int t_min_, int t_max_){
    t_min = t_min_;
    t_max = t_max_;
    agent_count = 0;
  }

};

class SafeIntervalTable
{
public:
  explicit SafeIntervalTable(const Instance& instance_);
  EdgeConstraintTable edge_constraint_table;
  bool                allow_soft_collisions = false;  // store the paths for SIPPS (true) or for classical SIPP (false)

  void               add_constraint(const TimePoint& timepoint);
  void               add_constraints(const TimePointPath& path);
  void               remove_constraint(const TimePoint& timepoint);
  void               remove_constraints(const TimePointPath& path);
  [[nodiscard]] auto get_safe_intervals(int location, const TimeInterval& time_interval) const
      -> std::pair<std::list<TimeInterval>::const_iterator, std::list<TimeInterval>::const_iterator>;
  [[nodiscard]] auto get_first_safe_interval(int location) const -> std::list<TimeInterval>::const_iterator;
  [[nodiscard]] auto get_max_path_len_estimate() -> int;
  void               update_latest_constraint_end_estimate();
  void               build_sequential(const std::vector<TimePointPath>& paths);


  void get_soft_safe_intervals(int from, int location, const TimeInterval& window, std::vector<TimeInterval>& result) const; //for SIPPS
  [[nodiscard]] auto get_future_collisions(int location, int time) const -> int; //for SIPPS

  [[nodiscard]] auto get_min_reach_time(int goal) const -> int
  {
    assertm(instance.get_map_data().is_in(goal), "Invalid goal location.");
    return safe_intervals[instance.location_to_free_location(goal)].back().t_min;
  }

  [[nodiscard]] auto get_last_safe_interval(int location) const -> const TimeInterval&
  {
    assertm(instance.get_map_data().is_in(location), "Invalid location");
    return safe_intervals[instance.location_to_free_location(location)].back();
  }

  void reset()
  {
    // reset safe intervals
    int num_free_cells = static_cast<int>(safe_intervals.size());
    safe_intervals     = std::vector<std::list<TimeInterval>>(num_free_cells, std::list<TimeInterval>(1, TimeInterval(0, INT_MAX)));
    ci_list_for_cell = std::vector<std::list<CIWithCount>>(num_free_cells, std::list<CIWithCount>(1, CIWithCount(0, INT_MAX)));

    // reset latest constraint end
    latest_constraint_end         = 0;
    latest_constraint_end_updated = true;

    // reset unlimited safe intervals
    unlimited_safe_intervals = instance.get_map_data().get_num_free_cells();

    // reset max path len estimate
    max_path_len_estimate = INT_MAX;

    // reset edge constraint table
    edge_constraint_table.reset();
  }



  // functions for SIPPS

  std::list<CIWithCount>::iterator split_at(std::list<CIWithCount>& list, int t){
    std::list<CIWithCount>::iterator it = list.begin();
    // skip all intervals ending earlier
    while(it->t_max < t){
      it++;
    }

    // if we find an interval beginning exactly at t, we have the one we want
    if(it->t_min == t){
      return it;
    }

    // else, we split the interval and return an iterator to   
    CIWithCount right_subint(t, it->t_max);
    right_subint.agent_count = it->agent_count;
    it->t_max = t-1;
    return list.insert(std::next(it), right_subint);

  }

  void change_ci_count(int location, int first_occupied_timestep, int last_occupied_timestep, int increment){
    std::list<CIWithCount> &CI_list =  ci_list_for_cell[instance.location_to_free_location(location)];

    auto first_affected_interval = split_at(CI_list, first_occupied_timestep); // either get SI corresponding to the first occupied timestep
                                                             // or split an existing SI and get the new one
    auto last_affected_interval = CI_list.end();

    if(last_occupied_timestep != INT_MAX) {
      // not in goal
      last_affected_interval = split_at(CI_list, last_occupied_timestep + 1); // + 1 because 
    }

    for(auto it = first_affected_interval; it != last_affected_interval; it++){
      it->agent_count += increment;
      assertm(it->agent_count >= 0, "Agent count in CI has to be larger or equal to zero");
    }

    if(last_affected_interval != CI_list.end() && std::prev(last_affected_interval)->agent_count == last_affected_interval->agent_count){
      std::prev(last_affected_interval)-> t_max = last_affected_interval->t_max;
      CI_list.erase(last_affected_interval);
    }

    if(first_affected_interval != CI_list.begin() && std::prev(first_affected_interval)->agent_count == first_affected_interval->agent_count){
      std::prev(first_affected_interval)->t_max = first_affected_interval->t_max;
      CI_list.erase(first_affected_interval);
    }
    
  }


private:
  std::vector<std::list<TimeInterval>> safe_intervals;  // safe intervals for each position
  const Instance&                      instance;
  int                                  max_path_len_estimate = INT_MAX;
  int                                  unlimited_safe_intervals;
  int                                  latest_constraint_end         = 0;
  bool                                 latest_constraint_end_updated = true;
  // a set of CIs to use for SIPPS
  std::vector<std::list<CIWithCount>>  ci_list_for_cell;
};
