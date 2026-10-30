/*
 * Author: Jan Chleboun
 * Date: 08-01-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#pragma once
#include <memory>
#include <random>
#include <set>
#include <string>
#include <vector>

#include "Instance.h"
#include "utils.h"

class Solution
// struct to store a solution of the mapf problem
{
public:
  bool feasible = true;

  // paths of the agents
  std::vector<TimePointPath> paths;
  mutable std::vector<Path>  converted_paths;
  std::vector<int>           delays;
  std::vector<int>           destroyed_paths;
  std::vector<int>           priorities;

  int cost;
  int sum_of_delays;  // cost - distance
  int sum_of_costs;
  int makespan;

  [[nodiscard]] auto is_valid(const Instance& instance) const -> bool;
  [[nodiscard]] auto get_collision_graph() const -> std::vector<std::set<int>>;  // for each agent, the agents it collides with
  void               load(const std::string& filename, const Instance& instance);
  void               save(const std::string& filename, const Instance& instance) const;
  void               calculate_cost(const Instance& instance);
  void               convert_paths() const;

  friend auto operator<<(std::ostream& os, Solution const& i) -> std::ostream&;

};

class Solver
// abstract class to represent a solver for the mapf problem
{
public:
  const std::string name;
  const Instance&   instance;

  // constructor
  Solver(std::string name_, const Instance& instance_, std::mt19937& rnd_generator_)
      : name(std::move(name_)), instance(instance_), rnd_generator(rnd_generator_)
  {
  }

  // virtual destructor
  virtual ~Solver() = default;

  // pure virtual function solve
  virtual void solve() = 0;

  virtual void solveFromInitial() = 0;

  void set_initial_solution(Solution solution_){ solution = solution_; };
  Solution      solution;  // the so far best solution
  std::mt19937& rnd_generator;
};
