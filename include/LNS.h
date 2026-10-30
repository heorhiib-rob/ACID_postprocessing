/*
 * Author: Jan Chleboun
 * Date: 08-01-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#pragma once
#include <random>
#include <set>
#include <utility>

#include "ConstraintTable.h"
#include "SIPP.h"
#include "SharedData.h"
#include "Solver.h"

enum class DESTROY_TYPE
{
  RANDOM,
  RANDOMWALK,
  INTERSECTION,
  ADAPTIVE,
  RANDOM_CHOOSE,
  BLOCKED
};

enum class TERM_CONDITION
{
  ITERATIONS,
  TIMELIMIT,
  ITER_TIME,
  SUCCESS
};

class Destroy_settings
{
public:
  Destroy_settings(DESTROY_TYPE type_, int size_) : type(type_), size(size_)
  {
  }
  DESTROY_TYPE type;
  int          size;
};

class LNS_settings
{
public:
  LNS_settings(int max_iter_, double time_limit_, Destroy_settings destroy_settings_, SIPP_settings sipp_settings_, bool restarts_ = false)
      : max_iter(max_iter_),
        time_limit(time_limit_),
        destroy_settings(destroy_settings_),
        sipp_settings(sipp_settings_),
        restarts(restarts_)
  {
  }
  void set_term_condition(TERM_CONDITION terminating_condition_){terminating_condition = terminating_condition_;}
  int              max_iter;
  double           time_limit;
  TERM_CONDITION   terminating_condition = TERM_CONDITION::ITER_TIME;
  Destroy_settings destroy_settings;
  SIPP_settings    sipp_settings;
  bool             allow_soft_collisions = false;  // plan with SIPPS and only remove collisions (true), or improve cost with SIPP (false)
  bool             restarts;
};

class Operator
{
public:
  explicit Operator(std::function<void(Solution& sol)> func_) : func(std::move(func_))
  {
  }

  void apply(Solution& solution)
  {
    func(solution);
  }

private:
  const std::function<void(Solution& sol)> func;
};

struct Logger
{
  [[nodiscard]] auto get_used_operator_str() const -> std::vector<std::string>
  {
    std::vector<std::string> used_operator_str;
    for (const auto& it : used_operator)
    {
      // convert the enum to string
      used_operator_str.emplace_back(magic_enum::enum_name(it));
    }
    return used_operator_str;
  }

  std::vector<DESTROY_TYPE> used_operator;
  std::vector<int>          bsf_solution_cost;
  std::vector<int>          bsf_makespan;
  std::vector<double>       iteration_time_cpu;
  std::vector<double>       iteration_time_wall;
};

class LNS : public Solver
{
public:
  mutable std::vector<SIPPInfo> sipp_info;
  std::unique_ptr<SIPP>         planner;

  // constructor
  LNS(const Instance& instance_, std::mt19937& rnd_generator_, SharedData* shared_data_, LNS_settings& settings_);

  // Destructor
  ~LNS() override = default;

  // solve method
  void solve() override;

  void solveFromInitial() override;

  bool find_initial_solution();
  bool remove_collisions(const Clock& clock);

  Solution PrioritizedPlanning();

  [[nodiscard]] int get_num_of_generated_nodes() const
  {
    return planner->generated_all_iter;
  }
  [[nodiscard]] int get_num_of_expanded_nodes() const 
  {
    return planner->expanded_all_iter;
  }
  void discard_solution(const Solution& sol, const Solution& prev_sol) const;

  void initialize_constraint_table(const std::vector<TimePointPath>& paths);

  Operator                        destroy_operator;
  mutable std::unordered_set<int> already_planned;
  Logger                          log;
  LNS_settings&                   settings;
  bool                            found_initial_solution = false;

private:
  void destroy_random(Solution& sol) const;
  void destroy_randomwalk(Solution& sol) const;
  bool random_walk(int agent_num, int neighborhood_size, int start_location, int start_time, int upperbound, std::set<int>& chosen) const;
  void destroy_intersection(Solution& sol) const;
  void destroy_blocked(Solution& sol) const;
  void destroy_adaptive(Solution& sol) const;
  void destroy_random_choose(Solution& sol) const;
  void destroy_collision_graph(Solution& sol, const std::vector<std::set<int>>& collision_graph) const;
  void destroy_target(Solution& sol, const std::vector<std::set<int>>& collision_graph) const;
  void get_target_agents(int agent_num, const std::vector<int>& goal_owner, std::set<int>& target_agents) const;
  void destroy_random_colliding(Solution& sol, const std::vector<std::set<int>>& collision_graph) const;
  void get_intersection_agents(std::unordered_set<int>& neighborhood, int current) const;
  void repair_default(Solution& sol) const;

  SharedData*                 shared_data;
  int                         iteration_num = 0;
  double                      initial_solution_time;
  double                      curr_time;
  Operator                    repair_operator;
  mutable ConstraintTable     constraint_table;
  mutable bool                constraint_table_initialized = false;
  mutable std::vector<double> destroy_weights;
  double                      reaction_factor = 0.01;  // TODO this are hardocded from the MAPF-LNS2 code for now
  double                      decay_factor    = 0.01;
  std::vector<double>         collision_destroy_weights;  // weights of the destroy operators removing collisions: target, collision graph, random
  double                      collision_reaction_factor = 0.05;
  double                      collision_decay_factor    = 0.05;
  mutable DESTROY_TYPE        last_destroy_strategy;
  float                       threshold_blocked = 1.0;
};
