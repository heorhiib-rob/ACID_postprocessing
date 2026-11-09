/*
 * Author: Jan Chleboun
 * Date: 08-02-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#include "LNS.h"

#include <numeric>
#include <queue>
#include <random>
#include <set>

#include "SIPP.h"
#include "utils.h"

#define MIN_BLOCKED_THRESHOLD 0.01
#define BLOCKED_REACTION_FACTOR 0.1


LNS::LNS(const Instance& instance_, std::mt19937& rnd_generator_, SharedData* shared_data_, LNS_settings& settings_)
    : Solver("LNS", instance_, rnd_generator_),
      destroy_operator(Operator([this](Solution& sol) {
        if (settings.destroy_settings.type == DESTROY_TYPE::RANDOMWALK)
        {
          destroy_randomwalk(sol);
        }
        else if (settings.destroy_settings.type == DESTROY_TYPE::INTERSECTION)
        {
          destroy_intersection(sol);
        }
        else if (settings.destroy_settings.type == DESTROY_TYPE::ADAPTIVE)
        {
          destroy_adaptive(sol);
        }
        else if (settings.destroy_settings.type == DESTROY_TYPE::RANDOM_CHOOSE)
        {
          destroy_random_choose(sol);
        }
        else if (settings.destroy_settings.type == DESTROY_TYPE::BLOCKED)
        {
          destroy_blocked(sol);
        }
        else
        {
          destroy_random(sol);
        }
      })),
      settings(settings_),
      shared_data(shared_data_),
      repair_operator(Operator([this](Solution& sol) { repair_default(sol); })),
      constraint_table(instance)
{
  // SIPP's lifetime can not be longer than LNS's lifetime, because then map_data might be deleted before SIPP
  planner = std::make_unique<SIPP>(instance, rnd_generator, settings.sipp_settings);
  planner->safe_interval_table.allow_soft_collisions = settings.allow_soft_collisions;

  // initialize destroy weights for adaptive LNS to 1
  if (settings.destroy_settings.type == DESTROY_TYPE::ADAPTIVE)
  {
    // the number of destroy operator is 1 less than destoroy types, because ADAPTIVE is not an operator
    // destroy_weights.assign(magic_enum::enum_count<DESTROY_TYPE>() - 1, 1);
    destroy_weights.assign(3, 1);  // RANDOM, RANDOMWALK, INTERSECTION
  }
}


void LNS::initialize_constraint_table(const std::vector<TimePointPath>& paths)
{
  assertm(!constraint_table_initialized, "Constraint table already initialized");
  constraint_table_initialized = true;
  // build the constraint table
  // Clock time_ct_build;
  // time_ct_build.start();
  constraint_table.build_sequential(solution.paths);
  // constraint_table.build_parallel(solution.paths);
  // constraint_table.build_parallel_2(solution.paths);
  // auto [time_wall, time_cpu] = time_ct_build.end();
  // std::cout << "Building the constraint_table took: " << time_wall << " s wall time and " << time_cpu << " s cpu_time" << std::endl;
}

void LNS::solveFromInitial()
{
  found_initial_solution = true;

  // store the paths of the initial solution in the planner
  planner->reset();
  planner->safe_interval_table.build_sequential(solution.paths);
  already_planned.clear();
  for (int i = 0; i < instance.get_num_of_agents(); i++)
  {
    already_planned.insert(i);
  }

  // the time limit was already decreased by the time of the first phase
  solve();
}

void LNS::solve()
{
  // start measuring time
  Clock clock;
  clock.start();

  // calculate initial solution
  while (!found_initial_solution && clock.get_current_time().first < settings.time_limit)
  {
    found_initial_solution = find_initial_solution();

    // reset the safe interval table and already planned if needed
    if (!found_initial_solution)
    {
      planner->reset();
      already_planned.clear();
    }

    // restart only if the settings allow it
    if (!settings.restarts)
    {
      break;
    }
  }

  // with soft collisions, only remove the collisions of the initial solution
  if (settings.allow_soft_collisions)
  {
    if (found_initial_solution)
    {
      found_initial_solution = remove_collisions(clock);
    }
    return;
  }

  if (!found_initial_solution || settings.max_iter == 0)
  {
    return;
  }
  assertm(found_initial_solution, "Could not find the initial solution");
  assertm(solution.is_valid(instance), "Found invalid solution");

  // std::cout << "Initial solution found in " << initial_solution_time << " second." << std::endl;

  // initialize the constraint table (needed for randomwalk and intersection destroy operator)
  if (settings.destroy_settings.type != DESTROY_TYPE::RANDOM)
  {
    initialize_constraint_table(solution.paths);
  }

  // improve the solution
  // TODO: use TERM_CONDITION to determine the terminating condition
  while (iteration_num < settings.max_iter && clock.get_current_time().first < settings.time_limit)
  {
    // check for visualization thread end
    if (settings.sipp_settings.info_type == INFO_type::visualisation)
    {
      if (shared_data->is_end.load(std::memory_order_acquire))
      {
        break;
      }
    }

    iteration_num++;

    // time the iteration
    Clock iteration_clock;
    iteration_clock.start();

    // copy solution
    Solution perturbed_sol = solution;

    // destroy operator
    std::cout << "LNS: Calling destroy" << std::endl;
    destroy_operator.apply(perturbed_sol);
    std::cout << "LNS: Destroy complete" << std::endl;

    // repair operator
    std::cout << "LNS: Calling repair" << std::endl;
    repair_operator.apply(perturbed_sol);
    std::cout << "LNS: Repair complete" << std::endl;

    // accept only feasible solutions
    bool accepted    = false;
    int  improvement = 0;
    if (!perturbed_sol.feasible)
    {
      discard_solution(perturbed_sol, solution);
      // std::cout << "Discarded infeasible solution." << std::endl;
    }
    else
    {
      // accept only improving solutions
      perturbed_sol.calculate_cost(instance);
      improvement = solution.sum_of_delays - perturbed_sol.sum_of_delays;

      // find out which destroy strategy was used
      auto destroy_strategy_index_help = magic_enum::enum_index<DESTROY_TYPE>(last_destroy_strategy);
      int  destroy_strategy_index      = -1;
      if (destroy_strategy_index_help.has_value())
      {
        destroy_strategy_index = destroy_strategy_index_help.value();
      }
      assertm(destroy_strategy_index >= 0 && destroy_strategy_index < static_cast<int>(magic_enum::enum_count<DESTROY_TYPE>()),
              "Wrong destroy strategy index.");

      if (improvement <= 0)
      {
        // update the weights
        if (settings.destroy_settings.type == DESTROY_TYPE::ADAPTIVE)
        {
          destroy_weights[destroy_strategy_index] = (1 - BLOCKED_REACTION_FACTOR) * destroy_weights[destroy_strategy_index];
        }
        if (last_destroy_strategy == DESTROY_TYPE::BLOCKED)
        {
          threshold_blocked = std::max((1 - BLOCKED_REACTION_FACTOR) * threshold_blocked, MIN_BLOCKED_THRESHOLD);
        }
        discard_solution(perturbed_sol, solution);
        // std::cout << "Discarded worse solution, difference: " << perturbed_sol.sum_of_costs - solution.sum_of_costs << std::endl;
      }
      else
      {
        // update the weights
        if (settings.destroy_settings.type == DESTROY_TYPE::ADAPTIVE)
        {
          destroy_weights[destroy_strategy_index] =
              reaction_factor * static_cast<double>(improvement) / static_cast<double>(settings.destroy_settings.size) +
              (1 - reaction_factor) * destroy_weights[destroy_strategy_index];
        }
        else if (last_destroy_strategy == DESTROY_TYPE::BLOCKED)
        {
          threshold_blocked = std::min((1 + reaction_factor) * threshold_blocked, 1.0);
        }
        accepted = true;
        // remember the perturbed solution
        // std::cout << "Solution improved by: " << solution.sum_of_delays - perturbed_sol.sum_of_delays << std::endl;
        solution = std::move(perturbed_sol);
      }
    }
    auto [iteration_time_wall, iteration_time_cpu] = iteration_clock.end();

    // construct LNS iteration info and add to the shared data
    if (settings.sipp_settings.info_type == INFO_type::visualisation)
    {
      // get the pointer to the original perturbed solution (which might have been moved)
      Solution* sol = &solution;
      if (!accepted)
      {
        sol = &perturbed_sol;
      }
      LNSIterationInfo lns_info(iteration_num, accepted, improvement, sipp_info, *sol,
                                (std::string)magic_enum::enum_name<DESTROY_TYPE>(last_destroy_strategy));
      sipp_info.clear();
      shared_data->update_lns_info(lns_info);
    }
    // log the iteration
    else if (settings.sipp_settings.info_type == INFO_type::experiment)
    {
      log.bsf_solution_cost.push_back(solution.sum_of_costs);
      log.bsf_makespan.push_back(solution.makespan);
      log.used_operator.push_back(last_destroy_strategy);
      log.iteration_time_wall.push_back(iteration_time_wall);
      log.iteration_time_cpu.push_back(iteration_time_cpu);
    }
  }
}

auto LNS::find_initial_solution() -> bool
{
  Clock clock;
  clock.start();

  // create initial solution
  solution = PrioritizedPlanning();
  auto [init_sol_time_wall, init_sol_time_cpu] = clock.get_current_time();
  solution.calculate_cost(instance);

  // construct LNS iteration info and add to the shared data
  if (settings.sipp_settings.info_type == INFO_type::visualisation)
  {
    LNSIterationInfo lns_info(0, solution.feasible, 0, sipp_info, solution, "None");
    sipp_info.clear();
    shared_data->update_lns_info(lns_info);
  }
  // log the experiment data
  else if (settings.sipp_settings.info_type == INFO_type::experiment)
  {
    log.bsf_solution_cost.push_back(solution.sum_of_costs);
    log.bsf_makespan.push_back(solution.makespan);
    log.used_operator.push_back(DESTROY_TYPE::ADAPTIVE);
    log.iteration_time_wall.push_back(init_sol_time_wall);
    log.iteration_time_cpu.push_back(init_sol_time_cpu);
  }

  std::cout << "Printing initial solution: " << std::endl;
  std::cout << solution << std::endl;

  // std::cout << "Solution has sum of costs: " << solution.sum_of_costs << ", sol has delay: " << solution.sum_of_delays << std::endl;
  return solution.feasible;
}

auto LNS::remove_collisions(const Clock& clock) -> bool
{
  // count the colliding pairs of agents
  std::vector<std::set<int>> collision_graph     = solution.get_collision_graph();
  int                        num_colliding_pairs = 0;
  for (int i = 0; i < (int)collision_graph.size(); i++)
  {
    num_colliding_pairs += (int)collision_graph[i].size();
  }
  num_colliding_pairs /= 2;  // each pair was counted for both agents

  // initialize the weights of the destroy operators to 1
  collision_destroy_weights.assign(3, 1);

  // replan colliding agents until there are no collisions
  while (num_colliding_pairs > 0 && clock.get_current_time().first < settings.time_limit)
  {
    // copy solution
    Solution perturbed_sol = solution;

    // choose the destroy operator by roulette wheel selection
    const double sum_of_weights = collision_destroy_weights[0] + collision_destroy_weights[1] + collision_destroy_weights[2];
    std::uniform_real_distribution<double> dist(0.0, sum_of_weights);
    const double                           rand_value = dist(rnd_generator);
    int                                    chosen     = 2;
    if (rand_value <= collision_destroy_weights[0])
    {
      chosen = 0;
    }
    else if (rand_value <= collision_destroy_weights[0] + collision_destroy_weights[1])
    {
      chosen = 1;
    }

    // destroy
    if (chosen == 0)
    {
      destroy_target(perturbed_sol, collision_graph);
    }
    else if (chosen == 1)
    {
      destroy_collision_graph(perturbed_sol, collision_graph);
    }
    else
    {
      destroy_random_colliding(perturbed_sol, collision_graph);
    }

    // no need to replan if none of the destroyed agents collides
    bool destroyed_colliding = false;
    for (int agent : perturbed_sol.destroyed_paths)
    {
      if (!collision_graph[agent].empty())
      {
        destroyed_colliding = true;
      }
    }
    if (!destroyed_colliding)
    {
      collision_destroy_weights[chosen] = (1 - collision_decay_factor) * collision_destroy_weights[chosen];
      continue;
    }

    // replan the destroyed agents
    repair_operator.apply(perturbed_sol);
    if (!perturbed_sol.feasible)
    {
      collision_destroy_weights[chosen] = (1 - collision_decay_factor) * collision_destroy_weights[chosen];
      discard_solution(perturbed_sol, solution);
      continue;
    }

    // count the colliding pairs of the perturbed solution
    std::vector<std::set<int>> perturbed_collision_graph     = perturbed_sol.get_collision_graph();
    int                        perturbed_num_colliding_pairs = 0;
    for (int i = 0; i < (int)perturbed_collision_graph.size(); i++)
    {
      perturbed_num_colliding_pairs += (int)perturbed_collision_graph[i].size();
    }
    perturbed_num_colliding_pairs /= 2;

    // update the weight of the chosen destroy operator
    if (perturbed_num_colliding_pairs < num_colliding_pairs)
    {
      collision_destroy_weights[chosen] = collision_reaction_factor * (num_colliding_pairs - perturbed_num_colliding_pairs) +
                                          (1 - collision_reaction_factor) * collision_destroy_weights[chosen];
    }
    else
    {
      collision_destroy_weights[chosen] = (1 - collision_decay_factor) * collision_destroy_weights[chosen];
    }

    // accept solutions that do not have more colliding pairs
    if (perturbed_num_colliding_pairs <= num_colliding_pairs)
    {
      solution            = std::move(perturbed_sol);
      collision_graph     = std::move(perturbed_collision_graph);
      num_colliding_pairs = perturbed_num_colliding_pairs;
    }
    else
    {
      discard_solution(perturbed_sol, solution);
    }
  }

  // time limit reached before all collisions were removed
  if (num_colliding_pairs > 0)
  {
    solution.feasible = false;
    return false;
  }

  solution.calculate_cost(instance);
  return true;
}

void LNS::destroy_target(Solution& sol, const std::vector<std::set<int>>& collision_graph) const
{
  const int neighborhood_size = settings.destroy_settings.size;
  assertm(neighborhood_size >= 2, "Invalid neighborhood size.");

  // choose a colliding agent, agents with more collisions are more likely to be chosen
  int num_collisions = 0;
  for (int i = 0; i < (int)collision_graph.size(); i++)
  {
    num_collisions += (int)collision_graph[i].size();
  }
  std::uniform_int_distribution<int> dist_collision(0, num_collisions - 1);
  const int                          rand_collision = dist_collision(rnd_generator);
  int                                agent          = -1;
  int                                sum            = 0;
  for (int i = 0; i < (int)collision_graph.size(); i++)
  {
    sum += (int)collision_graph[i].size();
    if (rand_collision < sum)
    {
      agent = i;
      break;
    }
  }
  assertm(agent >= 0 && !collision_graph[agent].empty(), "No colliding agent chosen.");

  // the agent whose goal is at each location, -1 if there is none
  std::vector<int> goal_owner(instance.get_num_cells(), -1);
  for (int i = 0; i < instance.get_num_of_agents(); i++)
  {
    goal_owner[instance.get_goal_locations()[i]] = i;
  }

  // other agents that visit the start location of the agent, ordered by the time of the visit
  sol.convert_paths();
  const int                     start = instance.get_start_locations()[agent];
  std::set<std::pair<int, int>> start_agents;  // (time, agent)
  for (int j = 0; j < (int)sol.converted_paths.size(); j++)
  {
    if (j == agent)
    {
      continue;
    }
    for (int t = 0; t < (int)sol.converted_paths[j].size(); t++)
    {
      if (sol.converted_paths[j][t] == start)
      {
        start_agents.insert(std::make_pair(t, j));
      }
    }
  }

  // other agents whose goals lie on a path of the agent that passes as few goals as possible
  std::set<int> target_agents;
  get_target_agents(agent, goal_owner, target_agents);

  std::set<int> chosen = {agent};
  if ((int)(start_agents.size() + target_agents.size()) >= neighborhood_size - 1)
  {
    if (start_agents.empty())
    {
      // choose random agents whose goals are on the path
      std::vector<int> shuffled_agents(target_agents.begin(), target_agents.end());
      std::shuffle(shuffled_agents.begin(), shuffled_agents.end(), rnd_generator);
      chosen.insert(shuffled_agents.begin(), shuffled_agents.begin() + neighborhood_size - 1);
    }
    else if ((int)target_agents.size() >= neighborhood_size)
    {
      // choose random agents whose goals are on the path and the first agent visiting the start location
      std::vector<int> shuffled_agents(target_agents.begin(), target_agents.end());
      std::shuffle(shuffled_agents.begin(), shuffled_agents.end(), rnd_generator);
      chosen.insert(shuffled_agents.begin(), shuffled_agents.begin() + neighborhood_size - 2);
      chosen.insert(start_agents.begin()->second);
    }
    else
    {
      // choose all agents whose goals are on the path, then the agents visiting the start location, earliest first
      chosen.insert(target_agents.begin(), target_agents.end());
      for (const std::pair<int, int>& start_agent : start_agents)
      {
        if ((int)chosen.size() >= neighborhood_size)
        {
          break;
        }
        chosen.insert(start_agent.second);
      }
    }
  }
  else if (!start_agents.empty() || !target_agents.empty())
  {
    // choose all of them
    chosen.insert(target_agents.begin(), target_agents.end());
    for (const std::pair<int, int>& start_agent : start_agents)
    {
      chosen.insert(start_agent.second);
    }

    // add agents whose goals lie on the paths of chosen agents, each chosen agent is tried at most once
    std::set<int> tried;
    while ((int)chosen.size() < neighborhood_size)
    {
      std::vector<int>                   chosen_vector(chosen.begin(), chosen.end());
      std::uniform_int_distribution<int> dist_chosen(0, (int)chosen_vector.size() - 1);
      const int                          chosen_agent = chosen_vector[dist_chosen(rnd_generator)];
      tried.insert(chosen_agent);
      if (tried.size() == chosen.size())
      {
        break;
      }

      std::vector<int> targets;
      for (int loc : sol.converted_paths[chosen_agent])
      {
        if (goal_owner[loc] >= 0)
        {
          targets.push_back(goal_owner[loc]);
        }
      }
      if (targets.empty())
      {
        continue;
      }
      std::uniform_int_distribution<int> dist_target(0, (int)targets.size() - 1);
      chosen.insert(targets[dist_target(rnd_generator)]);
    }
  }

  sol.destroyed_paths.assign(chosen.begin(), chosen.end());
  std::shuffle(sol.destroyed_paths.begin(), sol.destroyed_paths.end(), rnd_generator);
  assertm(sol.destroyed_paths.size() > 0, "No paths were destroyed.");
  sol.feasible = false;
}

void LNS::get_target_agents(int agent_num, const std::vector<int>& goal_owner, std::set<int>& target_agents) const
{
  const int start = instance.get_start_locations()[agent_num];
  const int goal  = instance.get_goal_locations()[agent_num];

  // search for the path that passes the fewest goals of other agents, among those the shortest one
  std::vector<int> best_num_goals(instance.get_num_cells(), INT_MAX);
  std::vector<int> best_length(instance.get_num_cells(), INT_MAX);
  std::vector<int> parent(instance.get_num_cells(), -1);

  // open list ordered by (number of goals passed, length, location), the smallest first
  std::set<std::tuple<int, int, int>> openlist;
  best_num_goals[start] = 0;
  best_length[start]    = 0;
  openlist.insert(std::make_tuple(0, 0, start));

  while (!openlist.empty())
  {
    // retrieve current node
    const auto [num_goals, length, current] = *openlist.begin();
    openlist.erase(openlist.begin());
    if (current == goal)
    {
      break;
    }

    for (int next : instance.get_neighbor_locations(current))
    {
      int next_num_goals = num_goals;
      if (goal_owner[next] >= 0 && next != goal)
      {
        next_num_goals++;
      }
      const int next_length = length + 1;

      // update the location if the new path to it is better
      if (next_num_goals < best_num_goals[next] || (next_num_goals == best_num_goals[next] && next_length < best_length[next]))
      {
        if (best_num_goals[next] != INT_MAX)
        {
          openlist.erase(std::make_tuple(best_num_goals[next], best_length[next], next));
        }
        best_num_goals[next] = next_num_goals;
        best_length[next]    = next_length;
        parent[next]         = current;
        openlist.insert(std::make_tuple(next_num_goals, next_length, next));
      }
    }
  }

  // collect the agents whose goals lie on the path
  for (int loc = goal; loc != -1; loc = parent[loc])
  {
    if (goal_owner[loc] >= 0 && loc != goal)
    {
      target_agents.insert(goal_owner[loc]);
    }
  }
}

void LNS::destroy_random_colliding(Solution& sol, const std::vector<std::set<int>>& collision_graph) const
{
  const int neighborhood_size = settings.destroy_settings.size;

  // if neighborhood size the same as number of agents or more, destroy all agents
  std::set<int> chosen;
  if (neighborhood_size >= instance.get_num_of_agents())
  {
    for (int i = 0; i < instance.get_num_of_agents(); i++)
    {
      chosen.insert(i);
    }
  }

  // choose random agents, each agent with weight 1 + the number of agents it collides with
  int sum_of_weights = 0;
  for (int i = 0; i < (int)collision_graph.size(); i++)
  {
    sum_of_weights += (int)collision_graph[i].size() + 1;
  }
  std::uniform_int_distribution<int> dist(0, sum_of_weights - 1);
  while ((int)chosen.size() < neighborhood_size)
  {
    const int rand_value = dist(rnd_generator);
    int       sum        = 0;
    for (int i = 0; i < (int)collision_graph.size(); i++)
    {
      sum += (int)collision_graph[i].size() + 1;
      if (rand_value < sum)
      {
        chosen.insert(i);
        break;
      }
    }
  }

  sol.destroyed_paths.assign(chosen.begin(), chosen.end());
  std::shuffle(sol.destroyed_paths.begin(), sol.destroyed_paths.end(), rnd_generator);
  assertm(sol.destroyed_paths.size() > 0, "No paths were destroyed.");
  sol.feasible = false;
}

void LNS::destroy_collision_graph(Solution& sol, const std::vector<std::set<int>>& collision_graph) const
{
  // choose a random agent that collides with another agent
  std::vector<int> colliding_agents;
  for (int i = 0; i < (int)collision_graph.size(); i++)
  {
    if (!collision_graph[i].empty())
    {
      colliding_agents.push_back(i);
    }
  }
  assertm(!colliding_agents.empty(), "No colliding agents to destroy.");
  std::uniform_int_distribution<int> dist_colliding(0, (int)colliding_agents.size() - 1);
  const int                          first_agent = colliding_agents[dist_colliding(rnd_generator)];

  // find all agents connected to it by collisions (breadth first search in the collision graph)
  std::set<int>   component = {first_agent};
  std::queue<int> openlist;
  openlist.push(first_agent);
  while (!openlist.empty())
  {
    const int current = openlist.front();
    openlist.pop();
    for (int other : collision_graph[current])
    {
      if (component.insert(other).second)
      {
        openlist.push(other);
      }
    }
  }

  std::set<int> chosen;
  if ((int)component.size() <= settings.destroy_settings.size)
  {
    // choose all the connected agents
    chosen = component;

    // add more agents found by random walks, stop after 10 unsuccessful walks
    int makespan = 0;
    for (int i = 0; i < (int)sol.paths.size(); i++)
    {
      makespan = std::max(makespan, sol.paths[i].back().interval.t_min);
    }
    const int neighborhood_size = std::min(settings.destroy_settings.size, instance.get_num_of_agents());
    int       failed_walks      = 0;
    while ((int)chosen.size() < neighborhood_size && failed_walks < 10)
    {
      // choose a random chosen agent
      std::vector<int>                   chosen_vector(chosen.begin(), chosen.end());
      std::uniform_int_distribution<int> dist_chosen(0, (int)chosen_vector.size() - 1);
      const int                          walk_agent = chosen_vector[dist_chosen(rnd_generator)];

      // start in its location at a random time
      std::uniform_int_distribution<int> dist_time(0, sol.paths[walk_agent].back().interval.t_min);
      int                                t   = dist_time(rnd_generator);
      int                                loc = location_at_time(sol.paths[walk_agent], t);

      // walk randomly until a location occupied by another agent is found
      int found_agent = -1;
      while (t <= makespan)
      {
        // other agents in the location at time t
        std::vector<int> agents_here;
        for (int i = 0; i < (int)sol.paths.size(); i++)
        {
          if (i != walk_agent && location_at_time(sol.paths[i], t) == loc)
          {
            agents_here.push_back(i);
          }
        }

        // choose one of them randomly
        if (!agents_here.empty())
        {
          std::uniform_int_distribution<int> dist_here(0, (int)agents_here.size() - 1);
          found_agent = agents_here[dist_here(rnd_generator)];
          break;
        }

        // move to a random neighbor or stay
        std::vector<int> next_locations = instance.get_neighbor_locations(loc);
        next_locations.push_back(loc);
        std::uniform_int_distribution<int> dist_next(0, (int)next_locations.size() - 1);
        loc = next_locations[dist_next(rnd_generator)];
        t++;
      }

      // only walks that find no agent count as unsuccessful
      if (found_agent < 0)
      {
        failed_walks++;
      }
      else
      {
        chosen.insert(found_agent);
      }
    }
  }
  else
  {
    // walk along the collisions from a random connected agent until enough agents are chosen
    std::vector<int>                   component_vector(component.begin(), component.end());
    std::uniform_int_distribution<int> dist_component(0, (int)component_vector.size() - 1);
    int                                current = component_vector[dist_component(rnd_generator)];
    chosen.insert(current);
    while ((int)chosen.size() < settings.destroy_settings.size)
    {
      std::vector<int>                   others(collision_graph[current].begin(), collision_graph[current].end());
      std::uniform_int_distribution<int> dist_other(0, (int)others.size() - 1);
      current = others[dist_other(rnd_generator)];
      chosen.insert(current);
    }
  }

  sol.destroyed_paths.assign(chosen.begin(), chosen.end());
  std::shuffle(sol.destroyed_paths.begin(), sol.destroyed_paths.end(), rnd_generator);
  assertm(sol.destroyed_paths.size() > 0, "No paths were destroyed.");
  sol.feasible = false;
}

auto LNS::PrioritizedPlanning() -> Solution
{
  Solution sol;
  sol.paths.resize(instance.get_num_of_agents(), TimePointPath());
  const int agent_num = instance.get_num_of_agents();

  // create a sequence from 0 to agent_num
  std::vector<int> priorities(agent_num);
  std::iota(priorities.begin(), priorities.end(), 0);

  // shuffle the sequence to get random priorities
  std::shuffle(priorities.begin(), priorities.end(), rnd_generator);

  // initialize iter info
  if (settings.sipp_settings.info_type == INFO_type::visualisation)
  {
    sipp_info.clear();
    sipp_info.resize(agent_num);
  }

  // plan path for each agent according to its priority
  assertm(already_planned.empty(), "Running Prioritized Planning when some agents were already planned.");
  for (int i = 0; i < agent_num; i++)
  {
    // get agent with priority i
    int agent_id = priorities[i];
    std::cout << "PP: planning for agent " << agent_id << std::endl;

    // plan path for agent
    TimePointPath tp_path = planner->plan(agent_id, already_planned);

    std::cout << "PP: planning finished" << std::endl;

    // get the visualization info
    if (settings.sipp_settings.info_type == INFO_type::visualisation)
    {
      sipp_info[agent_id] = std::move(planner->iter_info);
      planner->iter_info.clear();
    }

    // check validity
    if (tp_path.empty())
    {
      std::cout << "PP: Unable to find path. " << std::endl;
      sol.feasible = false;
      return sol;
    }

    // check validity
    assertm(instance.check_timepointpath_validity(tp_path), "SIPP planned an invalid timepointpath.");

    // update the safe interval table
    planner->safe_interval_table.add_constraints(tp_path);

    // add to the solution
    sol.paths[agent_id] = tp_path;

    // add to the already planned agents
    already_planned.insert(agent_id);
  }
  // add priorities to the visualization info
  if (settings.sipp_settings.info_type == INFO_type::visualisation)
  {
    sol.priorities = std::move(priorities);
  }
  return sol;
}

void LNS::destroy_random(Solution& sol) const
{
  last_destroy_strategy = DESTROY_TYPE::RANDOM;
  assertm(settings.destroy_settings.size >= 0 && settings.destroy_settings.size <= instance.get_num_of_agents(),
          "Invalid neighborhood size.");
  std::vector<int> path_idcs(sol.paths.size());
  std::iota(path_idcs.begin(), path_idcs.end(), 0);
  std::shuffle(path_idcs.begin(), path_idcs.end(), rnd_generator);
  assertm((int)path_idcs.size() > settings.destroy_settings.size, "Not enough paths for the destroy operator.");
  int num_to_destroy = std::min(settings.destroy_settings.size, instance.get_num_of_agents());
  sol.destroyed_paths =
      std::vector(std::make_move_iterator(path_idcs.begin()), std::make_move_iterator(path_idcs.begin() + num_to_destroy));
  assertm(sol.destroyed_paths.size() > 0, "No paths were destroyed.");
  sol.feasible = false;
}

void LNS::destroy_randomwalk(Solution& sol) const
{
  last_destroy_strategy = DESTROY_TYPE::RANDOMWALK;

  assertm(constraint_table_initialized, "Constraint table is not initialized.");
  assertm(settings.destroy_settings.size >= 0 && settings.destroy_settings.size <= instance.get_num_of_agents(),
          "Invalid neighborhood size.");
  // if neighborhood size the same as number of agents or more, return all agents
  if (settings.destroy_settings.size >= instance.get_num_of_agents())
  {
    destroy_random(sol);
    return;
  }

  // the tabu list
  static std::unordered_set<int> tabu_list;

  // find the most delayed agent that is not on the tabu list
  assertm(static_cast<int>(sol.delays.size()) == instance.get_num_of_agents(),
          "The length of delay list must be the same as the numebr of agents.");

  int max_delay    = 0;
  int most_delayed = -1;
  for (int i = 0; i < instance.get_num_of_agents(); i++)
  {
    // skip agents in tabu list
    if (tabu_list.find(i) != tabu_list.end())
    {
      continue;
    }

    if (sol.delays[i] > max_delay)
    {
      max_delay    = sol.delays[i];
      most_delayed = i;
    }
  }

  // check, whether to reset tabu list
  if (max_delay == 0)
  {
    // reset the tabu list and try again
    if (!tabu_list.empty())
    {
      tabu_list.clear();
      destroy_randomwalk(sol);
      return;
    }

    // select random agents
    std::cout << "Using random destroy" << std::endl;
    destroy_random(sol);
    return;
  }

  assertm(max_delay > 0, "Max delay is invalid.");
  assertm(most_delayed >= 0 && most_delayed < instance.get_num_of_agents() && sol.delays[most_delayed] == max_delay,
          "Invalid id of most delayed agent");

  // update the tabu list
  tabu_list.insert(most_delayed);
  if (static_cast<int>(tabu_list.size()) == instance.get_num_of_agents())
  {
    tabu_list.clear();
  }

  std::set<int> chosen = {
      most_delayed};  // TODO in the original code they use set, but I think it makes more sense to use either unordered set or vector
  int upperbound = sol.paths[most_delayed].back().interval.t_min;
  assertm(upperbound > 0, "Upperbound is too small.");
  // perform the first randomwalk from start - not in pseudocode, but in MAPF-LNS2 implementation
  int chosen_agent = most_delayed;
  if (!random_walk(chosen_agent, settings.destroy_settings.size, sol.paths[chosen_agent][0].location, 0, upperbound, chosen))
  {
    // perform the random walk again, maximum 10 iterations
    for (int i = 0; i < 10; i++)
    {
      // choose a random time from the agents path
      std::uniform_int_distribution<int> dist_time(0, upperbound);
      int                                chosen_t = dist_time(rnd_generator);

      // choose location
      int rw_start_location = location_at_time(sol.paths[chosen_agent], chosen_t);
      assertm(rw_start_location >= 0, "Invalid start location of randomwalk");

      // perform randomwalk
      if (random_walk(chosen_agent, settings.destroy_settings.size, rw_start_location, chosen_t, upperbound, chosen))
      {
        break;
      }
      assertm(static_cast<int>(chosen.size()) < settings.destroy_settings.size, "Enough agents were selected, but false returned.");

      // choose a random agent
      std::uniform_int_distribution<int> dist(0, chosen.size() - 1);
      int                                idx = dist(rnd_generator);
      assertm(idx >= 0 && idx < static_cast<int>(chosen.size()), "Index out of bounds.");
      // iterator to the first element
      auto it = chosen.begin();
      std::advance(it, idx);
      chosen_agent = *it;
      upperbound   = sol.paths[chosen_agent].back().interval.t_min;
    }
  }

  // if not enough agents selected, repeat
  if (static_cast<int>(chosen.size()) <= 1)
  {
    destroy_randomwalk(sol);
    return;
  }
  assertm(static_cast<int>(chosen.size()) > 1, "Not enough agents selected.");

  // but it is implemented like this in MAPF-LNS2
  // sol.destroyed_paths = std::vector(std::make_move_iterator(chosen.begin()), std::make_move_iterator(chosen.end()));
  sol.destroyed_paths.assign(chosen.begin(), chosen.end());

  std::shuffle(sol.destroyed_paths.begin(), sol.destroyed_paths.end(), rnd_generator);
  assertm(sol.destroyed_paths.size() > 1, "Not enough paths were destroyed.");
  sol.feasible = false;
}


auto LNS::random_walk(int agent_num, int neighborhood_size, int start_location, int start_time, int upperbound, std::set<int>& chosen) const
    -> bool
{
  assertm(static_cast<int>(chosen.size()) < neighborhood_size, "Randomwalk not needed.");
  int curr = start_location;
  for (int t = start_time; t < upperbound; t++)
  {
    std::vector<int> next_locations = instance.get_neighbor_locations(curr);
    next_locations.push_back(curr);  // agent can also stay
    std::shuffle(next_locations.begin(), next_locations.end(), rnd_generator);
    // try all possible neighbors in random order
    bool moved = false;
    for (auto loc : next_locations)
    {
      int next_h_val = instance.get_heuristic_distance(agent_num, loc);
      // if lowerbound on the time to reach goal smaller than limit, accept the move
      if (t + 1 + next_h_val < upperbound)
      {
        // get the agent that is in conflict and add it to the neighborhood
        auto [conflicting_agent_vertex, conflicting_agent_edge] = constraint_table.get_blocking_agent(curr, loc, t + 1);

        // if there is an agent in collision, add it to the neighborhood
        if (conflicting_agent_vertex != -1)
        {
          chosen.insert(conflicting_agent_vertex);
        }

        if (conflicting_agent_edge != -1)
        {
          chosen.insert(conflicting_agent_edge);
        }

        curr  = loc;
        moved = true;
        break;
      }
    }
    // test whether enough agents selected
    if (static_cast<int>(chosen.size()) >= neighborhood_size)
    {
      return true;
    }

    // if none of the next locations was valid, return
    if (!moved)
    {
      break;
    }
  }
  assertm(static_cast<int>(chosen.size()) <= neighborhood_size, "Enough agents selected, but false returned.");
  return false;
}

void LNS::destroy_intersection(Solution& sol) const
{
  last_destroy_strategy = DESTROY_TYPE::INTERSECTION;
  assertm(constraint_table_initialized, "Constraint table is not initialized.");

  // find all intersection vertices
  std::vector<int> intersection_vertices_free;
  intersection_vertices_free.reserve(instance.get_num_free_cells());  // upperbound on number of intersection vertices

  for (int i = 0; i < instance.get_num_free_cells(); i++)
  {
    const auto& agents_counts = constraint_table.get_agents_counts_free(i);
    if (static_cast<int>(agents_counts.size()) >= 3)
    {
      intersection_vertices_free.push_back(i);
    }
  }

  // create queue
  std::queue<int> openlist;

  // pick a random intersection vertex
  {
    std::uniform_int_distribution<int> dist(0, intersection_vertices_free.size() - 1);
    int                                idx    = dist(rnd_generator);
    int                                chosen = instance.free_location_to_location(intersection_vertices_free[idx]);
    openlist.push(chosen);
  }


  std::unordered_set<int> neighborhood;  // TODO try set
  std::vector<bool>       known(instance.get_num_cells(), false);

  while (!openlist.empty())
  {
    // retrieve current node
    int current = openlist.front();
    openlist.pop();
    // std::cout << "Current: " << current << std::endl;
    if (known[current])
    {
      continue;
    }
    known[current] = true;

    // check whether intersection vertex
    auto agents_counts = constraint_table.get_agents_counts(current);
    if (static_cast<int>(agents_counts.size()) >= 3)
    {
      // add agents that visit this vertex to the neighborhood
      get_intersection_agents(neighborhood, current);
    }

    // check whether enough agents selected
    // std::cout << "Selected: " << static_cast<int>(neighborhood.size()) << std::endl;
    if (settings.destroy_settings.size == static_cast<int>(neighborhood.size()))
    {
      break;
    }

    // add neighbors to the queue
    auto neighbors = instance.get_neighbor_locations(current);
    for (auto it : neighbors)
    {
      openlist.push(it);
    }
  }
  assertm(static_cast<int>(neighborhood.size()) > 1, "Not enough agents selected for destroy.");

  // convert to vector and shuffle
  sol.destroyed_paths = std::vector(std::make_move_iterator(neighborhood.begin()), std::make_move_iterator(neighborhood.end()));
  // shuffle the paths
  std::shuffle(sol.destroyed_paths.begin(), sol.destroyed_paths.end(), rnd_generator);
  assertm(sol.destroyed_paths.size() > 0, "No paths were destroyed.");
  sol.feasible = false;
}

void LNS::get_intersection_agents(std::unordered_set<int>& neighborhood, int current) const
{
  // find maximal T
  int t_max = constraint_table.get_last_constraint_start(current);

  // choose random time
  std::uniform_int_distribution<int> dist(0, t_max);
  int                                t     = dist(rnd_generator);
  int                                delta = 0;
  while (static_cast<int>(neighborhood.size()) < settings.destroy_settings.size && t + delta <= t_max && t - delta >= 0)
  {
    // find the agent, that blocks the cell at time t + delta
    auto [blocking_agent_1, blocking_agent_2] = constraint_table.get_blocking_agent(current, current, t + delta);
    // find the agent, that blocks the cell at time t - delta
    auto [blocking_agent_3, blocking_agent_4] = constraint_table.get_blocking_agent(current, current, t - delta);

    // add the agents to the neighborhood
    std::array<int, 4> blocking_agents = {blocking_agent_1, blocking_agent_2, blocking_agent_3, blocking_agent_4};
    for (auto blocking_agent : blocking_agents)
    {
      if (blocking_agent >= 0 && static_cast<int>(neighborhood.size()) < settings.destroy_settings.size)
      {
        neighborhood.insert(blocking_agent);
      }
    }
    delta++;
  }
}

void LNS::destroy_adaptive(Solution& sol) const
{
  // choose strategy by roulette wheel selection
  // calculate the sum of weights
  auto sum_of_weights = std::reduce(destroy_weights.begin(), destroy_weights.end());
  assertm(sum_of_weights > 0, "Invalid weights.");


  // choose a random number in the range [0, sum_of_weights]
  std::uniform_real_distribution<double> dist(0.0, sum_of_weights);
  double                                 rand_value = dist(rnd_generator);

  // spin the roulette
  double       cummulative_sum = 0.0;
  DESTROY_TYPE chosen_type     = DESTROY_TYPE::ADAPTIVE;
  for (int i = 0; i < static_cast<int>(destroy_weights.size()); i++)
  {
    cummulative_sum += destroy_weights[i];
    if (rand_value <= cummulative_sum)
    {
      auto chosen = magic_enum::enum_cast<DESTROY_TYPE>(i);
      if (chosen.has_value())
      {
        chosen_type = chosen.value();
        break;
      }
      throw std::runtime_error(std::string("Unknown DESTROY_TYPE ") + std::string(magic_enum::enum_name<DESTROY_TYPE>(chosen_type)) +
                               std::string(" encountered in adaptive destroy."));
    }
  }
  assertm(chosen_type != DESTROY_TYPE::ADAPTIVE && chosen_type != DESTROY_TYPE::RANDOM_CHOOSE, "Did not select any destroy strategy.");

  // if sum of weights too small, increase it
  if (sum_of_weights < 0.1)
  {
    for (double& destroy_weight : destroy_weights)
    {
      destroy_weight *= 10;
    }
  }
  // std::cout << "Weights are: " << std::endl;
  // for (auto it : destroy_weights)
  // {
  //   std::cout << it << ", ";
  // }
  // std::cout << std::endl;
  // std::cout << "Chosen destroy: " << magic_enum::enum_name<DESTROY_TYPE>(chosen_type) << std::endl;

  // apply the destroy operator
  switch (chosen_type)
  {
    case DESTROY_TYPE::RANDOM:
      destroy_random(sol);
      break;
    case DESTROY_TYPE::RANDOMWALK:
      destroy_randomwalk(sol);
      break;
    case DESTROY_TYPE::INTERSECTION:
      destroy_intersection(sol);
      break;
    default:
      throw std::runtime_error(std::string("Unknown DESTROY_TYPE ") + std::string(magic_enum::enum_name<DESTROY_TYPE>(chosen_type)) +
                               std::string(" encountered in adaptive destroy."));
  }
}

void LNS::destroy_blocked(Solution& sol) const
{
  last_destroy_strategy = DESTROY_TYPE::BLOCKED;

  // if a random value is higher, than the threshold, perform randomwalk instead
  std::uniform_real_distribution<float> blocked_dist(0, 1);
  if (blocked_dist(rnd_generator) >= threshold_blocked)
  {
    // std::cout << "Running randomwalk. Threshold: " << threshold_blocked << std::endl;
    destroy_randomwalk(sol);
    return;
  }

  // find all agents whose end is blocked
  std::vector<int> blocked_agents;
  for (int i = 0; i < instance.get_num_of_agents(); i++)
  {
    const int loc        = instance.get_goal_locations()[i];
    const int reach_time = sol.paths[i].back().interval.t_min;
    assertm(sol.paths[i].back().location == loc, "The path does not end in the goal location");
    // skip agents that dont have any delay
    if (reach_time == instance.get_heuristic_distance(i, instance.get_start_locations()[i]) - 1)
    {
      continue;
    }
    assertm(reach_time >= instance.get_heuristic_distance(i, instance.get_start_locations()[i]), "Invalid reach time");
    assertm(constraint_table_initialized, "Constraint table is not initialized.");
    int from = loc;
    if (static_cast<int>(sol.paths[i].size()) > 1)
    {
      from = std::prev(sol.paths[i].end(), 2)->location;
    }
    auto [blocking_agent_1, blocking_agent_2] = constraint_table.get_blocking_agent(from, loc, reach_time - 1);
    if (blocking_agent_1 != -1 || blocking_agent_2 != -1)
    {
      blocked_agents.push_back(i);
    }
  }

  // if no agent is blocked, use the randomwalk
  if (blocked_agents.empty())
  {
    destroy_randomwalk(sol);
    return;
  }
  // std::cout << "Number of blocked_agents: " << blocked_agents.size() << std::endl;

  // choose a random agent from blocked agents
  std::uniform_int_distribution<int> dist(0, blocked_agents.size() - 1);
  int                                chosen_agent = -1;
  {
    int idx      = dist(rnd_generator);
    chosen_agent = blocked_agents[idx];
  }
  // std::cout << "Chosen agent: " << chosen_agent << std::endl;

  // add the agent to the neighborhood
  std::unordered_set<int> neighborhood = {chosen_agent};
  int                     idx          = -1;
  sol.destroyed_paths                  = {chosen_agent};
  while (static_cast<int>(sol.destroyed_paths.size()) < settings.destroy_settings.size)
  {
    const int        loc                 = instance.get_goal_locations()[chosen_agent];
    const int        min_reach_time      = instance.get_heuristic_distance(chosen_agent, instance.get_start_locations()[chosen_agent]);
    std::vector<int> blocking_agents_new = constraint_table.get_blocking_agents(loc, min_reach_time - 1);

    // add all new blocking agents to the neighborhood
    for (auto it : blocking_agents_new)
    {
      if (neighborhood.find(it) == neighborhood.end())
      {
        neighborhood.insert(it);
        sol.destroyed_paths.push_back(it);
        // make sure that the neighborhood size is not exceeded
        // if (static_cast<int>(sol.destroyed_paths.size()) >= settings.destroy_settings.size)
        // {
        //   break;
        // }
      }
    }

    // update index
    idx++;
    if (idx >= static_cast<int>(sol.destroyed_paths.size()))
    {
      break;
    }

    // update chosen agent
    chosen_agent = sol.destroyed_paths[idx];
  }
  // shuffle randomly,  only when threshold is low (we need more variance)
  if (blocked_dist(rnd_generator) >= threshold_blocked)
  {
    // std::cout << "Shuffle. Threshold: " << threshold_blocked << std::endl;
    std::shuffle(sol.destroyed_paths.begin(), sol.destroyed_paths.end(), rnd_generator);
  }
  // dont exceed the neighborhood size
  if (settings.destroy_settings.size < static_cast<int>(sol.destroyed_paths.size()))
  {
    sol.destroyed_paths.resize(settings.destroy_settings.size);
  }
  assertm(settings.destroy_settings.size == 1 || static_cast<int>(sol.destroyed_paths.size()) > 1,
          "At least two paths should be destroyed.");
  sol.feasible = false;
}

void LNS::destroy_random_choose(Solution& sol) const
{
  std::uniform_int_distribution<int> dist(0, 2);
  int                                rand_value = dist(rnd_generator);
  // apply the destroy operator
  switch (rand_value)
  {
    case 0:
      destroy_random(sol);
      break;
    case 1:
      destroy_randomwalk(sol);
      break;
    case 2:
      destroy_intersection(sol);
      break;
    default:
      throw std::runtime_error("Unknown DESTROY_TYPE encountered in adaptive destroy.");
  }
}

void LNS::repair_default(Solution& sol) const
{
  assertm(!sol.feasible, "Can not repair a feasible solution");
  assertm((int)sol.destroyed_paths.size() > 0, "There are no destroyed paths in the solution to be repaired.");

  if (settings.sipp_settings.info_type == INFO_type::visualisation)
  {
    sipp_info.clear();
    sipp_info.resize(sol.destroyed_paths.size());
  }

  // remove the destroyed paths
  for (auto& it : sol.destroyed_paths)
  {
    // std::cout << "Removing path: " << it << std::endl;
    planner->safe_interval_table.remove_constraints(sol.paths[it]);
    if (constraint_table_initialized)
    {
      constraint_table.remove_constraints(sol.paths[it], it);
    }
    sol.paths[it].clear();
    already_planned.erase(it);
  }
  sol.feasible = true;

  // replan
  for (int i = 0; i < static_cast<int>(sol.destroyed_paths.size()); i++)
  {
    int           path_idx = sol.destroyed_paths[i];
    TimePointPath tp_path  = planner->plan(path_idx, already_planned);
    if (tp_path.empty())
    {
      sol.feasible = false;
      return;
    }

    // get the visualization info
    if (settings.sipp_settings.info_type == INFO_type::visualisation)
    {
      sipp_info[i] = std::move(planner->iter_info);
      planner->iter_info.clear();
    }
    // TODO : add ACID here or in the SIPPS so it is called for 1 agent only 
    planner->safe_interval_table.add_constraints(tp_path);
    if (constraint_table_initialized)
    {
      constraint_table.add_constraints(tp_path, path_idx);
    }
    sol.paths[path_idx] = tp_path;
    already_planned.insert(path_idx);
  }
}

void LNS::discard_solution(const Solution& sol, const Solution& prev_sol) const
{
  for (int it : sol.destroyed_paths)
  {
    // remove the destroyed paths
    if (!sol.paths[it].empty())
    {
      planner->safe_interval_table.remove_constraints(sol.paths[it]);
      if (constraint_table_initialized)
      {
        constraint_table.remove_constraints(sol.paths[it], it);
      }
    }
  }

  // restore the old paths
  for (int it : sol.destroyed_paths)
  {
    planner->safe_interval_table.add_constraints(prev_sol.paths[it]);
    if (constraint_table_initialized)
    {
      constraint_table.add_constraints(prev_sol.paths[it], it);
    }
  }
}
