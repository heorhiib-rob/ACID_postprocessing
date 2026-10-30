/*
 * Author: Jan Chleboun
 * Date: 06-04-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */
#include "Experiment.h"

#include <utility>

#include "LNS.h"
#include "Solver.h"

Experiment::Experiment(std::string experiment_name_, std::function<void(LNS& vis)> experiment_function_, std::vector<std::string> maps_,
                       std::vector<std::vector<int>> agent_nums_, std::vector<double> time_limits_, std::vector<Algorithm>& algorithms_,
                       int runs_per_map_, bool show_progress_, bool load_init_sol_)
    : experiment_name(std::move(experiment_name_)),
      experiment_function(std::move(experiment_function_)),
      maps(std::move(maps_)),
      agent_nums(std::move(agent_nums_)),
      time_limits(std::move(time_limits_)),
      algorithms(algorithms_),
      runs_per_map(runs_per_map_),
      show_progress(show_progress_),
      load_init_sol(load_init_sol_),
      rnd_gen(std::mt19937(std::random_device{}())),
      dist(0, std::numeric_limits<int>::max())
{
  // assertm(false, "Experiments should be compiled in Release mode to have reliable results!");
  assertm(agent_nums.size() == maps.size(), "Agent nums must be specified for each map.");
  assertm(time_limits.size() == maps.size(), "Time limit must be specified for each map.");
  skips = what_can_be_skipped(experiment_name, maps, agent_nums, runs_per_map, algorithms);
  if (show_progress)
  {
    create_progress_bars(agent_nums[0].size());
  }
  open_database();
}

void Experiment::create_progress_bars(int agent_nums_init)
{
  assertm(show_progress, "Creating progress bars when progress should not be shown.");
  progress_bars.push_back(create_progress_bar("Maps:\t\t", static_cast<int>(maps.size())));
  progress_bars.push_back(create_progress_bar("Agent nums:\t", agent_nums_init));
  progress_bars.push_back(create_progress_bar("Scene:\t\t", runs_per_map));
  multi_progress_bar.emplace(*progress_bars[0], *progress_bars[1], *progress_bars[2]);

  // first tick
  multi_progress_bar->set_progress<0>(static_cast<size_t>(0));
  multi_progress_bar->set_progress<1>(static_cast<size_t>(0));
  multi_progress_bar->set_progress<2>(static_cast<size_t>(0));
}

void Experiment::run()
{
  // iterate over all maps
  for (int i = 0; i < static_cast<int>(static_cast<int>(maps.size())); i++)
  {
    // reset the map progress bars
    if (show_progress)
    {
      multi_progress_bar->set_progress<0>(static_cast<size_t>(i));
      multi_progress_bar->set_option<0>(
          indicators::option::PostfixText{std::to_string(i) + "/" + std::to_string(static_cast<int>(maps.size())) + "    "});
    }

    double time_limit = time_limits[i];

    // skip maps that were already solved
    const int start_offset_map = sum_agent_nums * runs_per_map * static_cast<int>(algorithms.size());
    sum_agent_nums += static_cast<int>(agent_nums[i].size());
    const int end_offset_map = sum_agent_nums * runs_per_map * static_cast<int>(algorithms.size());

    if (std::all_of(skips.begin() + start_offset_map, skips.begin() + end_offset_map, [](bool b) { return b; }))
    {
      if (show_progress)
      {
        multi_progress_bar->set_progress<1>(agent_nums[i].size());
        multi_progress_bar->set_progress<2>(static_cast<size_t>(runs_per_map));
      }
      skipped_maps++;
      skipped_agent_nums += static_cast<int>(agent_nums[i].size());
      skipped_runs += static_cast<int>(agent_nums[i].size()) * runs_per_map;
      skipped_implementations += static_cast<int>(agent_nums[i].size()) * runs_per_map * static_cast<int>(algorithms.size());
      continue;
    }

    // retrieve map name and scene names
    const std::string              map_name = maps[i];
    const std::vector<std::string> scenes   = get_scene_names(map_name, runs_per_map);

    if (show_progress)
    {
      multi_progress_bar->set_option<1>(indicators::option::MaxProgress{(int)agent_nums[i].size()});
    }
    // iterate over agent numbers
    for (int j = 0; j < static_cast<int>(agent_nums[i].size()); j++)
    {
      // update the progress bar iteration number display
      if (show_progress)
      {
        multi_progress_bar->set_option<1>(
            indicators::option::PostfixText{std::to_string(j) + "/" + std::to_string(agent_nums[i].size()) + "     "});
        multi_progress_bar->set_progress<1>(static_cast<size_t>(j));
      }

      // skip agent_nums that were already solved
      const int start_offset_an = start_offset_map + j * runs_per_map * static_cast<int>(algorithms.size());
      const int end_offset_an   = start_offset_map + (j + 1) * runs_per_map * static_cast<int>(algorithms.size());
      if (std::all_of(skips.begin() + start_offset_an, skips.begin() + end_offset_an, [](bool b) { return b; }))
      {
        if (show_progress)
        {
          multi_progress_bar->set_progress<2>((size_t)runs_per_map);
        }
        skipped_agent_nums++;
        skipped_runs += runs_per_map;
        skipped_implementations += runs_per_map * static_cast<int>(algorithms.size());
        continue;
      }

      int agent_num = agent_nums[i][j];

      // load all instances TODO load only instances that will not be skipped
      auto [instances, preprocessing_times] = load_instances(scenes, map_name, agent_num);

      // make sure every map gets enough runs
      for (int k = 0; k < runs_per_map; k++)
      {
        // update the progress bar iteration number display
        if (show_progress)
        {
          multi_progress_bar->set_option<2>(
              indicators::option::PostfixText{std::to_string(k) + "/" + std::to_string(runs_per_map) + "     "});
          multi_progress_bar->set_progress<2>(static_cast<size_t>(k));
        }

        // skip runs that were already solved
        const int start_offset_run = start_offset_an + k * static_cast<int>(algorithms.size());
        const int end_offset_run   = start_offset_an + (k + 1) * static_cast<int>(algorithms.size());
        if (std::all_of(skips.begin() + start_offset_run, skips.begin() + end_offset_run, [](bool b) { return b; }))
        {
          skipped_runs++;
          skipped_implementations += static_cast<int>(algorithms.size());
          continue;
        }

        // choose instance
        assertm(scenes.size() == instances.size(), "The size of scenes and instances must be the same.");
        const int          idx        = k % static_cast<int>(scenes.size());
        const std::string& scene_name = scenes[idx];
        const Instance&    instance   = instances[idx];
        // check whether enough agents loaded
        if (instance.get_num_of_agents() != agent_num)
        {
          throw std::runtime_error("Loaded " + std::to_string(instance.get_num_of_agents()) + " agents instead of " +
                                   std::to_string(agent_num));
        }

        // generate a random seed (same for all implementations) TODO - for experiments found in the database use the same seed
        int seed = 0;
        // look into the database for a seed
        size_t dot_pos = scene_name.find('.');
        if (dot_pos == std::string::npos)
        {
          throw std::runtime_error("Could not find dot in scene name: '" + scene_name + "'");
        }
        const std::string scene_name_stripped = scene_name.substr(map_name.size() + 1, scene_name.find('.') - map_name.size() - 1);
        int               seed_from_db = get_seed_from_db(agent_num, map_name, scene_name_stripped, k / static_cast<int>(scenes.size()));
        // std::cout << "Seed from db: " << seed_from_db << std::endl;
        if (seed_from_db >= 0)
        {
          seed = seed_from_db;
        }
        else
        {
          seed = dist(rnd_gen);
        }

        // load the initial solution
        Solution init_sol;
        if (load_init_sol)
        {
          // get the file name
          std::string initsol_path = get_initsol_path(agent_num, map_name, scene_name_stripped, k / static_cast<int>(scenes.size()));
          // load the solution
          init_sol.load(initsol_path, instance);
          if (!init_sol.feasible)
          {
            throw std::runtime_error("Loaded infeasile initial solution.");
          }
          // calculate cost
          init_sol.calculate_cost(instance);
        }

        // iterate over all implementations
        for (int l = 0; l < static_cast<int>(algorithms.size()); l++)
        {
          // skip already solved
          if (skips[start_offset_run + l])
          {
            skipped_implementations++;
            continue;
          }


          Algorithm& algo = algorithms[l];

          // create the seeded random generator (for experiment reproducibility)
          auto rnd_generator = std::mt19937(seed);

          // prepare the solver
          LNS solver(instance, rnd_generator, nullptr, algo.lns_settings);

          // change the time limit to the time limit of this map
          solver.settings.time_limit = time_limit;

          // use the loaded initial solution
          if (load_init_sol)
          {
            solver.solution               = init_sol;
            solver.found_initial_solution = true;

            // build the safe interval table
            solver.planner->safe_interval_table.build_sequential(init_sol.paths);

            // add to the log
            solver.log.bsf_solution_cost.push_back(init_sol.sum_of_costs);
            solver.log.bsf_makespan.push_back(init_sol.makespan);
            solver.log.used_operator.push_back(DESTROY_TYPE::ADAPTIVE);
            solver.log.iteration_time_wall.push_back(0);
            solver.log.iteration_time_cpu.push_back(0);
          }

          // run the experiment function and measure its execution time
          Clock clock;
          clock.start();
          experiment_function(solver);
          auto [pp_duration_wall, pp_duration_cpu] = clock.end();

          // calculate the cost
          solver.solution.calculate_cost(instance);

          // write the results of the experiment
          const std::string algorithm_name = algo.get_name();
          nlohmann::json    experiment_res = {{"experiment_name", experiment_name},
                                           {"map_name", map_name},
                                           {"scene_name", scene_name},
                                           {"num_agents", agent_num},
                                           {"algo_name", algorithm_name},
                                           {"algo_parameters", algo.get_parameters_str()},
                                           {"feasible", solver.solution.feasible},
                                           {"sum_of_dist", instance.get_sum_of_distances()},
                                           {"sum_of_cost", solver.log.bsf_solution_cost},
                                           {"makespan", solver.log.bsf_makespan},
                                           {"preprocessing_time_wall", preprocessing_times[idx].first},
                                           {"preprocessing_time_cpu", preprocessing_times[idx].second},
                                           {"experiment_time_wall", pp_duration_wall},
                                           {"experiment_time_cpu", pp_duration_cpu},
                                           {"iteration_time_wall", solver.log.iteration_time_wall},
                                           {"iteration_time_cpu", solver.log.iteration_time_cpu},
                                           {"operators", solver.log.get_used_operator_str()},
                                           {"expanded", solver.get_num_of_expanded_nodes()},
                                           {"generated", solver.get_num_of_generated_nodes()},
                                           {"seed", seed}};

          // save the experiment
          save_experiment(experiment_res, agent_num, map_name, scene_name, experiment_name, algorithm_name);
        }
      }
    }
  }
  // mark all progress bars as finished
  if (show_progress)
  {
    multi_progress_bar->set_option<0>(indicators::option::PostfixText{std::to_string(static_cast<int>(maps.size())) + "/" +
                                                                      std::to_string(static_cast<int>(maps.size())) + "     "});
    multi_progress_bar->set_progress<0>(maps.size());
    multi_progress_bar->set_progress<1>(agent_nums.back().size());
    multi_progress_bar->set_option<1>(indicators::option::PostfixText{std::to_string(agent_nums.back().size()) + "/" +
                                                                      std::to_string(agent_nums.back().size()) + "     "});
    multi_progress_bar->set_option<2>(
        indicators::option::PostfixText{std::to_string(runs_per_map) + "/" + std::to_string(runs_per_map) + "     "});
    multi_progress_bar->set_progress<2>(static_cast<size_t>(runs_per_map));
  }

  std::cout << "Experiment finished. Skipped: " << skipped_maps << "/" << static_cast<int>(maps.size()) << " maps, " << skipped_agent_nums
            << "/" << sum_agent_nums << " agent_nums, " << skipped_runs << "/" << sum_agent_nums * runs_per_map << " runs, "
            << skipped_implementations << "/" << sum_agent_nums * runs_per_map * static_cast<int>(algorithms.size()) << " implementations."
            << std::endl;
}

auto Experiment::get_seed_from_db(int agent_num, const std::string& map_name, const std::string& scene_name_stripped, int run) -> int
{
  // Query the database
  std::string command = "SELECT seed FROM " + experiment_name + " WHERE file_name = ?";
  // std::string       command             = "SELECT seed FROM experiments";
  std::string experiment_filename;
  if (experiment_name == "sipp_pp")
  {
    experiment_filename = experiment_name + "_" + std::to_string(agent_num) + "agents_" + map_name + "-" + scene_name_stripped +
                          "SIPP_lns_orig_" + std::to_string(run);
  }
  else if (experiment_name == "influence_of_p" || experiment_name == "influence_of_w" || experiment_name == "repair" ||
           experiment_name == "influence_of_ap" || experiment_name == "overall_improvement")
  {
    return -1;  // w and p are only in my solver
  }
  else if (experiment_name == "destroy")
  {
    experiment_filename = experiment_name + "_" + std::to_string(agent_num) + "agents_" + map_name + "_" + scene_name_stripped +
                          "_destroy_type_ORIGO_Adaptive_neighborhood_size_4_" + std::to_string(run);
  }
  else if (experiment_name == "feasible" || experiment_name == "feasible_restarts")
  {
    experiment_filename = experiment_name + "_" + std::to_string(agent_num) + "agents_" + map_name + "-" + scene_name_stripped +
                          "SIPP_lns_orig_" + std::to_string(run);
  }
  else
  {
    throw std::runtime_error("Unknown experiment name: " + experiment_name);
  }

  // std::cout << "Filename: '" << experiment_filename << "'" << std::endl;
  sqlite3_stmt* stmt = nullptr;

  int rc = sqlite3_prepare_v2(db, command.c_str(), -1, &stmt, nullptr);
  if (rc == SQLITE_OK)  // fetched data
  {
    rc = sqlite3_bind_text(stmt, 1, experiment_filename.c_str(), experiment_filename.size(), SQLITE_TRANSIENT);
    if (rc != SQLITE_OK)
    {
      throw std::runtime_error("Unable to bind text in the sql statement.");
    }

    if (sqlite3_step(stmt) == SQLITE_ROW)
    {
      int seed = sqlite3_column_int(stmt, 0);
      if (seed < 0)
      {
        throw std::runtime_error("Invalid seed: '" + std::to_string(seed) + "'");
      }
      sqlite3_finalize(stmt);
      return seed;
    }
  }
  else
  {
    // the experiment table does not exist in the database yet
  }

  sqlite3_finalize(stmt);
  return -1;
}

auto Experiment::get_initsol_path(int agent_num, const std::string& map_name, const std::string& scene_name_stripped, int run)
    -> std::string
{
  return get_base_path() + "/experiments/" + experiment_name + "_" + "initsols/" + experiment_name + "_" + std::to_string(agent_num) +
         "agents_" + map_name + "_" + scene_name_stripped + "_" + std::to_string(run);
}

void Experiment::open_database()
{
  std::string path_to_database =
      std::filesystem::canonical("/proc/self/exe").parent_path().parent_path().parent_path().parent_path().string() + "/experiments/" +
      DATABASE_NAME;
  int rc = sqlite3_open(path_to_database.c_str(), &db);
  // check that the database was opened successfully
  if (rc != SQLITE_OK)
  {
    std::cout << "WARNING: Can't open database: " << sqlite3_errmsg(db) << std::endl;
  }
}
