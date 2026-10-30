/*
 * Author: Jan Chleboun
 * Date: 04-03-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */
#include <nlohmann/json.hpp>
#include <random>

#include "Experiment.h"
#include "Instance.h"
#include "LNS.h"
#include "indicators/indicators.hpp"
#include "magic_enum/magic_enum.hpp"
#include "utils.h"

#define RUNS_PER_MAP 100
#define EXPERIMENT_NAME "sipp_pp"

#define SHOW_PROGRESS

#define MAPS                                                                                      \
  {                                                                                               \
    "empty-8-8", "empty-32-32", "random-32-32-20", "warehouse-10-20-10-2-1", "ost003d", "den520d" \
  }

#define NUMS_AGENTS                                                                                                \
  {                                                                                                                \
    {8, 16, 24}, {100, 150, 200, 250, 300}, {20, 40, 60, 80, 100}, {50, 100, 150, 200}, {100, 200, 300, 400, 500}, \
    {                                                                                                              \
      100, 200, 300, 400, 500                                                                                      \
    }                                                                                                              \
  }

#define TIME_LIMITS  \
  {                  \
    1, 1, 1, 1, 1, 1 \
  }

static void experiment_function(LNS& lns);

auto main(int argc, char** argv) -> int
{
  // assertm(false, "Experiments should be compiled in Release mode to have reliable results!");

  // prepare the algorithms
  std::vector<Algorithm> algorithms;

  // original mapf
  {
    SIPP_implementation algo_type = SIPP_implementation::SIPP_mapf_lns;
    LNS_settings        lns_settings(0, 1, {DESTROY_TYPE::RANDOM, 10}, {algo_type, INFO_type::experiment, 1.0});
    algorithms.emplace_back(algo_type, lns_settings);
  }

  // SIPP mine ap
  {
    SIPP_implementation algo_type = SIPP_implementation::SIPP_mine_ap;
    LNS_settings        lns_settings(0, 1, {DESTROY_TYPE::RANDOM, 10}, {algo_type, INFO_type::experiment, 1.0});
    algorithms.emplace_back(algo_type, lns_settings);
  }

  // Suboptimal SIPP w = 1.0
  {
    SIPP_implementation algo_type = SIPP_implementation::SIPP_suboptimal_ap;
    LNS_settings        lns_settings(0, 1, {DESTROY_TYPE::RANDOM, 10}, {algo_type, INFO_type::experiment, 1.0, 5});
    algorithms.emplace_back(algo_type, lns_settings, std::make_pair("w", 1.0), std::make_pair("p", 5));
  }

  // Suboptimal SIPP w = 1.1
  {
    SIPP_implementation algo_type = SIPP_implementation::SIPP_suboptimal_ap;
    LNS_settings        lns_settings(0, 20, {DESTROY_TYPE::RANDOM, 10}, {algo_type, INFO_type::experiment, 1.1, 5});
    algorithms.emplace_back(algo_type, lns_settings, std::make_pair("w", 1.1), std::make_pair("p", 5));
  }

#ifdef SHOW_PROGRESS
  const bool show_progress = true;
#else
  const bool show_progress = false;
#endif

  // create the experiment
  Experiment experiment(EXPERIMENT_NAME, experiment_function, MAPS, NUMS_AGENTS, TIME_LIMITS, algorithms, RUNS_PER_MAP, show_progress);

  // run the experiment
  experiment.run();
}

void experiment_function(LNS& lns)
{
  lns.find_initial_solution();
}
