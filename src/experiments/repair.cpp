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

#define RUNS_PER_MAP 50
#define EXPERIMENT_NAME "repair"

#define SHOW_PROGRESS

#define MAPS                                                                                      \
  {                                                                                               \
    "empty-8-8", "empty-32-32", "random-32-32-20", "warehouse-10-20-10-2-1", "ost003d", "den520d" \
  }

#define NUMS_AGENTS                   \
  {                                   \
    {32}, {200}, {100}, {150}, {400}, \
    {                                 \
      500                             \
    }                                 \
  }

#define SUBOPTIMALITIES  \
  {                      \
    1.0, 1.02, 1.05, 1.1 \
  }

#define P 5

#define NEIGHBORHOOD_SIZE 8

#define MAX_ITER 10000
// #define MAX_ITER 0
#define TIME_LIMIT 20

#define TIME_LIMITS    \
  {                    \
    1, 5, 5, 5, 30, 30 \
  }

static void experiment_function(LNS& lns);

auto main(int argc, char** argv) -> int
{
  // assertm(false, "Experiments should be compiled in Release mode to have reliable results!");

  // prepare the algorithms
  std::vector<Algorithm> algorithms;
  std::vector<double>    suboptimalities(SUBOPTIMALITIES);

  // original version
  {
    LNS_settings lns_settings(MAX_ITER, TIME_LIMIT, {DESTROY_TYPE::RANDOMWALK, NEIGHBORHOOD_SIZE},
                              {SIPP_implementation::SIPP_mapf_lns, INFO_type::experiment, 1.0});
    algorithms.emplace_back(SIPP_implementation::SIPP_mapf_lns, lns_settings);
  }

  // optimal version
  {
    LNS_settings lns_settings(MAX_ITER, TIME_LIMIT, {DESTROY_TYPE::RANDOMWALK, NEIGHBORHOOD_SIZE},
                              {SIPP_implementation::SIPP_mine_ap, INFO_type::experiment, 1.0});
    algorithms.emplace_back(SIPP_implementation::SIPP_mine_ap, lns_settings);
  }

  // SIPP suboptimal with w = 1.0, p = 1
  {
    LNS_settings lns_settings(MAX_ITER, TIME_LIMIT, {DESTROY_TYPE::RANDOMWALK, NEIGHBORHOOD_SIZE},
                              {SIPP_implementation::SIPP_suboptimal_ap, INFO_type::experiment, 1.0, 1});
    algorithms.emplace_back(SIPP_implementation::SIPP_suboptimal_ap, lns_settings, std::make_pair("w", 1.0), std::make_pair("p", 1));
  }
  // SIPP suboptimal with various w and p = 5
  for (auto w : suboptimalities)
  {
    LNS_settings lns_settings(MAX_ITER, TIME_LIMIT, {DESTROY_TYPE::RANDOMWALK, NEIGHBORHOOD_SIZE},
                              {SIPP_implementation::SIPP_suboptimal_ap, INFO_type::experiment, w, P});
    algorithms.emplace_back(SIPP_implementation::SIPP_suboptimal_ap, lns_settings, std::make_pair("w", w), std::make_pair("p", P));
  }

#ifdef SHOW_PROGRESS
  const bool show_progress = true;
#else
  const bool show_progress = false;
#endif

  // create the experiment
  Experiment experiment(EXPERIMENT_NAME, experiment_function, MAPS, NUMS_AGENTS, TIME_LIMITS, algorithms, RUNS_PER_MAP, show_progress,
                        true);

  // run the experiment
  experiment.run();
}

void experiment_function(LNS& lns)
{
  lns.solve();
}
