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
#define EXPERIMENT_NAME "destroy"

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

#define NEIGHBORHOOD_SIZES \
  {                        \
    4, 8, 16               \
  }

#define MAX_ITER 2000
// #define MAX_ITER 0

#define TIME_LIMITS    \
  {                    \
    1, 5, 5, 5, 60, 60 \
  }

static void experiment_function(LNS& lns);

auto main(int argc, char** argv) -> int
{
  // assertm(false, "Experiments should be compiled in Release mode to have reliable results!");

  // prepare the algorithms
  std::vector<Algorithm> algorithms;
  std::vector<int>       neighborhood_sizes(NEIGHBORHOOD_SIZES);
  for (const auto& destroy : magic_enum::enum_values<DESTROY_TYPE>())
  {
    for (const auto& neighborhood_size : neighborhood_sizes)
    {
      LNS_settings lns_settings(MAX_ITER, 20, {destroy, neighborhood_size},
                                {SIPP_implementation::SIPP_mapf_lns, INFO_type::experiment, 1.0}, false);
      algorithms.emplace_back(SIPP_implementation::SIPP_mapf_lns, lns_settings,
                              std::make_pair("destroy_type", magic_enum::enum_name<DESTROY_TYPE>(destroy)),
                              std::make_pair("neighborhood_size", neighborhood_size));
    }
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
