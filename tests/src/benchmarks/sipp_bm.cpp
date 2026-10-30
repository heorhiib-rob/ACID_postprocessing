#include <benchmark/benchmark.h>

#include <algorithm>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>

#include "Instance.h"
#include "LNS.h"
#include "test_utils.h"


// helper function to run a benchmark
void BM_SIPP_helper(benchmark::State& state, std::string map_name, std::string scen_name)
{
  // setup the instance
  std::string     base_path  = get_base_path_tests();  // path to my_solver
  int             num_agents = state.range(0);
  std::unique_ptr instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/" + map_name, base_path + "/tests/test_scen/" + scen_name, num_agents);
  for (auto _ : state)
  {
    // create LNS
    auto         rnd_generator = std::mt19937(0);
    LNS_settings lns_settings(0, 5, {DESTROY_TYPE::RANDOM, 10}, {SIPP_implementation::SIPP_mine, INFO_type::no_info, 1.0});
    LNS          lns(*instance, rnd_generator, nullptr, lns_settings);

    // plan path by Prioritized Planning
    Solution sol = lns.PrioritizedPlanning();
  }
}

// Benchmark for SIPP using prioritized planning on map den520d scene 0
static void BM_PP_den520_scen0(benchmark::State& state)
{
  BM_SIPP_helper(state, "den520d.map", "den520d-random-0.scen");
}
BENCHMARK(BM_PP_den520_scen0)->Arg(1)->Arg(10)->Arg(100)->Arg(500);


// Benchmark for SIPP using prioritized planning on map den520d scene 1
static void BM_PP_den520_scen1(benchmark::State& state)
{
  BM_SIPP_helper(state, "den520d.map", "den520d-random-1.scen");
}
BENCHMARK(BM_PP_den520_scen1)->Arg(1)->Arg(10)->Arg(100)->Arg(500);

// Benchmark for SIPP using prioritized planning on map den520d scene 2
static void BM_PP_den520_scen2(benchmark::State& state)
{
  BM_SIPP_helper(state, "den520d.map", "den520d-random-2.scen");
}
BENCHMARK(BM_PP_den520_scen2)->Arg(1)->Arg(10)->Arg(100)->Arg(500);

static void BM_PP_den520_scen3(benchmark::State& state)
{
  BM_SIPP_helper(state, "den520d.map", "den520d-random-3.scen");
}
BENCHMARK(BM_PP_den520_scen3)->Arg(1)->Arg(10)->Arg(100)->Arg(500);

static void BM_PP_den520_scen4(benchmark::State& state)
{
  BM_SIPP_helper(state, "den520d.map", "den520d-random-4.scen");
}
BENCHMARK(BM_PP_den520_scen4)->Arg(1)->Arg(10)->Arg(100)->Arg(500);

static void BM_PP_den520_scen5(benchmark::State& state)
{
  BM_SIPP_helper(state, "den520d.map", "den520d-random-5.scen");
}
BENCHMARK(BM_PP_den520_scen5)->Arg(1)->Arg(10)->Arg(100)->Arg(500);

BENCHMARK_MAIN();
