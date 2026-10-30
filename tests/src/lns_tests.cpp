/*
 * Author: Jan Chleboun
 * Date: 12-03-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */


#include <gtest/gtest.h>

#include <climits>

#include "Instance.h"
#include "LNS.h"
#include "SafeIntervalTable.h"
#include "test_utils.h"


// test destroy operators
TEST(LNSDestroyOperators, RightNumberOfDestroyed)
{
  // load instance
  int                       agent_num = 50;
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance  = std::make_unique<Instance>(base_path + "/tests/test_maps/den520d.map",
                                                                  base_path + "/tests/test_scen/den520d-random-0.scen", agent_num);

  std::array<int, 5> neighborhood_sizes = {2, 4, 8, 16, 32};

  // find initial solution
  for (auto destroy_type : magic_enum::enum_values<DESTROY_TYPE>())
  {
    for (auto neighborhood_size : neighborhood_sizes)
    {
      auto         rnd_generator = std::mt19937(0);
      LNS_settings lns_settings(0, 5, {destroy_type, neighborhood_size}, {SIPP_implementation::SIPP_mine, INFO_type::no_info, 1.0});
      LNS          lns(*instance, rnd_generator, nullptr, lns_settings);
      lns.find_initial_solution();
      lns.initialize_constraint_table(lns.solution.paths);
      ASSERT_TRUE(lns.solution.feasible) << "Initial solution is not feasible";
      ASSERT_TRUE(lns.solution.is_valid(*instance)) << "Initial solution is not valid";

      // destroy agents
      lns.destroy_operator.apply(lns.solution);

      // check if the right number of agents were destroyed
      EXPECT_TRUE(static_cast<int>(lns.solution.destroyed_paths.size()) <= neighborhood_size);
    }
  }
}

TEST(LNSDestroyRandomWalk, TabuList)
{
  // load instance
  int                       agent_num = 2;
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance  = std::make_unique<Instance>(base_path + "/tests/test_maps/den520d.map",
                                                                  base_path + "/tests/test_scen/den520d-random-0.scen", agent_num);

  auto         rnd_generator = std::mt19937(0);
  LNS_settings lns_settings(0, 5, {DESTROY_TYPE::RANDOMWALK, 1}, {SIPP_implementation::SIPP_mine, INFO_type::no_info, 1.0});
  LNS          lns(*instance, rnd_generator, nullptr, lns_settings);
  lns.find_initial_solution();
  lns.initialize_constraint_table(lns.solution.paths);
  ASSERT_TRUE(lns.solution.feasible) << "Initial solution is not feasible";
  ASSERT_TRUE(lns.solution.is_valid(*instance)) << "Initial solution is not valid";

  // destroy agents
  lns.destroy_operator.apply(lns.solution);
  ASSERT_EQ(lns.solution.destroyed_paths.size(), 1);
  int destroyed_1 = lns.solution.destroyed_paths[0];

  // destroy again
  lns.destroy_operator.apply(lns.solution);
  ASSERT_EQ(lns.solution.destroyed_paths.size(), 1);
  int destroyed_2 = lns.solution.destroyed_paths[0];

  // check that the same agent was not destroyed twice
  EXPECT_NE(destroyed_1, destroyed_2);

  // check, that the tabu list reset
  lns.destroy_operator.apply(lns.solution);
  ASSERT_EQ(lns.solution.destroyed_paths.size(), 1);
}
