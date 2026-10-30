/*
 * Author: Jan Chleboun
 * Date: 16-02-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#include <gtest/gtest.h>

#include <climits>

#include "Instance.h"
#include "LNS.h"
#include "SIPP.h"
#include "SafeIntervalTable.h"
#include "magic_enum/magic_enum.hpp"
#include "test_utils.h"

// SIPP tests

// Basic pathfinding in an empty map
TEST(SIPPTest, BasicPathEmptyMap)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/empty_5_5.map", base_path + "/tests/test_scen/empty_5_5_scen_1.scen", 1);

  // create SIPP
  for (auto algo : magic_enum::enum_values<SIPP_implementation>())
  {
    std::mt19937  rnd_generator(0);
    SIPP_settings sipp_settings(algo, INFO_type::no_info, 1.0);
    SIPP          sipp(*instance, rnd_generator, sipp_settings);

    // from (0,0) to (4,4)
    TimePointPath path = sipp.plan(0, {});

    ASSERT_EQ(path.size(), 9) << "SIPP failed to find a basic path!";
    EXPECT_EQ(path.front().location, instance->get_start_locations()[0]) << "Path does not start at the correct point!";
    EXPECT_EQ(path.back().location, instance->get_goal_locations()[0]) << "Path does not end at the correct point!";
  }
}

// Pathfinding with static obstacles
TEST(SIPPTest, StaticObstacleAvoidance)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/wall_5_5.map", base_path + "/tests/test_scen/wall_5_5_scen_1.scen", 1);

  // create SIPP
  for (auto algo : magic_enum::enum_values<SIPP_implementation>())
  {
    std::mt19937  rnd_generator(0);
    SIPP_settings sipp_settings(algo, INFO_type::no_info, 1.0);
    SIPP          sipp(*instance, rnd_generator, sipp_settings);

    // from (0,0) to (4,4)
    TimePointPath path = sipp.plan(0, {});

    // check that the path size is correct
    ASSERT_EQ(path.size(), 9) << "SIPP failed to find a path around obstacles!";

    // check that the path starts and end at the correct points
    EXPECT_EQ(path.front().location, instance->get_start_locations()[0]) << "Path does not start at the correct point!";
    EXPECT_EQ(path.back().location, instance->get_goal_locations()[0]) << "Path does not end at the correct point!";

    // check that no obstacles are in the path
    for (const auto& tp : path)
    {
      EXPECT_TRUE(instance->get_map_data().index(tp.location) == 0) << "Path goes through an obstacle!";
    }
  }
}

// Handling of time constraints (SIPP-specific test)**
TEST(SIPPTest, TimeConstraintHandling)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/empty_5_5.map", base_path + "/tests/test_scen/empty_5_5_scen_1.scen", 1);

  // create SIPP
  for (auto algo : magic_enum::enum_values<SIPP_implementation>())
  {
    std::mt19937  rnd_generator(0);
    SIPP_settings sipp_settings(algo, INFO_type::no_info, 1.0);
    SIPP          sipp(*instance, rnd_generator, sipp_settings);

    // Create a wall from time constraints
    for (int i = 0; i < 5; i++)
    {
      if (i == 2)
      {
        continue;  // passage through (2,2)
      }
      sipp.safe_interval_table.add_constraint(TimePoint(instance->position_to_location({i, 2}), {0, 20}));
    }

    TimePointPath path = sipp.plan(0, {});

    ASSERT_EQ(path.size(), 9) << "Path length invalid!";

    // check that the path does not go through the blocked area
    for (const auto& tp : path)
    {
      if (instance->location_to_position(tp.location).y == 2)
      {
        EXPECT_EQ(instance->location_to_position(tp.location).x, 2) << "Path goes through a blocked area!";
      }
    }

    // check the completion time of the path
    EXPECT_EQ(path.back().interval.t_min, 8) << "Path does not complete in the expected time!";
  }
}

// No possible path due to obstacles
TEST(SIPPTest, NoPathPossible)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/biparted_5_5.map", base_path + "/tests/test_scen/empty_5_5_scen_1.scen", 1);

  // create SIPP
  for (auto algo : magic_enum::enum_values<SIPP_implementation>())
  {
    std::mt19937  rnd_generator(0);
    SIPP_settings sipp_settings(algo, INFO_type::no_info, 1.0);
    SIPP          sipp(*instance, rnd_generator, sipp_settings);

    TimePointPath path = sipp.plan(0, {});

    ASSERT_TRUE(path.empty()) << "SIPP should return an empty path when no path exists!";
  }
}

// No path due to time constraints**
TEST(SIPPTest, NoPathDueToTimeConstraints)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/empty_5_5.map", base_path + "/tests/test_scen/empty_5_5_scen_1.scen", 1);

  // create SIPP
  for (auto algo : magic_enum::enum_values<SIPP_implementation>())
  {
    std::mt19937  rnd_generator(0);
    SIPP_settings sipp_settings(algo, INFO_type::no_info, 1.0);
    SIPP          sipp(*instance, rnd_generator, sipp_settings);

    // Add constraints that make it impossible to leave start until time 10
    sipp.safe_interval_table.add_constraint(TimePoint(1, {0, 10}));  // {1, 0}
    sipp.safe_interval_table.add_constraint(TimePoint(5, {0, 10}));  // {0, 1}

    // add constraints that make it impossible to reach the goal from time 10 to 20
    sipp.safe_interval_table.add_constraint(TimePoint(instance->position_to_location({3, 4}), {10, 20}));  // {3, 4}
    sipp.safe_interval_table.add_constraint(TimePoint(instance->position_to_location({4, 3}), {10, 20}));  // {4, 3}

    // test path len estimate
    EXPECT_EQ(sipp.safe_interval_table.get_max_path_len_estimate(), 20 + 5 * 5) << "Invalid path length estimate.";

    // plan path
    TimePointPath path = sipp.plan(0, {});

    // check that path was found
    ASSERT_FALSE(path.empty()) << "SIPP returned no path!";

    // check that the path does not leave the start until time 10
    for (const auto& tp : path)
    {
      if (tp.location != instance->get_start_locations()[0])
      {
        EXPECT_TRUE(tp.interval.t_min > 10) << "Path leaves start before time 10!";
      }
    }

    // check that the path does not reach the goal before time 21
    for (const auto& tp : path)
    {
      if (tp.location == instance->get_goal_locations()[0])
      {
        EXPECT_TRUE(tp.interval.t_min > 20) << "Path reaches goal before or at time 20!";
      }
    }
  }
}

// dynamic obstacle at the goal after time of possible arrival**
TEST(SIPPTest, DynamicObstacleAtGoal)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/empty_5_5.map", base_path + "/tests/test_scen/empty_5_5_scen_1.scen", 1);

  // create SIPP
  for (auto algo : magic_enum::enum_values<SIPP_implementation>())
  {
    std::mt19937  rnd_generator(0);
    SIPP_settings sipp_settings(algo, INFO_type::no_info, 1.0);
    SIPP          sipp(*instance, rnd_generator, sipp_settings);

    // add a constraint that blocks the goal after time 20
    sipp.safe_interval_table.add_constraint(TimePoint(instance->get_goal_locations()[0], {20, 30}));

    TimePointPath path = sipp.plan(0, {});

    // the path should be found
    ASSERT_FALSE(path.empty()) << "SIPP should find a path when the goal is blocked after possible arrival time!";

    // check that the path reaches the goal at time 31
    EXPECT_EQ(path.back().interval.t_min, 31) << "Path does not reach the goal at the expected time!";
  }
}

// test that SIPP takes into account the edge constraints
// Edge constraints at the goal
TEST(SIPPTest, EdgeConstraintsAtGoal)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_2.scen", 1);

  // create SIPP
  for (auto algo : magic_enum::enum_values<SIPP_implementation>())
  {
    std::mt19937  rnd_generator(0);
    SIPP_settings sipp_settings(algo, INFO_type::no_info, 1.0);
    SIPP          sipp(*instance, rnd_generator, sipp_settings);

    // from (2, 0) to (2, 1)

    // add edge constraints at the goal so that the agent has to go around
    sipp.safe_interval_table.add_constraints({TimePoint(5, {0, 0}), TimePoint(2, {1, 1}), TimePoint(1, {2, 2}), TimePoint(0, {3, 3}),
                                              TimePoint(3, {4, 4}), TimePoint(6, {5, 5}), TimePoint(7, {6, 6}), TimePoint(8, {7, 7})});


    TimePointPath path = sipp.plan(0, {});

    // check that the path has the right length
    ASSERT_EQ(path.size(), 8) << "SIPP should find a path.";

    // check that the path is correct
    EXPECT_EQ(path, TimePointPath({TimePoint(2, {0, 0}), TimePoint(1, {1, 1}), TimePoint(0, {2, 2}), TimePoint(3, {3, 3}),
                                   TimePoint(6, {4, 4}), TimePoint(7, {5, 5}), TimePoint(8, {6, 6}), TimePoint(5, {7, INT_MAX})}));
  }
}


// SIPP plans the same path as A* when no constraints are present
TEST(SIPPTest, SamePathAsAStar1)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/empty_5_5.map", base_path + "/tests/test_scen/empty_5_5_scen_1.scen", 1);

  // create SIPP
  for (auto algo : magic_enum::enum_values<SIPP_implementation>())
  {
    std::mt19937  rnd_generator(0);
    SIPP_settings sipp_settings(algo, INFO_type::no_info, 1.0);
    SIPP          sipp(*instance, rnd_generator, sipp_settings);

    // from (0,0) to (4,4)

    // plan path by ASTAR
    Path path_astar = find_path_distance_gradient(0, *instance);

    // assert that astar found a path
    ASSERT_FALSE(path_astar.empty()) << "A* should find a path when no constraints are present!";

    // plan path by SIPP
    TimePointPath path_sipp = sipp.plan(0, {});

    // assert that SIPP found a path
    ASSERT_FALSE(path_sipp.empty()) << "SIPP should find a path when no constraints are present!";

    // convert SIPP path to the same format as A*
    Path path_sipp_converted = timepointpath_to_path(path_sipp);

    // check that the paths have the same length
    EXPECT_EQ(path_sipp_converted.size(), path_astar.size()) << "The paths found by SIPP and ASTAR should have the same length!";
  }
}

// SIPP plans the same path as A* when no dynamic constraints are present
TEST(SIPPTest, SamePathAsAStar2)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/den520d.map", base_path + "/tests/test_scen/den520d-random-0.scen", 1);

  // create SIPP
  for (auto algo : magic_enum::enum_values<SIPP_implementation>())
  {
    std::mt19937  rnd_generator(0);
    SIPP_settings sipp_settings(algo, INFO_type::no_info, 1.0);
    SIPP          sipp(*instance, rnd_generator, sipp_settings);

    // from (0,0) to (4,4)

    // plan path by ASTAR
    Path path_astar = find_path_distance_gradient(0, *instance);

    // assert that astar found a path
    ASSERT_FALSE(path_astar.empty()) << "A* should find a path when no constraints are present!";

    // plan path by SIPP
    TimePointPath path_sipp = sipp.plan(0, {});

    // assert that SIPP found a path
    ASSERT_FALSE(path_sipp.empty()) << "SIPP should find a path when no constraints are present!";

    // convert SIPP path to the same format as A*
    Path path_sipp_converted = timepointpath_to_path(path_sipp);

    // check that the paths have the same length
    EXPECT_EQ(path_sipp_converted.size(), path_astar.size()) << "The paths found by SIPP and ASTAR should have the same length!";
  }
}

// Test Prioritized Planning
// Prioritized Planning with dummy map and 1 agent
TEST(SIPPTest, PrioritizedPlanning1agent)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 1);

  // create LNS


  // plan path by Prioritized Planning
  for (auto algo : magic_enum::enum_values<SIPP_implementation>())
  {
    auto         rnd_generator = std::mt19937(0);
    LNS_settings lns_settings(0, 5, {DESTROY_TYPE::RANDOM, 10}, {algo, INFO_type::no_info, 1.0});
    LNS          lns(*instance, rnd_generator, nullptr, lns_settings);
    Solution     sol = lns.PrioritizedPlanning();
    // check solution validity
    EXPECT_TRUE(sol.is_valid(*instance)) << "Solution is invalid!";
  }
}

// Prioritized Planning with empty map and 2 agents
TEST(SIPPTest, PrioritizedPlanningEmpty2agents)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/empty_5_5.map", base_path + "/tests/test_scen/empty_5_5_scen_1.scen", 2);


  // plan path by Prioritized Planning
  for (auto algo : magic_enum::enum_values<SIPP_implementation>())
  {
    // create LNS
    auto         rnd_generator = std::mt19937(0);
    LNS_settings lns_settings(0, 5, {DESTROY_TYPE::RANDOM, 10}, {algo, INFO_type::no_info, 1.0});
    LNS          lns(*instance, rnd_generator, nullptr, lns_settings);
    Solution     sol = lns.PrioritizedPlanning();

    // check solution validity
    EXPECT_TRUE(sol.is_valid(*instance)) << "Solution is invalid!";
  }
}

// Test Prioritized Planning
TEST(SIPPTest, PrioritizedPlanning)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/den520d.map", base_path + "/tests/test_scen/den520d-random-0.scen", 100);


  // plan path by Prioritized Planning
  for (auto algo : magic_enum::enum_values<SIPP_implementation>())
  {
    // create LNS
    auto         rnd_generator = std::mt19937(0);
    LNS_settings lns_settings(0, 5, {DESTROY_TYPE::RANDOM, 10}, {algo, INFO_type::no_info, 1.0});
    LNS          lns(*instance, rnd_generator, nullptr, lns_settings);
    Solution     sol = lns.PrioritizedPlanning();
    // check solution validity
    EXPECT_TRUE(sol.is_valid(*instance)) << "Solution is invalid!";
  }
}

TEST(SIPPTest, SameAfterDestroy)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/den520d.map", base_path + "/tests/test_scen/den520d-random-0.scen", 200);


  // plan path by Prioritized Planning
  for (auto algo : magic_enum::enum_values<SIPP_implementation>())
  {
    // create LNS
    auto         rnd_generator = std::mt19937(0);
    LNS_settings lns_settings(0, 5, {DESTROY_TYPE::RANDOM, 10}, {algo, INFO_type::no_info, 1.0});
    LNS          lns(*instance, rnd_generator, nullptr, lns_settings);
    Solution     sol = lns.PrioritizedPlanning();
    // check solution validity
    EXPECT_TRUE(sol.is_valid(*instance)) << "Solution is invalid!";
    int           agent_num = 100;
    TimePointPath path0     = sol.paths[agent_num];
    // remove the path
    lns.planner->safe_interval_table.remove_constraints(path0);
    lns.already_planned.erase(agent_num);

    // replan
    TimePointPath path1 = lns.planner->plan(agent_num, lns.already_planned);

    // expect the same path
    EXPECT_EQ(path0.back().interval.t_min, path1.back().interval.t_min) << "Paths should have the same length!";
  }
}
