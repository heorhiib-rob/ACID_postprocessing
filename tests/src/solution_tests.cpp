/*
 * Author: Jan Chleboun
 * Date: 12-02-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#include <gtest/gtest.h>

#include "Instance.h"
#include "Solver.h"
#include "test_utils.h"
#include "utils.h"

// Test solution loading
TEST(SolutionTest, LoadSolutionDummy)
{
  // load solution
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 1);
  Solution sol;
  sol.load(base_path + "/tests/test_solutions/dummy_1.sol", *instance);

  Path pth0 = timepointpath_to_path(sol.paths[0]);
  Path pth1 = timepointpath_to_path(sol.paths[1]);

  // check that both agents have paths
  EXPECT_EQ(sol.paths.size(), 2);
  // check size of paths
  EXPECT_EQ(pth0.size(), 5);
  EXPECT_EQ(pth1.size(), 5);
  // check that paths are correct
  EXPECT_EQ(pth0[0], 0);
  EXPECT_EQ(pth0[1], 3);
  EXPECT_EQ(pth0[2], 6);
  EXPECT_EQ(pth0[3], 7);
  EXPECT_EQ(pth0[4], 8);
  EXPECT_EQ(pth1[0], 8);
  EXPECT_EQ(pth1[1], 5);
  EXPECT_EQ(pth1[2], 2);
  EXPECT_EQ(pth1[3], 1);
  EXPECT_EQ(pth1[4], 0);
}

TEST(SolutionTest, LoadSolutionByLNS)
{
  // load solution
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 1);
  Solution sol;
  sol.load(base_path + "/tests/test_solutions/den520d_scen0_500agents.sol", *instance);

  // check that all agents have paths
  EXPECT_EQ(sol.paths.size(), 500);

  // check start of first path
  EXPECT_EQ(sol.paths[0].front().location, instance->position_to_location(Point2d(61, 174)));

  // check end of first path
  EXPECT_EQ(sol.paths[0].back().location, instance->position_to_location(Point2d(132, 100)));

  // check start of last path
  EXPECT_EQ(sol.paths[499].front().location, instance->position_to_location(Point2d(207, 190)));

  // check end of last path
  EXPECT_EQ(sol.paths[499].back().location, instance->position_to_location(Point2d(179, 175)));
}

TEST(SolutionTest, SaveSolution)
{
  // load solution
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/den520d.map", base_path + "/tests/test_scen/den520d-random-0.scen", 500);
  Solution sol;
  sol.load(base_path + "/tests/test_solutions/den520d_scen0_500agents.sol", *instance);

  // the solution should be feasible
  ASSERT_TRUE(sol.is_valid(*instance));

  // save the solution
  sol.save(base_path + "/tests/test_solutions/generated.sol", *instance);

  // load the saved solution
  Solution sol2;
  sol2.load(base_path + "/tests/test_solutions/generated.sol", *instance);

  // check whether the solution is the same
  EXPECT_EQ(sol.paths, sol2.paths);
  EXPECT_EQ(sol.feasible, sol2.feasible);
}

// Test validity check

// Test Case 1: Valid Solution (No Collisions, Correct Paths)
TEST(SolutionTest, ValidSolution)
{
  // load instance
  std::string     base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 2);


  Solution sol;
  sol.paths = {
      path_to_timepointpath({0, 1, 2, 5, 8}),  // Agent 1
      path_to_timepointpath({8, 7, 6, 3, 0})   // Agent 2
  };

  EXPECT_TRUE(sol.is_valid(*instance));
}

// Test Case 2: Invalid Solution (Wrong Start Position)
TEST(SolutionTest, InvalidStartPosition)
{
  // load instance
  std::string     base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 2);

  Solution sol;
  sol.paths = {path_to_timepointpath({1, 2, 5, 8}),  // Agent 1 starts at wrong position
               path_to_timepointpath({8, 7, 6, 3, 0})};

  EXPECT_FALSE(sol.is_valid(*instance));
}

// Test Case 3: Invalid Solution (Wrong Goal Position)
TEST(SolutionTest, InvalidGoalPosition)
{
  // load instance
  std::string     base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 2);

  Solution sol;
  sol.paths = {path_to_timepointpath({0, 1, 2, 5, 4}),  // Agent 1 does not reach goal
               path_to_timepointpath({8, 7, 6, 3, 0})};

  EXPECT_FALSE(sol.is_valid(*instance));
}

// Test Case 4: Invalid Solution (Collision at Same Position)
TEST(SolutionTest, InvalidCollisionSamePosition)
{
  // load instance
  std::string     base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 2);

  Solution sol;
  sol.paths = {
      path_to_timepointpath({0, 1, 2, 5, 8}),  // Agent 1
      path_to_timepointpath({8, 5, 2, 1, 0})   // Collision at (2,0) at t=2
  };

  EXPECT_FALSE(sol.is_valid(*instance));
}

// Test Case 5: Invalid Solution (Edge Swap Collision)
TEST(SolutionTest, InvalidEdgeSwapCollision)
{
  // load instance
  std::string     base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 2);

  Solution sol;
  sol.paths = {
      path_to_timepointpath({0, 1, 2, 5, 8}),    // Agent 1
      path_to_timepointpath({8, 5, 5, 2, 1, 0})  // Agents swap places at t=3
  };

  EXPECT_FALSE(sol.is_valid(*instance));
}

// Test Case 6: Invalid Solution (Passing Through Obstacle)
TEST(SolutionTest, InvalidThroughObstacle)
{
  // load instance
  std::string     base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 2);

  Solution sol;
  sol.paths = {path_to_timepointpath({0, 3, 4, 7, 8}),  // Moves through obstacle
               path_to_timepointpath({8, 7, 6, 3, 0})};

  EXPECT_FALSE(sol.is_valid(*instance));
}

// Test Case 7: Empty Paths (No Solution Found)
TEST(SolutionTest, NoSolution)
{
  // load instance
  std::string     base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 2);

  Solution sol;
  sol.paths = {};  // No paths

  EXPECT_FALSE(sol.is_valid(*instance));
}

// Test Case 8: One empty Path
TEST(SolutionTest, AgentWithoutPath)
{
  // load instance
  std::string     base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 2);

  Solution sol;
  sol.paths = {path_to_timepointpath({0, 1, 2, 5, 8}),  // agent 1
               {}};                                     // agent 2 has no path

  EXPECT_FALSE(sol.is_valid(*instance));
}


// Test Case 9: Agent Moves Diagonally (Invalid Move)
TEST(SolutionTest, InvalidDiagonalMove)
{
  // load instance
  std::string     base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/empty_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 2);

  Solution sol;
  sol.paths = {path_to_timepointpath({0, 4, 8}),  // Moves diagonally
               path_to_timepointpath({8, 7, 6, 3, 0})};

  EXPECT_FALSE(sol.is_valid(*instance));
}

// Test Case 10: Agent Moves Outside Map to negative (Out of Bounds)
TEST(SolutionTest, InvalidMoveOutsideMapNeg)
{
  // load instance
  std::string     base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 2);

  Solution sol;
  sol.paths = {path_to_timepointpath({0, -1, 0, 1, 2, 5, 8}),  // Moves out of the map (negative x)
               path_to_timepointpath({8, 7, 6, 3, 0})};

  EXPECT_FALSE(sol.is_valid(*instance));
}


// Test Case 11: Agent Moves Outside Map to positive (Out of Bounds)
TEST(SolutionTest, InvalidMoveOutsideMapPos)
{
  // load instance
  std::string     base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 2);

  Solution sol;
  sol.paths = {path_to_timepointpath({0, 1, 2, 5, 8, 9, 8}),  // Moves out of the map
               path_to_timepointpath({8, 7, 6, 3, 0})};

  EXPECT_FALSE(sol.is_valid(*instance));
}

// Test Case 12: Agent Moves More Than One Cell Per Step (Invalid Jump)
TEST(SolutionTest, InvalidJumpMove)
{
  // load instance
  std::string     base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 2);

  Solution sol;
  sol.paths = {path_to_timepointpath({0, 2, 8}),  // Jumps from (0,0) to (2,0)
               path_to_timepointpath({8, 7, 6, 3, 0})};

  EXPECT_FALSE(sol.is_valid(*instance));
}

// Test Case 13: Large instance solution generated by MAPF-LNS
TEST(SolutionTest, LargeInstanceValid)
{
  // load instance
  std::string     base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/den520d.map", base_path + "/tests/test_scen/den520d-random-0.scen", 500);

  // load solution by MAPF-LNS
  Solution sol;
  sol.load(base_path + "/tests/test_solutions/den520d_scen0_500agents.sol", *instance);

  EXPECT_TRUE(sol.is_valid(*instance));
}

