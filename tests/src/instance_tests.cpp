/*
 * Author: Jan Chleboun
 * Date: 19-02-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#include <gtest/gtest.h>
#include <math.h>
#include <stdint.h>

#include <iostream>
#include <numeric>

#include "Instance.h"
#include "test_utils.h"

// Test initialization when instance loaded
TEST(InstanceTest, Initialized)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 1);
  // check whether the instance is initialized
  EXPECT_TRUE(instance->is_initialized());
}

// Check initialization when empty instance is created
TEST(InstanceTest, NotInitialized)
{
  // create empty instance
  std::unique_ptr<Instance> instance = std::make_unique<Instance>();

  // check whether the instance is initialized
  EXPECT_FALSE(instance->is_initialized());
}

// Test manual initialization
TEST(InstanceTest, ManualInitialization)
{
  // create empty instance
  std::unique_ptr<Instance> instance = std::make_unique<Instance>();

  // initialize instance
  std::string base_path = get_base_path_tests();  // path to my_solver
  instance->initialize(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 1);

  // check whether the instance is initialized
  EXPECT_TRUE(instance->is_initialized());
}

// Test map loading
TEST(InstanceTest, LoadMap)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 1);

  // check whether the map looks as expected
  std::vector<uint8_t> expected_map = {0, 0, 0, 0, 1, 0, 0, 0, 0};
  const Map&           loaded_map   = instance->get_map_data();

  // check that the loaded map is right
  EXPECT_EQ(loaded_map.width, 3) << "Width of the map is not correct";
  EXPECT_EQ(loaded_map.height, 3) << "Height of the map is not correct";
  EXPECT_EQ(loaded_map.get_num_free_cells(), 8) << "Number of free cells is not correct";

  EXPECT_EQ(loaded_map.data, expected_map) << "Map is not correct";
}

// Test scene loading
TEST(InstanceTest, LoadScene)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 3);

  // check that the number of agents is right
  EXPECT_EQ(instance->get_num_of_agents(), 3);

  // check that the start positions are right
  const std::vector<Point2d> expected_starts = {{0, 0}, {2, 2}, {1, 2}};
  EXPECT_EQ(instance->get_start_positions(), expected_starts);

  // check that the goal positions are right
  const std::vector<Point2d> expected_goals = {{2, 2}, {0, 0}, {2, 1}};
  EXPECT_EQ(instance->get_goal_positions(), expected_goals);
}

// Check manhattan distance heuristics
#ifdef CALCULATE_OTHER_HEURISTICS
TEST(InstanceTest, ManhattanDistance)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance  = std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map",
                                                                  base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 3, true);

  // expected heuristics for each agent
  std::vector<std::vector<int>> expected_heuristics = {
      {4, 3, 2, 3, 2, 1, 2, 1, 0}, {0, 1, 2, 1, 2, 3, 2, 3, 4}, {3, 2, 1, 2, 1, 0, 3, 2, 1}};
  // check with the calculated
  for (int i = 0; i < instance->get_num_of_agents(); i++)
  {
    for (int loc = 0; loc < instance->get_map_data().height * instance->get_map_data().width; loc++)
    {
      EXPECT_EQ(instance->get_heuristic_manhattan(i, loc), expected_heuristics[i][loc]) << "Heuristic is not correct";
    }
  }
}

// Check euclidean distance heuristics
TEST(InstanceTest, EuclideanDistance)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance  = std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map",
                                                                  base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 3, false, true);

  // expected heuristics for each agent
  std::vector<std::vector<double>> expected_heuristics = {
      {std::sqrt(8.0), std::sqrt(5.0), 2.0, std::sqrt(5.0), std::sqrt(2.0), 1.0, 2.0, 1.0, 0.0},
      {0, 1, 2, 1, std::sqrt(2.0), std::sqrt(5.0), 2.0, std::sqrt(5.0), std::sqrt(8.0)},
      {std::sqrt(5.0), std::sqrt(2.0), 1.0, 2.0, 1.0, 0.0, std::sqrt(5.0), std::sqrt(2.0), 1.0}};
  // check with the calculated
  for (int i = 0; i < instance->get_num_of_agents(); i++)
  {
    for (int loc = 0; loc < instance->get_map_data().height * instance->get_map_data().width; loc++)
    {
      EXPECT_EQ(instance->get_heuristic_euclidean(i, loc), expected_heuristics[i][loc]) << "Heuristic is not correct";
    }
  }
}
#endif

// Test position-location conversion
// Position to location
TEST(InstanceTest, PositionToLocation)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/empty_5_5.map", base_path + "/tests/test_scen/empty_5_5_scen_1.scen", 1);

  std::vector<int> calculated_locations;
  // check with the calculated
  for (int y = 0; y < instance->get_map_data().height; y++)
  {
    for (int x = 0; x < instance->get_map_data().width; x++)
    {
      calculated_locations.push_back(instance->position_to_location(Point2d(x, y)));
    }
  }

  // construct the expected locations
  std::vector<int> expected_locations(calculated_locations.size());
  std::iota(expected_locations.begin(), expected_locations.end(), 0);

  // check equality
  EXPECT_EQ(calculated_locations, expected_locations) << "Locations are not correct";
}

// Location to position
TEST(InstanceTest, LocationToPosition)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/empty_5_5.map", base_path + "/tests/test_scen/empty_5_5_scen_1.scen", 1);

  std::vector<Point2d> calculated_positions;
  // check with the calculated
  for (int i = 0; i < instance->get_map_data().height * instance->get_map_data().width; i++)
  {
    calculated_positions.push_back(instance->location_to_position(i));
  }

  // construct the expected positions
  std::vector<Point2d> expected_positions;
  for (int y = 0; y < instance->get_map_data().height; y++)
  {
    for (int x = 0; x < instance->get_map_data().width; x++)
    {
      expected_positions.push_back(Point2d(x, y));
    }
  }

  // check equality
  EXPECT_EQ(calculated_positions, expected_positions) << "Positions are not correct";
}

// Check that location - to position - to location is identity
TEST(InstanceTest, LocationPositionIdentity)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/empty_5_5.map", base_path + "/tests/test_scen/empty_5_5_scen_1.scen", 1);

  // check with the calculated
  for (int i = 0; i < instance->get_map_data().height * instance->get_map_data().width; i++)
  {
    EXPECT_EQ(i, instance->position_to_location(instance->location_to_position(i))) << "Location to position to location is not identity";
  }
}

// Check that position - to location - to position is identity
TEST(InstanceTest, PositionLocationIdentity)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/empty_5_5.map", base_path + "/tests/test_scen/empty_5_5_scen_1.scen", 1);

  // check with the calculated
  for (int y = 0; y < instance->get_map_data().height; y++)
  {
    for (int x = 0; x < instance->get_map_data().width; x++)
    {
      EXPECT_EQ(Point2d(x, y), instance->location_to_position(instance->position_to_location(Point2d(x, y))))
          << "Position to location to position is not identity";
    }
  }
}

// Test Path to PointPath
TEST(InstanceTest, PathToPointPath)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/empty_5_5.map", base_path + "/tests/test_scen/empty_5_5_scen_1.scen", 1);

  // create path
  std::vector<int> path = {0, 1, 2, 7, 12, 11, 10, 5, 6};

  // convert path to point path
  std::vector<Point2d> point_path = instance->path_to_pointpath(path);

  // check that the path is correct
  std::vector<Point2d> expected_point_path = {{0, 0}, {1, 0}, {2, 0}, {2, 1}, {2, 2}, {1, 2}, {0, 2}, {0, 1}, {1, 1}};
  EXPECT_EQ(point_path, expected_point_path) << "Path to point path is not correct";
}

// Test distance heuristic
TEST(InstanceTest, DistanceHeuristic)
{
  // load instance
  std::string               base_path = get_base_path_tests();  // path to my_solver
  std::unique_ptr<Instance> instance =
      std::make_unique<Instance>(base_path + "/tests/test_maps/dummy_3_3.map", base_path + "/tests/test_scen/dummy_3_3_scen_1.scen", 3);

  // goal positions are {2, 2}, {0, 0}, {2, 1}

  // expected heuristics for each agent
  std::vector<std::vector<int>> expected_heuristics = {
      {4, 3, 2, 3, -1, 1, 2, 1, 0}, {0, 1, 2, 1, -1, 3, 2, 3, 4}, {3, 2, 1, 4, -1, 0, 3, 2, 1}};

  // check with the calculated
  for (int i = 0; i < instance->get_num_of_agents(); i++)
  {
    for (int loc = 0; loc < instance->get_map_data().height * instance->get_map_data().width; loc++)
    {
      EXPECT_EQ(instance->get_heuristic_distance(i, loc), expected_heuristics[i][loc]) << "Heuristic is not correct";
    }
  }
}

