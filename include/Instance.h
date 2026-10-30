/*
 * Author: Jan Chleboun
 * Date: 08-01-2025
 * Email: chlebja3@fel.cvut.cz
 * Description: A class that represents an instance of the MAPF problem and related functions.
 */

#pragma once
#include <string>
#include <vector>

#include "Map.h"
#include "utils.h"

// #define CALCULATE_OTHER_HEURISTICS

class Instance
{
public:
  // constructor
  Instance() = default;
  Instance(std::string map_fname_, std::string scene_fname_, int num_of_agents_ = 0, bool calculate_manhattan = false,
           bool calculate_euclidean = false);

  void initialize(const std::string& map_fname, const std::string& scene_fname, int num_of_agents = 0, bool calculate_manhattan = false,
                  bool calculate_euclidean = false);
  void initialize(bool calculate_manhattan, bool calculate_euclidean);
  void calculate_heuristics(bool calculate_manhattan, bool calculate_euclidean);
  void precompute_neighbors();
  [[nodiscard]] auto path_to_pointpath(const Path& path) const -> PointPath;
  [[nodiscard]] auto check_timepointpath_validity(const TimePointPath& tp_path) const -> bool;
  void               reset();
  // function to print the agents starts and goal positions
  void print_agents() const;

  [[nodiscard]] auto is_initialized() const -> bool
  {
    return initialized;
  }

  // function to print the map
  void print_map() const
  {
    assertm(initialized, "Instance not initialized.");
    map_data.print();
  }


  // function to obtain the map data
  [[nodiscard]] inline auto get_map_data() const -> const Map&
  {
    assertm(initialized, "Instance not initialized.");
    return map_data;
  }

  // functions to obtain the agent starts, goals and optimal paths
  [[nodiscard]] auto get_start_positions() const -> const std::vector<Point2d>&
  {
    assertm(initialized, "Instance not initialized.");
    return start_positions;
  }

  [[nodiscard]] inline auto get_start_locations() const -> const std::vector<int>&
  {
    assertm(initialized, "Instance not initialized.");
    return start_locations;
  }

  [[nodiscard]] auto get_goal_positions() const -> const std::vector<Point2d>&
  {
    assertm(initialized, "Instance not initialized.");
    return goal_positions;
  }
  [[nodiscard]] inline auto get_goal_locations() const -> const std::vector<int>&
  {
    assertm(initialized, "Instance not initialized.");
    return goal_locations;
  }

#ifdef CALCULATE_OTHER_HEURISTICS
  [[nodiscard]] inline auto get_heuristic_manhattan(const int agent_num, const int loc) const -> int
  {
    assertm(initialized, "Instance not initialized.");
    assertm(agent_num >= 0 && agent_num < (int)heuristic_manhattan.size(), "Agent number out of range of the heuristic vector.");
    assertm(map_data.is_in(loc), "Trying to index a point that is not in the map.");
    return heuristic_manhattan[agent_num][loc];
  };

  [[nodiscard]] auto get_heuristic_euclidean(int agent_num, int loc) const -> double
  {
    assertm(initialized, "Instance not initialized.");
    assertm(agent_num >= 0 && agent_num < (int)heuristic_euclidean.size(), "Agent number out of range of the heuristic vector.");
    assertm(map_data.is_in(loc), "Trying to index a point that is not in the map.");
    return heuristic_euclidean[agent_num][loc];
  }
#endif

  [[nodiscard]] inline auto get_heuristic_distance(const int agent_num, const int loc) const -> int
  {
    assertm(agent_num >= 0 && agent_num < (int)heuristic_distance.size(), "Agent number out of range of the heuristic vector.");
    assertm(map_data.is_in(loc), "Trying to index a point that is not in the map.");
    return heuristic_distance[agent_num][loc];
  };

  [[nodiscard]] inline auto get_neighbor_locations(int loc) const -> const std::vector<int>&
  {
    assertm(map_data.is_in(loc), "Trying to get neighbors of invalid location.");
    assertm(loc < (int)neighbors.size(), "Neighbors are not precomputed for this location.");
    return neighbors[loc];
  }

  [[nodiscard]] inline auto get_sum_of_distances() const -> int
  {
    assertm(initialized, "Instance not initialized.");
    return sum_of_distances;
  }

  [[nodiscard]] auto location_to_position(int location) const -> Point2d
  {
    assertm(initialized, "Instance not initialized.");
    return Point2d(location % map_data.width, (int)(location / map_data.width));
  }

  [[nodiscard]] auto position_to_location(const Point2d& position) const -> int
  {
    assertm(initialized, "Instance not initialized.");
    return position.y * map_data.width + position.x;
  }


  [[nodiscard]] inline auto get_num_cells() const -> int
  {
    return map_data.get_num_cells();
  }

  [[nodiscard]] inline auto get_num_free_cells() const -> int
  {
    assertm(initialized, "Instance not initialized.");
    return map_data.get_num_free_cells();
  }

  [[nodiscard]] inline auto is_goal_location(int loc) const -> bool
  {
    assertm(initialized, "Instance not initialized.");
    assertm(map_data.is_in(loc), "Invalid location.");
    return location_to_goal_array[loc] != -1;
  }

  [[nodiscard]] inline auto whose_goal(int loc) const -> int
  {
    assertm(initialized, "Instance not initialized.");
    assertm(map_data.is_in(loc), "Invalid location.");
    assertm(loc >= 0 && loc < static_cast<int>(location_to_goal_array.size()), "Location outside the vector size");
    return location_to_goal_array[loc];
  }

  [[nodiscard]] inline auto get_num_of_agents() const -> int
  {
    assertm(initialized, "Instance not initialized.");
    return num_of_agents;
  }

  [[nodiscard]] inline auto location_to_free_location(int loc) const -> int
  {
    assertm(initialized, "Instance not initialized.");
    return map_data.location_to_free_location(loc);
  }

  [[nodiscard]] inline auto free_location_to_location(int free_loc) const -> int
  {
    assertm(initialized, "Instance not initialized.");
    return map_data.free_location_to_location(free_loc);
  }

private:
  /**
   * @brief Load the map from a file specified during Instance initialization.
   *
   * @return Success of the operation
   */
  void load_map();

  // function to load a scene
  void load_scene();

  void calculate_distance_heuristic();

  bool                 initialized = false;
  std::string          map_fname;
  std::string          scene_fname;
  Map                  map_data;
  std::vector<Point2d> start_positions;
  std::vector<int>     start_locations;
  std::vector<Point2d> goal_positions;
  std::vector<int>     goal_locations;          // goal location for each agent
  std::vector<int>     location_to_goal_array;  // goal location for each location
  // std::vector<Path>             optimal_paths;
  std::vector<std::vector<int>> neighbors;

  // heuristics
#ifdef CALCULATE_OTHER_HEURISTICS
  std::vector<std::vector<int>>    heuristic_manhattan;
  std::vector<std::vector<double>> heuristic_euclidean;
#endif
  std::vector<std::vector<int>> heuristic_distance;

  int sum_of_distances = 0;
  int num_of_agents;
};

