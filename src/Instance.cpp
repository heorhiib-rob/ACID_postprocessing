/*
 * Author: Jan Chleboun
 * Date: 08-01-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#include "Instance.h"

#include <bits/stdc++.h>

#include <boost/algorithm/string.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <utility>

Instance::Instance(std::string map_fname_, std::string scene_fname_, int num_of_agents_, bool calculate_manhattan, bool calculate_euclidean)
    : num_of_agents(num_of_agents_), map_fname(std::move(map_fname_)), scene_fname(std::move(scene_fname_))
{
  initialize(calculate_manhattan, calculate_euclidean);
}

void Instance::initialize(const std::string& map_fname_, const std::string& scene_fname_, int num_of_agents_, bool calculate_manhattan,
                          bool calculate_euclidean)
{
  map_fname     = map_fname_;
  scene_fname   = scene_fname_;
  num_of_agents = num_of_agents_;
  initialize(calculate_manhattan, calculate_euclidean);
}

void Instance::initialize(bool calculate_manhattan, bool calculate_euclidean)
{
  assertm(!initialized, "Instance already initialized.");
  load_map();
  load_scene();
  precompute_neighbors();
  calculate_heuristics(calculate_manhattan, calculate_euclidean);
  // calculate sum of distances
  sum_of_distances = 0;
  for (int i = 0; i < num_of_agents; i++)
  {
    sum_of_distances += get_heuristic_distance(i, start_locations[i]);
  }

  // prepare location to goal conversion
  location_to_goal_array.resize(get_num_cells(), -1);  // initialize to -1 which means not a goal location
  for (int i = 0; i < static_cast<int>(goal_locations.size()); i++)
  {
    // for each location remember the number of agent whose goal it is
    location_to_goal_array[goal_locations[i]] = i;
  }

  initialized = true;
}

void Instance::reset()
{
  assertm(initialized, "Instance already reset.");
  num_of_agents = 0;
  initialized   = false;
  map_fname     = "";
  scene_fname   = "";
  map_data      = Map();
  start_positions.clear();
  start_locations.clear();
  goal_positions.clear();
  goal_locations.clear();
  location_to_goal_array.clear();
  // optimal_paths.clear();
  neighbors.clear();
#ifdef CALCULATE_OTHER_HEURISTICS
  heuristic_manhattan.clear();
  heuristic_euclidean.clear();
#endif
  heuristic_distance.clear();
  sum_of_distances = 0;
}

void Instance::load_map()
{
  // reset the map data
  Map new_map;
  try
  {
    new_map.load(map_fname);
  }
  catch (const std::exception& e)
  {
    throw std::runtime_error("Unable to load map: '" + std::string(e.what()) + "'");
  }
  map_data = std::move(new_map);
}

void Instance::load_scene()
{
  // open the scene file
  std::ifstream scene_file(scene_fname.c_str());

  // check if the open was successful
  if (!scene_file.is_open())
  {
    throw std::runtime_error("Unable to open scene file.");
  }

  std::string line;

  // read the version
  if (!getline(scene_file, line) || line != "version 1")
  {
    throw std::invalid_argument("Invalid version of scene file.");
  }

  int loaded_agents = 0;
  start_positions.reserve(num_of_agents);
  goal_positions.reserve(num_of_agents);
  // read the file line by line until enough agents were loaded
  while (loaded_agents < num_of_agents && getline(scene_file, line))
  {
    // split the line - the format is 'bucket, map, map_width, map_height, start_x, start_y, goal_x, goal_y, optimal_length'
    std::vector<std::string> split_line;
    boost::split(split_line, line, boost::is_any_of("\t "));

    // filter out invalid lines
    if (split_line.size() != 9)
    {
      std::cout << "Invalid line in scen file: " << line << " length: " << split_line.size() << std::endl;
      continue;
    }

    // read the start and goal positions
    try
    {
      const int start_x = stoi(split_line[4]);
      const int start_y = stoi(split_line[5]);
      const int goal_x  = stoi(split_line[6]);
      const int goal_y  = stoi(split_line[7]);

      // store the positions in the list
      start_positions.emplace_back(start_x, start_y);
      start_locations.push_back(map_data.position_to_index(start_positions.back()));
      goal_positions.emplace_back(goal_x, goal_y);
      goal_locations.push_back(map_data.position_to_index(goal_positions.back()));
      loaded_agents++;
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error("Could not read start and goal positions: '" + (std::string)e.what() + "'");
    }
  }

  // cloase the map file
  scene_file.close();

  // check whether enough agents were loaded
  if (loaded_agents != num_of_agents)
  {
    std::cout << "WARNING: Unable to load " << num_of_agents << " agents, as only " << loaded_agents
              << " agents are available in the scene file." << std::endl;
    num_of_agents = loaded_agents;
  }
}

void Instance::print_agents() const
{
  for (int i = 0; i < (int)start_positions.size(); i++)
  {
    std::cout << "Agent " << i << ": \t start (" << start_positions[i].x << "," << start_positions[i].y << ") \t"
              << "goal (" << goal_positions[i].x << "," << goal_positions[i].y << ")" << std::endl;
  }
}

void Instance::calculate_heuristics(bool calculate_manhattan, bool calculate_euclidean)
{

  // check map validity
  assertm(map_data.width * map_data.height > 0, "Invalid map.");
  assertm((int)map_data.data.size() == map_data.width * map_data.height, "Map size invalid");
  assertm(map_data.get_num_free_cells() > 0 && map_data.get_num_free_cells() <= (int)map_data.data.size(), "Invalid free cell count.");
#ifndef CALCULATE_OTHER_HEURISTICS
  assertm(calculate_euclidean == false || calculate_manhattan == false, "Heuristics turned off during copimle.");
#endif

  const std::vector<int> int_initializer(map_data.width * map_data.height, -1);
#ifdef CALCULATE_OTHER_HEURISTICS
  if (calculate_euclidean)
  {
    // clear the vectors
    heuristic_euclidean.clear();
    // preallocate vectors for the heuristics
    const std::vector<double> euclidean_initializer(map_data.width * map_data.height, 0.0);
    heuristic_euclidean.resize(num_of_agents, euclidean_initializer);
  }

  if (calculate_manhattan)
  {
    heuristic_manhattan.clear();
    heuristic_manhattan.resize(num_of_agents, int_initializer);
  }


  // fill the vectors
  if (calculate_euclidean || calculate_manhattan)
  {
#pragma omp parallel for collapse(2)
    for (int i = 0; i < num_of_agents; i++)  // iterate over agents
    {
      for (int y = 0; y < map_data.height; y++)  // iterate over all positions
      {
        const Point2d& goal = goal_positions[i];
        for (int x = 0; x < map_data.width; x++)
        {
          if (calculate_manhattan)
          {
            heuristic_manhattan[i][y * map_data.width + x] = abs(goal.x - x) + abs(goal.y - y);
          }
          if (calculate_euclidean)
          {
            heuristic_euclidean[i][y * map_data.width + x] = std::hypot(goal.x - x, goal.y - y);
          }
        }
      }
    }
  }
#endif

  // precompute the distance heuristic
  heuristic_distance.clear();
  heuristic_distance.resize(num_of_agents, int_initializer);
  calculate_distance_heuristic();
}

void Instance::precompute_neighbors()
{
  // check map validity
  assertm(map_data.width * map_data.height > 0, "Invalid map.");
  assertm((int)map_data.data.size() == map_data.width * map_data.height, "Map size invalid");

  // clear the previous neighbors
  neighbors.clear();
  neighbors.resize(map_data.width * map_data.height, {});

  // iterate over all locations
  for (int i = 0; i < map_data.width * map_data.height; i++)
  {
    if (map_data.index(i) != 0)
    {
      continue;
    }
    neighbors[i] = map_data.find_neighbors(i);
  }
}

void Instance::calculate_distance_heuristic()
{
  // calculate distance from each goal for each position
#pragma omp parallel for
  for (int i = 0; i < (int)goal_locations.size(); i++)
  {
    const int         goal = goal_locations[i];
    std::vector<int>& curr = heuristic_distance[i];

    // by default the first integer is used as priority, and the elements with highest priority are popped first
    std::priority_queue<std::pair<int, int>> open_list;

    // create first node which is the goal
    curr[goal] = 0;
    open_list.push({0, goal});

    // find distance from the goal for each position
    while (!open_list.empty())
    {
      // pop the element with the highest priority
      auto [priority, location] = open_list.top();
      open_list.pop();

      // find the neighbor locations
      const std::vector<int>& curr_neighbors = get_neighbor_locations(location);
      for (int neighbor : curr_neighbors)
      {
        // if the neighbor is not visited yet and is free cell
        if (curr[neighbor] == -1 && map_data.index(neighbor) == 0)
        {
          // calculate the distance
          curr[neighbor] = curr[location] + 1;
          open_list.push({-curr[neighbor], neighbor});  // use negative priority to get the elements closest to goal first
        }
      }
    }
  }
}


auto Instance::path_to_pointpath(const Path& path) const -> PointPath
{
  PointPath ret;
  for (const auto& loc : path)
  {
    ret.push_back(location_to_position(loc));
  }
  return ret;
}


auto Instance::check_timepointpath_validity(const TimePointPath& tp_path) const -> bool
{
  // check whether all locations are neighbors
  if (tp_path.size() >= 2)
  {
    // check first location
    if (!map_data.is_in(tp_path.front().location))
    {
      return false;
    }

    // check other locations
    for (int i = 0; i < (int)tp_path.size() - 1; i++)
    {
      Point2d p1 = location_to_position(tp_path[i].location);
      Point2d p2 = location_to_position(tp_path[i + 1].location);
      if (!map_data.is_in(p2))
      {
        return false;
      }

      if (std::abs(p1.x - p2.x) + std::abs(p1.y - p2.y) != 1)
      {
        return false;
      }
    }
  }
  // perform the other validity checks
  return is_valid_timepointpath(tp_path);
}
