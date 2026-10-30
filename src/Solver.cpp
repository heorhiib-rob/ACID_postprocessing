/*
 * Author: Jan Chleboun
 * Date: 08-01-2025
 * Email: chlebja3@fel.cvut.cz
 * Description:
 */

#include "Solver.h"

#include <boost/algorithm/string_regex.hpp>
#include <fstream>
#include <iostream>
#include <sstream>
#include <unordered_set>
#include <unordered_map>
#include <utility>

auto operator<<(std::ostream& os, Solution const& sol) -> std::ostream&
{

  std::stringstream output;
  output << "Solution:" << std::endl;
  for(unsigned i = 0; i < sol.paths.size(); ++i){
    output << "Agent " << i << ": ";
    for(unsigned j = 0; j < sol.paths[i].size(); ++j){
      output << sol.paths[i][j].location << ", "; 
      if(sol.paths[i][j].interval.col){
        output << "C!";
      }
      output << "(" << sol.paths[i][j].interval.t_min << ", "; 
      if(sol.paths[i][j].interval.t_max < INT_MAX){
      output << sol.paths[i][j].interval.t_max << "); ";
      } else {
      output << "inf); ";
      }
    }
    output << std::endl;
  }

  os << output.str();
  return os;
}

auto Solution::is_valid(const Instance& instance) const -> bool
{
  // check that each agent has a path
  if (instance.get_num_of_agents() != (int)paths.size())
  {
    // std::cout << "Agent without a path." << std::endl;
    return false;
  }

  convert_paths();

  for (int i = 0; i < (int)paths.size(); i++)
  {
    // check validity of each timepointpath
    if (!instance.check_timepointpath_validity(paths[i]))
    {
      return false;
    }

    // convert the path
    converted_paths[i] = timepointpath_to_path(paths[i]);
  }

  // check that each path starts and ends in the correct position and find the longest path
  int T_max = 0;
  for (int i = 0; i < (int)converted_paths.size(); i++)
  {
    // check that no path has length 0
    if (converted_paths[i].empty())
    {
      // std::cout << "Empty path." << std::endl;
      return false;
    }

    // check that the path starts in the start position
    if (!(converted_paths[i].front() == instance.get_start_locations()[i]))
    {
      // std::cout << "Path does not start in the start position." << std::endl;
      return false;
    }

    // check that the path ends in the goal position
    if (!(converted_paths[i].back() == instance.get_goal_locations()[i]))
    {
      // std::cout << "Path does not end in the goal position." << std::endl;
      return false;
    }

    // remember the length of the longest path
    if ((int)converted_paths[i].size() > T_max)
    {
      T_max = converted_paths[i].size();
    }
  }
  //  std::cout << "Max length of path: " << T_max << std::endl;

  // now check that there are no collisions or invalid points or invalid moves
  for (int t = 0; t < T_max; t++)
  {
    // create reservation set for each time step to check for dynamic coliisions
    std::unordered_set<int, std::hash<int>> occupied;

    for (const auto& path : converted_paths)
    {
      int       idx          = t;
      const int cur_path_len = (int)path.size();

      // paths that are already finished at time t
      if (t >= cur_path_len)
      {
        // stay at the last point
        idx = cur_path_len - 1;
      }

      if (idx > 0)
      {
        // check if the move is valid
        Point2d p1 = instance.location_to_position(path[idx - 1]);
        Point2d p2 = instance.location_to_position(path[idx]);
        if (abs(p2.x - p1.x) + abs(p2.y - p1.y) > 1)
        {
          // std::cout << "Invalid move." << std::endl;
          return false;
        }
      }

      // check the path does not leave the map
      if (!instance.get_map_data().is_in(path[idx]))
      {
        // std::cout << "Path leaves the map in point " << path[idx] << std::endl;
        return false;
      }

      // check collisions with static obstacles
      if (instance.get_map_data().index(path[idx]) != 0)
      {
        // std::cout << "Collision with static obstacle." << std::endl;
        return false;
      }

      // check collisions with dynamic obstacles
      if (occupied.find(path[idx]) != occupied.end())
      {
        // std::cout << "Collision with dynamic obstacle." << std::endl;
        return false;
      }

      // add the point to the reservation set
      occupied.insert(path[idx]);
    }
  }

  // check that the agents do not swap edges
  for (int i = 0; i < (int)converted_paths.size(); i++)
  {
    for (int j = i + 1; j < (int)converted_paths.size(); j++)
    {
      for (int t = 1; t < T_max; t++)
      {
        const int cur_path_len_i = (int)converted_paths[i].size();
        const int cur_path_len_j = (int)converted_paths[j].size();

        // paths that are already finished at time t
        if (t >= cur_path_len_i || t >= cur_path_len_j)
        {
          continue;
        }

        if (converted_paths[i][t] == converted_paths[j][t - 1] && converted_paths[i][t - 1] == converted_paths[j][t])
        {
          // std::cout << "Agents swap edges." << std::endl;
          return false;
        }
      }
    }
  }

  return true;
}

auto Solution::get_collision_graph() const -> std::vector<std::set<int>>
{
  std::vector<std::set<int>> collision_graph(paths.size());
  convert_paths();

  // find the longest path
  int T_max = 0;
  for (int i = 0; i < (int)converted_paths.size(); i++)
  {
    T_max = std::max(T_max, (int)converted_paths[i].size());
  }

  // agents in each location at the previous time step
  std::unordered_map<int, std::vector<int>> agents_at_previous;

  for (int t = 0; t < T_max; t++)
  {
    // agents in each location at time t, finished agents stay at their last location
    std::unordered_map<int, std::vector<int>> agents_at;
    for (int i = 0; i < (int)converted_paths.size(); i++)
    {
      const int idx = std::min(t, (int)converted_paths[i].size() - 1);
      agents_at[converted_paths[i][idx]].push_back(i);
    }

    for (int i = 0; i < (int)converted_paths.size(); i++)
    {
      const Path& path = converted_paths[i];
      const int   idx  = std::min(t, (int)path.size() - 1);

      // agents in the same location at the same time
      for (int j : agents_at[path[idx]])
      {
        if (j != i)
        {
          collision_graph[i].insert(j);
        }
      }

      // agents swapping places with agent i: agent i moves from "from" to "to", the other agent from "to" to "from"
      if (t == 0 || t >= (int)path.size() || path[t - 1] == path[t])
      {
        continue;
      }
      const int from = path[t - 1];
      const int to   = path[t];
      auto      it   = agents_at_previous.find(to);
      if (it == agents_at_previous.end())
      {
        continue;
      }
      for (int j : it->second)
      {
        const int j_idx = std::min(t, (int)converted_paths[j].size() - 1);
        if (converted_paths[j][j_idx] == from)
        {
          collision_graph[i].insert(j);
        }
      }
    }

    agents_at_previous = std::move(agents_at);
  }

  return collision_graph;
}

void Solution::load(const std::string& filename, const Instance& instance)
{
  // open the solution file
  std::ifstream sol_file(filename.c_str());

  // check if the file is open
  if (!sol_file.is_open())
  {
    throw std::runtime_error("Cannot open file '" + filename + "'");
  }

  // read agents one by one
  std::string line;
  int         expected_agent_num = 0;

  while (getline(sol_file, line))
  {
    // remove leading and trailing spaces
    line = boost::algorithm::trim_copy(line);

    // find the agent number
    const size_t idx_an = line.find("Agent ");
    if (idx_an == std::string::npos)
    {
      throw std::runtime_error("Cannot find agent number in line '" + line + "'");
    }

    const size_t idx2_an = line.find(':');
    if (idx_an == std::string::npos)
    {
      throw std::runtime_error("Cannot find : in line '" + line + "'");
    }

    // get the agent number
    int agent_num = 0;
    try
    {
      agent_num = std::stoi(line.substr(idx_an + 6, idx2_an - idx_an - 6));
    }
    catch (const std::exception& e)
    {
      throw std::runtime_error("Cannot convert agent number in line '" + line + "'");
    }

    // check that the agent number is correct
    if (agent_num != expected_agent_num)
    {
      throw std::runtime_error("Unexpected agent number in line '" + line + "'");
    }
    expected_agent_num++;

    // split the line
    std::vector<std::string> split_line;
    boost::algorithm::split_regex(split_line, line.substr(idx2_an + 1), boost::regex("->"));

    // read the path
    Path path;
    for (const std::string& point_str : split_line)
    {
      if (point_str.empty())
      {
        continue;
      }
      // remove the leading and trailing spaces
      std::string point_str_trimmed = boost::algorithm::trim_copy(point_str);

      // find the opening bracket of the point
      size_t idx = point_str_trimmed.find('(');
      if (idx == std::string::npos)
      {
        throw std::runtime_error("Cannot find '(' in point '" + point_str + "'");
      }

      // find the closing bracket of the point
      size_t idx2 = point_str_trimmed.find(')');
      if (idx2 == std::string::npos)
      {
        throw std::runtime_error("Cannot find ')' in point '" + point_str + "'");
      }

      // find ,
      size_t idx_comma = point_str_trimmed.find(',');
      if (idx_comma == std::string::npos)
      {
        throw std::runtime_error("Cannot find ',' in point '" + point_str + "'");
      }

      // get the coordinates
      int x, y;
      try
      {
        // load in format (y,x)
        y = std::stoi(point_str_trimmed.substr(idx + 1, idx_comma - 1));
        x = std::stoi(point_str_trimmed.substr(idx_comma + 1, idx2 - idx_comma - 1));
      }
      catch (const std::exception& e)
      {
        throw std::runtime_error("Cannot convert coordinates in line '" + point_str_trimmed +
                                 "', x: " + point_str_trimmed.substr(idx + 1, idx_comma - 1) +
                                 ", y: " + point_str_trimmed.substr(idx_comma + 1, idx2 - idx_comma - 1));
      }

      // add the point to the path
      path.push_back(instance.position_to_location(Point2d(x, y)));
    }
    // add the path to the solution
    paths.push_back(path_to_timepointpath(path));
  }

  // close the file
  sol_file.close();

  // check whether the solution is feasible
  feasible = is_valid(instance);
}

void Solution::save(const std::string& filename, const Instance& instance) const
// save in the same format as MAPF-LNS2
{
  std::ofstream output;
  output.open(filename);

  // convert the trajectories to paths
  convert_paths();

  // iterate over the converted paths
  for (int i = 0; i < static_cast<int>(converted_paths.size()); i++)
  {
    // write agent number
    output << "Agent " << i << ":";

    // iterate over the path
    for (const auto& it : converted_paths[i])
    {
      // write the position
      Point2d pt = instance.location_to_position(it);
      output << "(" << pt.y << "," << pt.x << ")->";
    }
    output << std::endl;
  }
  output.close();
}

void Solution::calculate_cost(const Instance& instance)
{
  if (!feasible)
  {
    delays.resize(paths.size(), -1);
    sum_of_delays = -1;
    sum_of_costs  = -1;
    makespan      = -1;
    return;
  }
  assertm(feasible, "Can not calculate cost of infeasible solution");
  assertm(!paths.empty(), "Can not calculate cost of an empty solution.");
  delays.resize(paths.size(), 0);
  sum_of_delays = 0;
  sum_of_costs  = 0;
  makespan      = paths[0].back().interval.t_min;

  for (int i = 0; i < static_cast<int>(paths.size()); i++)
  {
    makespan = std::max(makespan, paths[i].back().interval.t_min);
    sum_of_costs += paths[i].back().interval.t_min;
    const int delay = paths[i].back().interval.t_min - instance.get_heuristic_distance(i, instance.get_start_locations()[i]);
    sum_of_delays += delay;
    assertm(delay >= 0, "Delay can not be negative.");
    assertm(sum_of_costs >= delay && sum_of_costs >= makespan, "Sum of costs should be greater than delay and makespan.");
    delays[i] = delay;
  }
}

void Solution::convert_paths() const
{
  converted_paths.clear();
  converted_paths.resize(paths.size());
  for (int i = 0; i < static_cast<int>(paths.size()); i++)
  {
    converted_paths[i] = timepointpath_to_path(paths[i]);
  }
}
