/*
 * Author: Jan Chleboun
 * Date: 09-02-2025
 * Email: chlebja3@fel.cvut.cz
 * Description: Implementation of the SIPP algorithm for path planning in a grid map. The algorithm is based on the A* search algorithm, but
 * searches also in the time dimension. To reduce dimensionality of the space, the time is discretized into intervals.
 */

#include "SIPP.h"

#include <climits>
#include <cmath>
#include <memory>
#include <random>

#include "utils.h"

#define NODE_POOL_MERGE_SIZE 500  // merge the extra list with the node pool vector only of there are more elements in the list
#define INITIAL_NODE_POOL_SIZE 2  // the multiple of map size to use as initial node pool size

using namespace sipp;

SIPP::SIPP(const Instance& instance_, std::mt19937& rnd_generator_, const SIPP_settings& settings_)
    : safe_interval_table(SafeIntervalTable(instance_)),
      instance(instance_),
      node_pool(NodePool(INITIAL_NODE_POOL_SIZE * instance.get_num_cells())),
      rnd_generator(rnd_generator_),
      settings(settings_)
{
  known_max.resize(instance.get_num_cells(), -1);
  known_min.resize(instance.get_num_cells(), -1);
  sipps_nodes_at.resize(instance.get_num_cells());
  // TODO maybe only remember free cells?
}


auto SIPPNode::operator==(const SIPPNode& other) const -> bool
{
  return g == other.g && h == other.h && time_point == other.time_point;
}

auto SIPPNodeComparator::operator()(SIPPNode const* n1, SIPPNode const* n2) const -> bool
{
  // compare f
  if (n1->f == n2->f)
  {
    if (n1->h == n2->h)
    {  // tie in reach time
      if (n1->h2 == n2->h2)
      {  // tie in distance from goal
        if (n1->h3 == n2->h3)
        {
          // if tie in all heuristics, choose randomly
          return dist(*rnd_generator);
        }
        // if f is the same and h is the same and h2 is the same prefer locations that dont block goals of other agents
        return n1->h3 > n2->h3;
      }
      return n1->h2 > n2->h2;  // if f is the same and h is the same prefer nodes closer to goal
    }
    return n1->h > n2->h;  // if f is the same, prefer earlier intervals
  }
  return n1->f > n2->f;
}


auto SIPPSNodeComparator::operator()(SIPPNode const* n1, SIPPNode const* n2) const -> bool
{
  // compare number of collisions
  if (n1->num_col == n2->num_col){
    // compare f
    if (n1->f == n2->f)
    {
      if (n1->h == n2->h)
      {  // tie in reach time
        if (n1->h2 == n2->h2)
        {  // tie in distance from goal
          if (n1->h3 == n2->h3)
          {
            // if tie in all heuristics, choose randomly
            return dist(*rnd_generator);
          }
          // if f is the same and h is the same and h2 is the same prefer locations that dont block goals of other agents
          return n1->h3 > n2->h3;
        }
        return n1->h2 > n2->h2;  // if f is the same and h is the same prefer nodes closer to goal
      }
      return n1->h > n2->h;  // if f is the same, prefer earlier intervals
    }
    return n1->f > n2->f;
  }
  return n1->num_col > n2->num_col;
}

auto SIPPNodeComparatorSuboptimal::operator()(SIPPNode const* n1, SIPPNode const* n2) const -> bool
{
  // compare f
  if (std::abs(n1->f - n2->f) <= max_suboptimality_absolute)
  {
    if (n1->h == n2->h)
    {  // tie
      if (n1->h2 == n2->h2)
      {
        if (n1->f == n2->f)
        {
          if (n1->h3 == n2->h3)
          {
            // if tie, choose randomly
            return dist(*rnd_generator);
          }
          return n1->h3 > n2->h3;
        }
        return n1->f > n2->f;
      }
      return n1->h2 > n2->h2;
    }
    return n1->h > n2->h;
  }
  return n1->f > n2->f;
}


NodePool::NodePool(const int size)
{
  // initialize the pool with dummy nodes
  pool.resize(size, SIPPNode(0, {0, 0}, 0, 0, 0, 0, nullptr));
  end = 0;
}

auto NodePool::add_node(SIPPNode node) -> SIPPNode*
{
  // check, whether there is still space in the node pool
  if (end < (int)pool.size())
  {
    // move to the end of the pool and increment the count
    pool[end++] = std::move(node);
    return &pool[end - 1];
  }

  // if there is no space in the vector, add to the list, which will be merged with the vector at the end of the iteration
  extra.push_back(node);
  return &extra.back();
}


void NodePool::merge_extra()
{
  if (extra.size() > NODE_POOL_MERGE_SIZE)
  {
    // merge the list with the vector
    // be careful, this invalidates all pointers to the nodes in the node pool
    pool.insert(pool.end(), std::move_iterator{extra.begin()}, std::move_iterator{extra.end()});
    extra.clear();
  }
  // reset the node counter
  end = 0;
}

auto SIPP::plan(int agent_num, const std::unordered_set<int>& already_planned) -> TimePointPath
{
  // reset vector of blocked counts
  // if (settings.generate_blocked)
  // {
  //   blocked_counts.resize(0, instance.get_num_of_agents());
  // }
  // plan with SIPPS while the solution still has collisions
  if (safe_interval_table.allow_soft_collisions)
  {
    return plan_sipps(agent_num);
  }

  if (settings.implementation == SIPP_implementation::SIPP_suboptimal)
  {
    // suboptimal sipp
    return plan_suboptimal(agent_num, already_planned, settings.w, false);
  }

  if (settings.implementation == SIPP_implementation::SIPP_suboptimal_ap)
  {
    // suboptimal sipp
    return plan_suboptimal(agent_num, already_planned, settings.w, true);
  }

  assertm(settings.w == 1.0, "Suboptimality factor must be 1 for optimal algorithms.");
  // optimal sipp
  if (settings.implementation == SIPP_implementation::SIPP_mine)
  {
    return plan_sipp_mine(agent_num);
  }

  if (settings.implementation == SIPP_implementation::SIPP_mine_ap)
  {
    return plan_sipp_mine_ap(agent_num, already_planned);
  }

  if (settings.implementation == SIPP_implementation::SIPP_mapf_lns)
  {
    return plan_mapflns_heuristic(agent_num);
  }

  throw std::runtime_error("Unknown SIPP implementation");
}

auto SIPP::plan_sipps(const int agent_num) -> TimePointPath
{
  if (settings.info_type != INFO_type::no_info)
  {
    initialize_iter_info();
  }

  const Map& map_data = instance.get_map_data();
  const int  start    = instance.get_start_locations()[agent_num];
  const int  goal     = instance.get_goal_locations()[agent_num];


  // check whether start and goal inside the map and not obstructed
  assertm(map_data.is_in(start) && map_data.is_in(goal), "Start or goal outside the map.");
  assertm(map_data.index(start) == 0 && map_data.index(goal) == 0, "Start or goal obstructed.");

  // estimate the minimum and maximum time the goal can be reached
  // const int min_time = safe_interval_table.get_min_reach_time(goal);
  const int min_time = 0;
  // const int max_time = safe_interval_table.get_max_path_len_estimate();
  // assertm(min_time >= 0 && max_time >= 0, "Time can not be negative.");

  // create openlist
  sipp::PriorityQueueSIPPS open_list{SIPPSNodeComparator(&rnd_generator)};

  // retrieve the first safe interval
  // auto time_interval_start = safe_interval_table.get_first_safe_interval(start);
  //
  std::vector<TimeInterval> obtained_sis;
  safe_interval_table.get_soft_safe_intervals(start, start, TimeInterval(0,0), obtained_sis);
  TimeInterval time_interval_start = obtained_sis[0];

  // SIPPS: clear nodes generated in the previous search
  for (int i = 0; i < (int)sipps_nodes_at.size(); i++)
  {
    sipps_nodes_at[i].clear();
  }

  // calculate the heuristic of the start node
  {
    const double h2 = instance.get_heuristic_distance(agent_num, start);
    const double h1 = std::max((double)min_time, h2);

    // create the start node
    SIPPNode start_node(start, time_interval_start, 0, h1, time_interval_start.col, h2, 0, nullptr);
    SIPPNode* start_added = node_pool.add_node(start_node); // get the pointer to the node in the node_pool
    sipps_nodes_at[start].push_back(start_added);
    open_list.push(start_added);

    if (settings.info_type != INFO_type::no_info)
    {
      generated_this_iter++;
    }
  }

  // clear known nodes database
  std::fill(known_min.begin(), known_min.end(), INT_MAX);
  std::fill(known_max.begin(), known_max.end(), -1);

  // main loop
  while (!open_list.empty())
  {
    if (settings.info_type != INFO_type::no_info)
    {
      iteration_num++;
    }

    // pop the node to be processed
    SIPPNode const* const current = open_list.top();
    open_list.pop();

    // std::cout << "\tCurrent: " << instance.location_to_position(current->time_point.location) << ", " << current->time_point.interval
    //          << ", f: " << current->f << std::endl;

    // SIPPS: skip nodes replaced by a better one
    if (current->dead)
    {
      continue;
    }

    // SIPPS: we popped a goal node with collisions that we previously put into open.
    // this means that there is no better goal to be found
    // (if there was collision-free goal node, it would have been returned already)
    if (current->is_goal)
    {
      TimePointPath ret = sipp::extract_path(current); // extract the path
      node_pool.merge_extra(); // reset the node pool

      // generate last iteration info
      if (settings.info_type != INFO_type::no_info){
        update_iter_info(*current, true);
      }

      return ret;
    }

    // SIPPS: the agent arrives at the goal from another vertex
    if (current->time_point.location == goal && !current->wait_at_goal)
    {
      // collisions with other agents passing through the goal after the agent arrived and stays there forever
      const int future_collisions = safe_interval_table.get_future_collisions(goal, current->time_point.interval.t_min);

      // no agent visits the vertex later, search finished
      // because no better goal may be found
      if (future_collisions == 0)
      {
        // extract the path
        TimePointPath ret = sipp::extract_path(current);
        ret.back().interval.t_max = INT_MAX;

        // reset the node pool
        node_pool.merge_extra();

        // generate last iteration info
        if (settings.info_type != INFO_type::no_info)
        {
          update_iter_info(*current, true);
        }

        return ret;
      }

      // otherwise store the goal node in OPEN and continue to search for one without future collisions
      SIPPNode goal_node = *current;
      goal_node.is_goal  = true;
      goal_node.num_col += future_collisions;
      goal_node.h        = 0;
      goal_node.f        = goal_node.g;
      goal_node.time_point.interval.t_max = INT_MAX;
      goal_node.time_point.interval.col   = true;
      goal_node.si_end                    = INT_MAX;

      // if we have a better goal node, filter this one out
      if (sipps_keep_node(goal_node))
      {
        SIPPNode* goal_added = node_pool.add_node(goal_node);
        sipps_nodes_at[goal].push_back(goal_added);
        open_list.push(goal_added);

        if (settings.info_type != INFO_type::no_info)
        {
          generated_this_iter++;
        }
      }
    }

    // calculate the time window to enter neighboring positions
    const TimeInterval neighbor_entry_time_interval(safe_increase(current->time_point.interval.t_min),
                                                    safe_increase(current->time_point.interval.t_max));

    // expand current node
    const std::vector<int>& neighbors = instance.get_neighbor_locations(current->time_point.location);
    for (const auto& neighbor : neighbors)
    {
      // calculate the heuristic
      double h2 = instance.get_heuristic_distance(agent_num, neighbor);

      double h3 = 0;

      // prefer non goal locations
      if (neighbor != goal && instance.is_goal_location(neighbor))
      {
        h3 = 1.0;  // this is the logically right option
                   // h2 += 1.0; // this works well but shouldnt
      }
      // std::cout << "Neighbor: " << instance.location_to_position(neighbor) << ", h2: " << h2 << std::endl;

      // retrieve the safe intervals for the neighbor

      // SIPPS: if arriving early swaps places with another agent, the intervals also contain the later arrival without the swap
      safe_interval_table.get_soft_safe_intervals(current->time_point.location, neighbor, neighbor_entry_time_interval, obtained_sis);

      // iterate through all soft safe intervals
      for (auto it = obtained_sis.begin(); it != obtained_sis.end(); it++)
      {
        TimePoint neighbor_time_point(neighbor, *it);
        // modify the interval to be valid
        if (it->t_min < neighbor_entry_time_interval.t_min)
        {
          neighbor_time_point.interval.t_min = neighbor_entry_time_interval.t_min;
        }
        assertm(neighbor_time_point.interval.t_min <= neighbor_time_point.interval.t_max, "Modifying the time interval made it invalid.");

        // skip intervals that start later then the maximum time to reach the goal
        // DOES NOT MAKE SENSE WITH SIPPS
        // if (neighbor_time_point.interval.t_min > max_time)
        // {
        //   continue;
        // }

        assertm(neighbor_time_point.interval.t_min <= neighbor_entry_time_interval.t_max, "Unreachable interval among neighbors.");


        assertm(neighbor_time_point.interval.t_min > current->time_point.interval.t_min, "Invalid interval.");

        // check whether the is goal reached in the minimal reach time
        // if (neighbor_time_point.location == goal && neighbor_time_point.interval.t_min == min_time)
        //{
        //  // add with heuristic INT_MIN so that the node is selected next
        //  open_list.push(node_pool.add_node(SIPPNode(neighbor_time_point, neighbor_time_point.interval.t_min, INT_MIN, INT_MIN,
        //  current)));
        //}
        // else
        //{
        //  // add to the open list
        //  open_list.push(node_pool.add_node(SIPPNode(neighbor_time_point, neighbor_time_point.interval.t_min, h, current)));
        //}
        double num_col = current->num_col;
        if (it->col)
        {
          num_col += 1;
        }
        if (it->edge_col)
        {
          num_col += 1;
        }

        // SIPPS heuristic:
        // h2 = distance to goal
        // h1 = includes the arrival time to the safe interval in the target node
        double h1 = h2;
        if (num_col == 0)
        {
          h1 = std::max(h2, current->f - neighbor_time_point.interval.t_min);
        }

        SIPPNode neighbor_node(neighbor_time_point, neighbor_time_point.interval.t_min, h1, num_col, h2, h3, current);
        neighbor_node.si_end = it->t_max;  // the end of the interval; the dominance check may make t_max earlier

        // keep the node only if no node generated earlier is at least as good
        if (!sipps_keep_node(neighbor_node))
        {
          continue;
        }
        SIPPNode* neighbor_added = node_pool.add_node(neighbor_node);
        sipps_nodes_at[neighbor].push_back(neighbor_added);
        open_list.push(neighbor_added);
        // std::cout << "interval:" << neighbor_time_point.interval << ", h1: " << h1 << ", h2: " << h2 << std::endl;

        if (settings.info_type != INFO_type::no_info)
        {
          generated_this_iter++;
        }
      }
    }

    // stay in the current cell into its next interval
    // only if agent stays in it until the end and doesn't have to leave earlier 
    // && the current interval is not the last one
    if (current->time_point.interval.t_max == current->si_end && current->si_end != INT_MAX)
    {
      const int location = current->time_point.location;

      // the next interval of the same cell starts right after the current one ends
      safe_interval_table.get_soft_safe_intervals(location, location, TimeInterval(current->si_end + 1, current->si_end + 1), obtained_sis);
      const TimeInterval next_interval = obtained_sis[0];
      TimePoint          wait_time_point(location, next_interval);

      double h2 = instance.get_heuristic_distance(agent_num, location);
      double h3 = 0;
      // h3: additional penalty to avoid other agents' goals
      if (location != goal && instance.is_goal_location(location))
      {
        h3 = 1.0;
      }
      double num_col = current->num_col;
      if (next_interval.col)
      {
        num_col += 1;
      }

      // SIPPS heuristic
      double h1 = h2;
      if (num_col == 0)
      {
        h1 = std::max(h2, current->f - wait_time_point.interval.t_min);
      }

      SIPPNode wait_node(wait_time_point, wait_time_point.interval.t_min, h1, num_col, h2, h3, current);
      wait_node.wait_at_goal = (location == goal);  // waiting at the goal is not a new arrival there

      // keep the node only if a similarly good or better node was not generated earlier
      if (sipps_keep_node(wait_node))
      {
        SIPPNode* wait_added = node_pool.add_node(wait_node);
        sipps_nodes_at[location].push_back(wait_added);
        open_list.push(wait_added);

        if (settings.info_type != INFO_type::no_info)
        {
          generated_this_iter++;
        }
      }
    }

    // save the iteration info
    if (settings.info_type != INFO_type::no_info)
    {
      update_iter_info(*current, false);
    }
  }
  // reset the node pool
  node_pool.merge_extra();

  return TimePointPath();
}

// decides whether a new node is worth keeping, compared with the nodes already generated
auto SIPP::sipps_keep_node(SIPPNode& new_node) -> bool
{
  std::vector<SIPPNode*>& nodes = sipps_nodes_at[new_node.time_point.location];
  for (int i = 0; i < (int)nodes.size(); i++)
  {
    SIPPNode* old_node = nodes[i];

    // only nodes of the same interval and kind are compared
    if (old_node->si_end != new_node.si_end || old_node->is_goal != new_node.is_goal ||
        old_node->wait_at_goal != new_node.wait_at_goal)
    {
      continue;
    }

    // the old node arrives no later and has no more collisions: the new node is not needed
    if (old_node->time_point.interval.t_min <= new_node.time_point.interval.t_min && old_node->num_col <= new_node.num_col)
    {
      return false;
    }

    // the new node arrives no later and has no more collisions: the old node is not needed
    if (old_node->time_point.interval.t_min >= new_node.time_point.interval.t_min && old_node->num_col >= new_node.num_col)
    {
      old_node->dead = true;
      nodes.erase(nodes.begin() + i);
      i--;
      continue;
    }

    // neither is better: the one arriving earlier (with more collisions) may only stay until the other arrives
    if (old_node->time_point.interval.t_min <= new_node.time_point.interval.t_max &&
        new_node.time_point.interval.t_min <= old_node->time_point.interval.t_max)
    {
      if (old_node->time_point.interval.t_min < new_node.time_point.interval.t_min)
      {
        old_node->time_point.interval.t_max = new_node.time_point.interval.t_min - 1;
      }
      else
      {
        new_node.time_point.interval.t_max = old_node->time_point.interval.t_min - 1;
      }
    }
  }
  return true;
}

auto SIPP::plan_sipp_mine(const int agent_num) -> TimePointPath
{
  if (settings.info_type != INFO_type::no_info)
  {
    initialize_iter_info();
  }

  const Map& map_data = instance.get_map_data();
  const int  start    = instance.get_start_locations()[agent_num];
  const int  goal     = instance.get_goal_locations()[agent_num];


  // check whether start and goal inside the map and not obstructed
  assertm(map_data.is_in(start) && map_data.is_in(goal), "Start or goal outside the map.");
  assertm(map_data.index(start) == 0 && map_data.index(goal) == 0, "Start or goal obstructed.");

  // estimate the minimum and maximum time the goal can be reached
  const int min_time = safe_interval_table.get_min_reach_time(goal);
  const int max_time = safe_interval_table.get_max_path_len_estimate();
  assertm(min_time >= 0 && max_time >= 0, "Time can not be negative.");

  // create openlist
  sipp::PriorityQueue open_list{SIPPNodeComparator(&rnd_generator)};

  // retrieve the first safe interval
  auto time_interval_start = safe_interval_table.get_first_safe_interval(start);

  // calculate the heuristic of the start node
  {
    const double h2 = instance.get_heuristic_distance(agent_num, start);
    const double h1 = std::max((double)min_time, h2);

    // create the start node
    open_list.push(node_pool.add_node(SIPPNode(start, *time_interval_start, 0, h1, 0, h2, 0, nullptr)));

    if (settings.info_type != INFO_type::no_info)
    {
      generated_this_iter++;
    }
  }

  // clear known nodes database
  std::fill(known_min.begin(), known_min.end(), INT_MAX);
  std::fill(known_max.begin(), known_max.end(), -1);

  // main loop
  while (!open_list.empty())
  {
    if (settings.info_type != INFO_type::no_info)
    {
      iteration_num++;
    }

    // pop the node to be processed
    SIPPNode const* const current = open_list.top();
    open_list.pop();

    // std::cout << "\tCurrent: " << instance.location_to_position(current->time_point.location) << ", " << current->time_point.interval
    //          << ", f: " << current->f << std::endl;

    // check whether node was expanded already (same position and tmax higher then current tmin)
    if (current->time_point.interval.t_min >= known_min[current->time_point.location] &&
        current->time_point.interval.t_min <= known_max[current->time_point.location])
    {
      continue;
    }

    // check whether goal location is reached and that there will be no more dynamic constraints at the goal location
    if (current->time_point.location == goal && current->time_point.interval.t_max == INT_MAX)
    {
      // extract the path
      TimePointPath ret = sipp::extract_path(current);

      // reset the node pool
      node_pool.merge_extra();

      // generate last iteration info
      if (settings.info_type != INFO_type::no_info)
      {
        update_iter_info(*current, true);
      }

      return ret;
    }

    // add to known nodes
    known_min[current->time_point.location] = std::min(current->time_point.interval.t_min, known_min[current->time_point.location]);
    known_max[current->time_point.location] = std::max(current->time_point.interval.t_max, known_max[current->time_point.location]);
    // std::cout << "Known min: " << known_min[current->time_point.location] << std::endl;

    // calculate the time window to enter neighboring positions
    const TimeInterval neighbor_entry_time_interval(safe_increase(current->time_point.interval.t_min),
                                                    safe_increase(current->time_point.interval.t_max));

    // expand current node
    const std::vector<int>& neighbors = instance.get_neighbor_locations(current->time_point.location);
    for (const auto& neighbor : neighbors)
    {
      // calculate the heuristic
      double h2 = instance.get_heuristic_distance(agent_num, neighbor);

      double h3 = 0;

      double num_col = 0;
      if(current->parent != nullptr){
        num_col = current->time_point.interval.col + current->parent->num_col;
      }
      // prefer non goal locations
      if (neighbor != goal && instance.is_goal_location(neighbor))
      {
        h3 = 1.0;  // this is the logically right option
                   // h2 += 1.0; // this works well but shouldnt
      }
      // std::cout << "Neighbor: " << instance.location_to_position(neighbor) << ", h2: " << h2 << std::endl;

      // retrieve the safe intervals for the neighbor

      auto [sf_start, sf_end] = safe_interval_table.get_safe_intervals(neighbor, neighbor_entry_time_interval);

      for (auto it = sf_start; it != sf_end; it++)
      {
        TimePoint neighbor_time_point(neighbor, *it);
        // modify the interval to be valid
        if (it->t_min < neighbor_entry_time_interval.t_min)
        {
          neighbor_time_point.interval.t_min = neighbor_entry_time_interval.t_min;
        }
        assertm(neighbor_time_point.interval.t_min <= neighbor_time_point.interval.t_max, "Modifying the time interval made it invalid.");

        // skip intervals that start later then the maximum time to reach the goal
        if (neighbor_time_point.interval.t_min > max_time)
        {
          continue;
        }

        // check edge collision
        if (safe_interval_table.edge_constraint_table.get(neighbor_time_point.location, current->time_point.location,
                                                          neighbor_time_point.interval.t_min))
        {
          continue;
        }

        assertm(neighbor_time_point.interval.t_min <= neighbor_entry_time_interval.t_max, "Unreachable interval among neighbors.");

        assertm(neighbor_time_point.interval.t_min <= min_time || known_min[neighbor] == INT_MAX ||
                    neighbor_time_point.interval.t_min >= known_min[neighbor],
                "The heuristic can be non consistent only for nodes starting before the goal obstruction time.");
        // check whether known
        if (neighbor_time_point.interval.t_min >= known_min[neighbor] && neighbor_time_point.interval.t_min <= known_max[neighbor])
        {
          continue;
        }


        assertm(neighbor_time_point.interval.t_min > current->time_point.interval.t_min, "Invalid interval.");

        // check whether the is goal reached in the minimal reach time
        // if (neighbor_time_point.location == goal && neighbor_time_point.interval.t_min == min_time)
        //{
        //  // add with heuristic INT_MIN so that the node is selected next
        //  open_list.push(node_pool.add_node(SIPPNode(neighbor_time_point, neighbor_time_point.interval.t_min, INT_MIN, INT_MIN,
        //  current)));
        //}
        // else
        //{
        //  // add to the open list
        //  open_list.push(node_pool.add_node(SIPPNode(neighbor_time_point, neighbor_time_point.interval.t_min, h, current)));
        //}
        const double h1 = std::max((double)min_time, neighbor_time_point.interval.t_min + h2) - neighbor_time_point.interval.t_min;
        open_list.push(node_pool.add_node(SIPPNode(neighbor_time_point, neighbor_time_point.interval.t_min, h1, num_col, h2, h3, current)));
        // std::cout << "interval:" << neighbor_time_point.interval << ", h1: " << h1 << ", h2: " << h2 << std::endl;

        if (settings.info_type != INFO_type::no_info)
        {
          generated_this_iter++;
        }
      }
    }

    // save the iteration info
    if (settings.info_type != INFO_type::no_info)
    {
      update_iter_info(*current, false);
    }
  }
  // reset the node pool
  node_pool.merge_extra();

  return TimePointPath();
}


auto SIPP::plan_sipp_mine_ap(const int agent_num, const std::unordered_set<int>& already_planned) -> TimePointPath
{
  assertm(already_planned.find(agent_num) == already_planned.end(), "Planning agent that was already planned.");

  if (settings.info_type != INFO_type::no_info)
  {
    initialize_iter_info();
  }

  const Map& map_data = instance.get_map_data();
  const int  start    = instance.get_start_locations()[agent_num];
  const int  goal     = instance.get_goal_locations()[agent_num];


  // check whether start and goal inside the map and not obstructed
  assertm(map_data.is_in(start) && map_data.is_in(goal), "Start or goal outside the map.");
  assertm(map_data.index(start) == 0 && map_data.index(goal) == 0, "Start or goal obstructed.");

  // estimate the minimum and maximum time the goal can be reached
  const int min_time = safe_interval_table.get_min_reach_time(goal);
  const int max_time = safe_interval_table.get_max_path_len_estimate();
  assertm(min_time >= 0 && max_time >= 0, "Time can not be negative.");

  // std::cout << "Agent: " << agent_num << ", min time: " << min_time << std::endl;
  // std::cout << "Min time estimate is: " << min_time << std::endl;
  // std::cout << "Max time estimate is: " << max_time << std::endl;


  // create openlist
  sipp::PriorityQueue open_list{SIPPNodeComparator(&rnd_generator)};

  // retrieve the first safe interval
  auto time_interval_start = safe_interval_table.get_first_safe_interval(start);

  // calculate the heuristic of the start node
  {
    const double h2 = instance.get_heuristic_distance(agent_num, start);
    const double h1 = std::max((double)min_time, h2);

    // create the start node
    open_list.push(node_pool.add_node(SIPPNode(start, *time_interval_start, 0, h1, h2, 0, nullptr)));

    if (settings.info_type != INFO_type::no_info)
    {
      generated_this_iter++;
    }
  }

  // clear known nodes database
  std::fill(known_min.begin(), known_min.end(), INT_MAX);
  std::fill(known_max.begin(), known_max.end(), -1);

  // main loop
  while (!open_list.empty())
  {
    if (settings.info_type != INFO_type::no_info)
    {
      iteration_num++;
    }

    // pop the node to be processed
    SIPPNode const* const current = open_list.top();
    open_list.pop();

    // std::cout << "\tCurrent: " << instance.location_to_position(current->time_point.location) << ", " << current->time_point.interval
    //          << ", f: " << current->f << std::endl;

    // check whether node was expanded already (same position and tmax higher then current tmin)
    if (current->time_point.interval.t_min >= known_min[current->time_point.location] &&
        current->time_point.interval.t_min <= known_max[current->time_point.location])
    {
      continue;
    }

    // check whether goal location is reached and that there will be no more dynamic constraints at the goal location
    if (current->time_point.location == goal && current->time_point.interval.t_max == INT_MAX)
    {
      // extract the path
      TimePointPath ret = sipp::extract_path(current);

      // reset the node pool
      node_pool.merge_extra();

      // generate last iteration info
      if (settings.info_type != INFO_type::no_info)
      {
        update_iter_info(*current, true);
      }

      return ret;
    }

    // add to known nodes
    known_min[current->time_point.location] = std::min(current->time_point.interval.t_min, known_min[current->time_point.location]);
    known_max[current->time_point.location] = std::max(current->time_point.interval.t_max, known_max[current->time_point.location]);
    // std::cout << "Known min: " << known_min[current->time_point.location] << std::endl;

    // calculate the time window to enter neighboring positions
    const TimeInterval neighbor_entry_time_interval(safe_increase(current->time_point.interval.t_min),
                                                    safe_increase(current->time_point.interval.t_max));

    // expand current node
    const std::vector<int>& neighbors = instance.get_neighbor_locations(current->time_point.location);
    for (const auto& neighbor : neighbors)
    {
      // calculate the heuristic
      double h_distance = instance.get_heuristic_distance(agent_num, neighbor);

      double h_ap_goal = 0;
      // penalize goal locations of not yet planned agents
      if (neighbor != goal && instance.is_goal_location(neighbor) &&
          already_planned.find(instance.whose_goal(neighbor)) == already_planned.end())
      {
        h_ap_goal = 1.0;
      }
      // std::cout << "Neighbor: " << instance.location_to_position(neighbor) << ", h2: " << h2 << std::endl;

      // retrieve the safe intervals for the neighbor

      auto [sf_start, sf_end] = safe_interval_table.get_safe_intervals(neighbor, neighbor_entry_time_interval);

      for (auto it = sf_start; it != sf_end; it++)
      {
        TimePoint neighbor_time_point(neighbor, *it);
        // modify the interval to be valid
        if (it->t_min < neighbor_entry_time_interval.t_min)
        {
          neighbor_time_point.interval.t_min = neighbor_entry_time_interval.t_min;
        }
        assertm(neighbor_time_point.interval.t_min <= neighbor_time_point.interval.t_max, "Modifying the time interval made it invalid.");

        // skip intervals that start later then the maximum time to reach the goal
        if (neighbor_time_point.interval.t_min > max_time)
        {
          continue;
        }

        // check edge collision
        if (safe_interval_table.edge_constraint_table.get(neighbor_time_point.location, current->time_point.location,
                                                          neighbor_time_point.interval.t_min))
        {
          continue;
        }

        assertm(neighbor_time_point.interval.t_min <= neighbor_entry_time_interval.t_max, "Unreachable interval among neighbors.");

        assertm(neighbor_time_point.interval.t_min <= min_time || known_min[neighbor] == INT_MAX ||
                    neighbor_time_point.interval.t_min >= known_min[neighbor],
                "The heuristic can be non consistent only for nodes starting before the goal obstruction time.");
        // check whether known
        if (neighbor_time_point.interval.t_min >= known_min[neighbor] && neighbor_time_point.interval.t_min <= known_max[neighbor])
        {
          continue;
        }


        assertm(neighbor_time_point.interval.t_min > current->time_point.interval.t_min, "Invalid interval.");

        // check whether the is goal reached in the minimal reach time
        // if (neighbor_time_point.location == goal && neighbor_time_point.interval.t_min == min_time)
        //{
        //  // add with heuristic INT_MIN so that the node is selected next
        //  open_list.push(node_pool.add_node(SIPPNode(neighbor_time_point, neighbor_time_point.interval.t_min, INT_MIN, INT_MIN,
        //  current)));
        //}
        // else
        //{
        //  // add to the open list
        //  open_list.push(node_pool.add_node(SIPPNode(neighbor_time_point, neighbor_time_point.interval.t_min, h, current)));
        //}
        const double h1 = std::max((double)min_time, neighbor_time_point.interval.t_min + h_distance) - neighbor_time_point.interval.t_min;
        open_list.push(
            node_pool.add_node(SIPPNode(neighbor_time_point, neighbor_time_point.interval.t_min, h1, h_ap_goal, h_distance, current)));
        // std::cout << "interval:" << neighbor_time_point.interval << ", h1: " << h1 << ", h2: " << h2 << std::endl;

        if (settings.info_type != INFO_type::no_info)
        {
          generated_this_iter++;
        }
      }
    }

    // save the iteration info
    if (settings.info_type != INFO_type::no_info)
    {
      update_iter_info(*current, false);
    }
  }
  // reset the node pool
  node_pool.merge_extra();

  return TimePointPath();
}


auto SIPP::plan_suboptimal(const int agent_num, const std::unordered_set<int>& already_planned, double w, bool ap) -> TimePointPath
{
  assertm(w >= 1.0, "Suboptimality factor must be more than 1");
  assertm(already_planned.find(agent_num) == already_planned.end(), "Planning agent that was already planned.");

  if (settings.info_type != INFO_type::no_info)
  {
    initialize_iter_info();
  }

  const Map& map_data = instance.get_map_data();
  const int  start    = instance.get_start_locations()[agent_num];
  const int  goal     = instance.get_goal_locations()[agent_num];


  // check whether start and goal inside the map and not obstructed
  assertm(map_data.is_in(start) && map_data.is_in(goal), "Start or goal outside the map.");
  assertm(map_data.index(start) == 0 && map_data.index(goal) == 0, "Start or goal obstructed.");

  // calculate the absolute suboptimality factor
  const int suboptimality_absolute = std::floor((w - 1.0) * instance.get_heuristic_distance(agent_num, start));
  assertm(w != 1.0 || suboptimality_absolute == 0, "Absolute suboptimality must be 0 when w is 1.0");

  // estimate the minimum and maximum time the goal can be reached
  const int min_time = safe_interval_table.get_min_reach_time(goal);
  // const int min_time = safe_interval_table.get_min_reach_time(goal) + suboptimality_absolute;
  const int max_time = safe_interval_table.get_max_path_len_estimate();
  assertm(min_time >= 0 && max_time >= 0, "Time can not be negative.");

  // create openlist
  sipp::PriorityQueueSuboptimal open_list{SIPPNodeComparatorSuboptimal(&rnd_generator, suboptimality_absolute)};

  // retrieve the first safe interval
  auto time_interval_start = safe_interval_table.get_first_safe_interval(start);

  // calculate the heuristic of the start node
  {
    const double h2 = instance.get_heuristic_distance(agent_num, start);
    const double h1 = std::max((double)min_time, h2);

    // create the start node
    open_list.push(node_pool.add_node(SIPPNode(start, *time_interval_start, 0, h1, h2, 0, nullptr)));

    if (settings.info_type != INFO_type::no_info)
    {
      generated_this_iter++;
    }
  }

  // clear known nodes database
  std::fill(known_min.begin(), known_min.end(), INT_MAX);
  std::fill(known_max.begin(), known_max.end(), -1);

  // main loop
  while (!open_list.empty())
  {
    if (settings.info_type != INFO_type::no_info)
    {
      iteration_num++;
    }

    // pop the node to be processed
    SIPPNode const* const current = open_list.top();
    open_list.pop();

    // std::cout << "\tCurrent: " << instance.location_to_position(current->time_point.location) << ", " << current->time_point.interval
    //          << ", f: " << current->f << std::endl;

    // check whether node was expanded already (same position and tmax higher then current tmin)
    if (current->time_point.interval.t_min >= known_min[current->time_point.location] &&
        current->time_point.interval.t_min <= known_max[current->time_point.location])
    {
      continue;
    }

    // check whether goal location is reached and that there will be no more dynamic constraints at the goal location
    if (current->time_point.location == goal && current->time_point.interval.t_max == INT_MAX)
    {
      // extract the path
      TimePointPath ret = sipp::extract_path(current);

      // reset the node pool
      node_pool.merge_extra();

      // generate last iteration info
      if (settings.info_type != INFO_type::no_info)
      {
        update_iter_info(*current, true);
      }

      return ret;
    }

    // add to known nodes
    known_min[current->time_point.location] = std::min(current->time_point.interval.t_min, known_min[current->time_point.location]);
    known_max[current->time_point.location] = std::max(current->time_point.interval.t_max, known_max[current->time_point.location]);
    // std::cout << "Known min: " << known_min[current->time_point.location] << std::endl;

    // calculate the time window to enter neighboring positions
    const TimeInterval neighbor_entry_time_interval(safe_increase(current->time_point.interval.t_min),
                                                    safe_increase(current->time_point.interval.t_max));

    // expand current node
    const std::vector<int>& neighbors = instance.get_neighbor_locations(current->time_point.location);
    for (const auto& neighbor : neighbors)
    {
      // calculate the heuristic
      // const double h = instance.get_heuristic_euclidean(agent_num, neighbor);
      // const double h = instance.get_heuristic_manhattan(agent_num, neighbor);
      double h2 = instance.get_heuristic_distance(agent_num, neighbor);
      // std::cout << "Neighbor: " << instance.location_to_position(neighbor) << ", h2: " << h2 << std::endl;

      double h3 = 0;
      if (ap)
      {
        if (neighbor != goal && instance.is_goal_location(neighbor) &&
            already_planned.find(instance.whose_goal(neighbor)) == already_planned.end())
        {
          h2 += settings.p;
        }
      }
      else
      {
        if (neighbor != goal && instance.is_goal_location(neighbor))
        {
          h2 += settings.p;
        }
      }
      // retrieve the safe intervals for the neighbor
      auto [sf_start, sf_end] = safe_interval_table.get_safe_intervals(neighbor, neighbor_entry_time_interval);

      for (auto it = sf_start; it != sf_end; it++)
      {
        TimePoint neighbor_time_point(neighbor, *it);
        // modify the interval to be valid
        if (it->t_min < neighbor_entry_time_interval.t_min)
        {
          neighbor_time_point.interval.t_min = neighbor_entry_time_interval.t_min;
        }
        assertm(neighbor_time_point.interval.t_min <= neighbor_time_point.interval.t_max, "Modifying the time interval made it invalid.");

        // skip intervals that start later then the maximum time to reach the goal
        if (neighbor_time_point.interval.t_min > max_time)
        {
          continue;
        }

        // check edge collision
        if (safe_interval_table.edge_constraint_table.get(neighbor_time_point.location, current->time_point.location,
                                                          neighbor_time_point.interval.t_min))
        {
          continue;
        }

        assertm(neighbor_time_point.interval.t_min <= neighbor_entry_time_interval.t_max, "Unreachable interval among neighbors.");

        // check whether known
        if (neighbor_time_point.interval.t_min >= known_min[neighbor] && neighbor_time_point.interval.t_min <= known_max[neighbor])
        {
          continue;
        }


        assertm(neighbor_time_point.interval.t_min > current->time_point.interval.t_min, "Invalid interval.");

        // check whether the is goal reached in the minimal reach time
        // if (neighbor_time_point.location == goal && neighbor_time_point.interval.t_min == min_time)
        //{
        //  // add with heuristic INT_MIN so that the node is selected next
        //  open_list.push(node_pool.add_node(SIPPNode(neighbor_time_point, neighbor_time_point.interval.t_min, INT_MIN, INT_MIN,
        //  current)));
        //}
        // else
        //{
        //  // add to the open list
        //  open_list.push(node_pool.add_node(SIPPNode(neighbor_time_point, neighbor_time_point.interval.t_min, h, current)));
        //}
        const double h1 = std::max((double)min_time, neighbor_time_point.interval.t_min + h2) - neighbor_time_point.interval.t_min;
        open_list.push(node_pool.add_node(SIPPNode(neighbor_time_point, neighbor_time_point.interval.t_min, h1, h2, h3, current)));
        // std::cout << "interval:" << neighbor_time_point.interval << ", h1: " << h1 << ", h2: " << h2 << std::endl;

        if (settings.info_type != INFO_type::no_info)
        {
          generated_this_iter++;
        }
      }
    }

    // save the iteration info
    if (settings.info_type != INFO_type::no_info)
    {
      update_iter_info(*current, false);
    }
  }
  // reset the node pool
  node_pool.merge_extra();

  return TimePointPath();
}


auto SIPP::plan_mapflns_heuristic(int agent_num) -> TimePointPath
{
  if (settings.info_type != INFO_type::no_info)
  {
    initialize_iter_info();
  }

  const Map& map_data = instance.get_map_data();
  const int  start    = instance.get_start_locations()[agent_num];
  const int  goal     = instance.get_goal_locations()[agent_num];

  // check whether start and goal inside the map and not obstructed
  assertm(map_data.is_in(start) && map_data.is_in(goal), "Start or goal outside the map.");
  assertm(map_data.index(start) == 0 && map_data.index(goal) == 0, "Start or goal obstructed.");

  // estimate the minimum and maximum time the goal can be reached
  const int min_time = safe_interval_table.get_min_reach_time(goal);
  const int max_time = safe_interval_table.get_max_path_len_estimate();
  assertm(min_time >= 0 && max_time >= 0, "Time can not be negative.");

  // create openlist
  sipp::PriorityQueue open_list{SIPPNodeComparator(&rnd_generator)};

  // retrieve the first safe interval
  auto time_interval_start = safe_interval_table.get_first_safe_interval(start);

  // calculate the heuristic of the start node
  {
    const double h = std::max((double)min_time, (double)instance.get_heuristic_distance(agent_num, start));

    // create the start node
    open_list.push(node_pool.add_node(SIPPNode(start, *time_interval_start, 0, h, h, 0, nullptr)));

    if (settings.info_type != INFO_type::no_info)
    {
      generated_this_iter++;
    }
  }

  // clear known nodes database
  std::fill(known_min.begin(), known_min.end(), INT_MAX);
  std::fill(known_max.begin(), known_max.end(), -1);

  // main loop
  while (!open_list.empty())
  {
    if (settings.info_type != INFO_type::no_info)
    {
      iteration_num++;
    }

    // pop the node to be processed
    SIPPNode const* const current = open_list.top();
    open_list.pop();

    // std::cout << "\tCurrent: " << instance.location_to_position(current->time_point.location) << ", " << current->time_point.interval
    //          << ", f: " << current->f << std::endl;

    // check whether node was expanded already (same position and tmax higher then current tmin)
    if (current->time_point.interval.t_min >= known_min[current->time_point.location] &&
        current->time_point.interval.t_min <= known_max[current->time_point.location])
    {
      continue;
    }

    // check whether goal location is reached and that there will be no more dynamic constraints at the goal location
    if (current->time_point.location == goal && current->time_point.interval.t_max == INT_MAX)
    {
      // extract the path
      TimePointPath ret = sipp::extract_path(current);

      // reset the node pool
      node_pool.merge_extra();

      // generate last iteration info
      if (settings.info_type != INFO_type::no_info)
      {
        update_iter_info(*current, true);
      }

      return ret;
    }

    // add to known nodes
    known_min[current->time_point.location] = std::min(current->time_point.interval.t_min, known_min[current->time_point.location]);
    known_max[current->time_point.location] = std::max(current->time_point.interval.t_max, known_max[current->time_point.location]);
    // std::cout << "Known min: " << known_min[current->time_point.location] << std::endl;

    // calculate the time window to enter neighboring positions
    const TimeInterval neighbor_entry_time_interval(safe_increase(current->time_point.interval.t_min),
                                                    safe_increase(current->time_point.interval.t_max));

    // expand current node
    const std::vector<int>& neighbors = instance.get_neighbor_locations(current->time_point.location);
    for (const auto& neighbor : neighbors)
    {
      // calculate the heuristic
      // const double h = instance.get_heuristic_euclidean(agent_num, neighbor);
      // const double h = instance.get_heuristic_manhattan(agent_num, neighbor);
      const double h2 = instance.get_heuristic_distance(agent_num, neighbor);
      // std::cout << "Neighbor: " << instance.location_to_position(neighbor) << ", h2: " << h2 << std::endl;

      // retrieve the safe intervals for the neighbor

      std::list<TimeInterval>::const_iterator sf_start, sf_end;
      std::tie(sf_start, sf_end) = safe_interval_table.get_safe_intervals(neighbor, neighbor_entry_time_interval);

      for (auto it = sf_start; it != sf_end; it++)
      {
        TimePoint neighbor_time_point(neighbor, *it);
        // modify the interval to be valid
        if (it->t_min < neighbor_entry_time_interval.t_min)
        {
          neighbor_time_point.interval.t_min = neighbor_entry_time_interval.t_min;
        }
        assertm(neighbor_time_point.interval.t_min <= neighbor_time_point.interval.t_max, "Modifying the time interval made it invalid.");

        // skip intervals that start later then the maximum time to reach the goal
        if (neighbor_time_point.interval.t_min > max_time)
        {
          continue;
        }

        // check edge collision
        if (safe_interval_table.edge_constraint_table.get(neighbor_time_point.location, current->time_point.location,
                                                          neighbor_time_point.interval.t_min))
        {
          continue;
        }

        assertm(neighbor_time_point.interval.t_min <= neighbor_entry_time_interval.t_max, "Unreachable interval among neighbors.");

        // assertm(neighbor_time_point.interval.t_min <= min_time || known_min[neighbor] == INT_MAX ||
        //             neighbor_time_point.interval.t_min >= known_min[neighbor],
        //         "The heuristic can be non consistent only for nodes starting before the goal obstruction time.");
        // check whether known
        if (neighbor_time_point.interval.t_min >= known_min[neighbor] && neighbor_time_point.interval.t_min <= known_max[neighbor])
        {
          continue;
        }


        assertm(neighbor_time_point.interval.t_min > current->time_point.interval.t_min, "Invalid interval.");

        const double h1 = std::max(h2, current->f - neighbor_time_point.interval.t_min);
        open_list.push(node_pool.add_node(SIPPNode(neighbor_time_point, neighbor_time_point.interval.t_min, h1, h1, 0, current)));

        if (settings.info_type != INFO_type::no_info)
        {
          generated_this_iter++;
        }
      }
    }

    // save the iteration info
    if (settings.info_type != INFO_type::no_info)
    {
      update_iter_info(*current, false);
    }
  }
  // reset the node pool
  node_pool.merge_extra();

  return TimePointPath();
}

void SIPP::initialize_iter_info()
{
  generated_this_iter = 0;
  expanded_this_iter  = 0;
  iteration_num       = -1;
}

void SIPP::update_iter_info(const sipp::SIPPNode& expanded, const bool last_iter)
{
  expanded_this_iter += 1;

  // store information about the expanded node for visualization only
  if (settings.info_type == INFO_type::visualisation)
  {
    iter_info.emplace_back(expanded.time_point, expanded.g, expanded.h, expanded.h2, expanded.h3, generated_this_iter, expanded_this_iter,
                           iteration_num);
  }

  if (last_iter)
  {
    // update the info across all sipp runs
    generated_all_iter += generated_this_iter;
    expanded_all_iter += expanded_this_iter;
  }
}

auto sipp::extract_path(SIPPNode const* const final_node) -> TimePointPath
{
  TimePointPath   path;
  SIPPNode const* curr = final_node;
  SIPPNode const* prev = nullptr;
  while (curr != nullptr)
  {
    // SIPPS: agent is staying in the same cell, but enters another interval right after, so merge the two
    if (!path.empty() && path.back().location == curr->time_point.location)
    {
      path.back().interval.t_min = curr->time_point.interval.t_min;
      path.back().interval.col   = path.back().interval.col || curr->time_point.interval.col;
    }
    else if (prev == nullptr)  // push the whole last interval
    {
      path.push_back(curr->time_point);
    }
    else  // push only the occupied part of other intervals
    {
      path.push_back(
          TimePoint(curr->time_point.location, TimeInterval(curr->time_point.interval.t_min, prev->time_point.interval.t_min - 1, curr->time_point.interval.col)));
    }
    prev = curr;
    curr = curr->parent;
  }
  std::reverse(path.begin(), path.end());
  return path;
}
