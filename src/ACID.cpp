#include "ACID.h"

#include <queue>
#include <algorithm>
using namespace acid;

size_t ACIDConstrainedGraph::size( void ) const
{
    return nodes.size();
}

const ACIDNode& ACIDConstrainedGraph::get_node( size_t idx) const
{
    return nodes.at(idx);
}

ACIDConstrainedGraph::ACIDConstrainedGraph(int _agent_id, const TimePointPath& path)
: agent_id(_agent_id)
{
    
    for(size_t i = 0; i < path.size(); i++)
    {
        ACIDNode new_node(i, path[i]);
        nodes.push_back(new_node);
    }
}

ACIDPlanner :: ACIDPlanner(            
    const SafeIntervalTable &hard_constraints
):hard_constraints(hard_constraints)
{
}

bool is_free(
    const SafeIntervalTable& table,
    int location,
    int t_min,
    int t_max
)
{
    if(t_min > t_max) return false;

    auto [first, last] = table.get_safe_intervals(
        location,
        TimeInterval(t_min, t_max)
    );

    return first != last && 
            first->t_min <= t_min &&
            last->t_max >= t_max;
}

bool ACIDPlanner::find_earliest_arrival_t(
    const int src,
    const int dst, 
    int& earliest_t, 
    const int latest_t
)
{
    while (earliest_t <= latest_t)
    {
        if (!hard_constraints.edge_constraint_table.get(
            dst, src, earliest_t
        ))
            return true;

        if (earliest_t == latest_t)
            break;

        ++earliest_t;
    }

    return false;
}

std::vector<ACIDSearchNode> ACIDPlanner :: get_successors(
    int current_idx  
)
{
    // TODO :
    //  - SIPP style expansion

    // Current nodes
    ACIDSearchNode current_state = search_graph.at(current_idx); //search state
    ACIDNode current_loc = constrained_graph->get_node(current_state.node_id);

    if(current_state.node_id + 1 >= constrained_graph->size())
        return {}; // last node => no successors

    // * Transfer times to the next location
    int earliest_arrival = std::max(
        current_state.time + 1,
        current_loc.min_leave_t + 1
    );

    int latest_arrival =
    current_state.interval.t_max == INT_MAX
        ? INT_MAX
        : current_state.interval.t_max + 1;

    if(earliest_arrival > latest_arrival) 
        return {}; //  safe interval will end earlier then we could leave it

    // Next location
    int next_idx = current_state.node_id + 1;
    ACIDNode next = constrained_graph->get_node(next_idx);

    auto [begin, end] = hard_constraints.get_safe_intervals(
        next.location, 
        TimeInterval(earliest_arrival, latest_arrival)
    );

    std::vector<ACIDSearchNode> successors;

    for(auto it = begin; it != end; ++it)
    {
        const TimeInterval& interval = *it;
        int arrival_t = std::max(
            earliest_arrival,
            interval.t_min
        ); // * start of time search in the current interval

        int latest_t = std::min(
            latest_arrival,
            interval.t_max
        ); // * end

        bool found = find_earliest_arrival_t(
            current_loc.location,
            next.location,
            arrival_t,
            latest_t
        );
        if(!found) continue;

        successors.emplace_back(
            next.id,
            interval,
            arrival_t,
            current_idx
        );
    }

    return successors;
}

TimePointPath ACIDPlanner::plan(const TimePointPath& original_path)
{
    search_graph.clear();
    constrained_graph.emplace(0, original_path);
    ACIDNode start_loc = constrained_graph->get_node(0);
    // TODO: 
    //  - create start
    //  - some kind of comparator
    //  - basically an A*

    auto start_int = *(hard_constraints.get_first_safe_interval(
        start_loc.location
    ));

    if(start_int.t_min > 0) return {};

    ACIDSearchNode start(
        start_loc.id,
        start_int,
        0
    );

    auto cmp = [this](int a_idx, int b_idx){
        return search_graph[a_idx].time >
                search_graph[b_idx].time;
    };
    std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);

    search_graph.push_back(start);
    open.push(0);

    while(!open.empty())
    {
        int current_idx = open.top();
        open.pop();

        const ACIDSearchNode& current = search_graph.at(current_idx);

        if(current.node_id + 1 == constrained_graph->size() && 
            current.interval.t_max == INT_MAX)
            return retrieve_path(current_idx);
        
        for(const auto& successor : get_successors(current_idx))
        {
            int new_id = search_graph.size();
            search_graph.push_back(successor);
            open.push(new_id);
        }
    }

    return {};
}

TimePointPath ACIDPlanner::retrieve_path(int goal_idx)
{
    int current_idx = goal_idx;
    int t_max = INT_MAX;
    TimePointPath path;
    while(current_idx != -1)
    {
        const ACIDSearchNode current = search_graph.at(current_idx);
        ACIDNode current_loc = constrained_graph->get_node(current.node_id);
        TimeInterval interval(current.time, t_max);
        t_max = current.time - 1;

        path.emplace_back(
            current_loc.location,
            interval
        );

        current_idx = current.parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
}