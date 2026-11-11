#include "ACID.h"

#include <queue>

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
    const ACIDConstrainedGraph& graph,
    const SafeIntervalTable &hard_constraints,
    const SafeIntervalTable &soft_constraints
)
: constrined_graph(graph), hard_constraints(hard_constraints), soft_constraints(soft_constraints)
{}


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
    ACIDNode current_loc = constrined_graph.get_node(current_state.node_id);

    if(current_state.node_id + 1 >= constrined_graph.size())
        return {}; // last node => no successors

    // Transfer times to the next location
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
    ACIDNode next = constrined_graph.get_node(next_idx);

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
        ); // start of time search in the current interval

        int latest_t = std::min(
            latest_arrival,
            interval.t_max
        ); // end

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

    /*
     Go through intervals of next 
     Check for collisions 
     add successors
    */
}

TimePointPath ACIDPlanner::plan(void)
{
    search_graph.clear();
    ACIDNode start_loc = constrined_graph.get_node(0);
    TimeInterval 
    // TODO: 
    //  - some kind of comparator
    //  - basically an A*

}

ACIDPostProcessor::Paths ACIDPostProcessor::solve(
    const Paths& paths,
    const SafeIntervalTable& hard_constraints
)
{
    // ! Dont need it
    buildConstrainedGraphs(paths);

    // TODO : build soft_constraints from paths
    // * a SafeIntervalTable of agents that will be planned by ACID

    for(const auto& path : paths)
    {
        // TODO:
        //    - Remove path from soft_constraints
        //    - Creat ACIDPlanner
        //    - Run plan()
        //    - add new paths to hard_constraints
        
    }
    return {};
}