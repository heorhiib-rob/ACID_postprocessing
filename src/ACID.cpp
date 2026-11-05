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

bool ACIDSearchNode::operator>(const ACIDSearchNode& other) const
{
    if(original_collisions != other.original_collisions) 
        return original_collisions > other.original_collisions;
    
    return delays > other.delays;
}

ACIDPlanner :: ACIDPlanner(const ACIDConstrainedGraph& graph)
: constrined_graph(graph)
{}

std::vector<ACIDSearchNode> ACIDPlanner :: get_successors(
    int current_idx  
)
{
    const ACIDSearchNode& current = 
        search_graph.at(current_idx); // get current search state

    const ACIDNode& node =
        constrined_graph.get_node(current.node_id);
    
    std::vector<ACIDSearchNode> successors;

    // terminal node
    if(node.min_leave_t == -1)
        return successors;

    // *when to leave with no delays
    int earlies_transfer_t = std::max(
        current.time+1,
        node.min_leave_t
    );  
    // * WAIT action
    ACIDSearchNode wait(
        current.node_id,
        earlies_transfer_t, // basically skips mandatory delay if any wasnt added yet
        current_idx
    );
    wait.delays = current.delays + 1;
    // TODO: collisions check
    wait.original_collisions = current.original_collisions;
    wait.fixed_collisions = current.fixed_collisions;

    successors.push_back(wait);

    // * ADVANCE
    ACIDSearchNode advance(
        current.node_id + 1,
        earlies_transfer_t,
        current_idx
    );

    advance.delays = current.delays;
    // TODO: collisions check
    advance.original_collisions = current.original_collisions;
    advance.fixed_collisions = current.fixed_collisions;

    successors.push_back(advance);

    return successors;
}

TimePointPath ACIDPlanner::plan( void )
{
    search_graph.clear();
    // TODO: think of initial collisions
    // ? considering LNS2 gotta some how find fixed collisions with 
    // ? not alternated with ACID paths

    auto cmp = [this](int a, int b)
    {
        return search_graph[a] > search_graph[b];
    };

    std::priority_queue<int, std::vector<int>, decltype(cmp)> open(cmp);

    search_graph.emplace_back(0,0);
    open.push(0);

    while(!open.empty())
    {
        int current_idx = open.top();
        open.pop();
        
        const ACIDSearchNode& current =
            search_graph[current_idx];

        if(constrined_graph.get_node(current.node_id).min_leave_t == -1)
        {
            // GOAL
            return;
        }

        // Expand
        std::vector<ACIDSearchNode> successors = get_successors(current_idx);
        for(auto& successor : successors)
        {
            int successor_idx = search_graph.size();
            search_graph.push_back(std::move(successor));
            open.push(successor_idx);
        }
    }

    return {}; // !NO SOLUTION
}