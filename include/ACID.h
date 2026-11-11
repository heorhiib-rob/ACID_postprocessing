#include <vector>
#include <optional>

#include "utils.h"
#include "SafeIntervalTable.h"

namespace acid
{
class ACIDNode
{
    // * a vertex of a constrained graph
    public:
        size_t id;
        int location;
        int min_leave_t; // ? if I get to the node later doesnt always mean I cannot leave as planned

        ACIDNode(
            size_t _id,
            const TimePoint& point
        )
            : 
            id(_id), 
            location(point.location), 
            min_leave_t(
                point.interval.t_max == INT_MAX ?
                -1 : 
                point.interval.t_max
            ) 
            {}
};

class ACIDConstrainedGraph
{
    // * A constrained graph of 1 agent
    // it is static so just an initial plan is stored
    public:
        ACIDConstrainedGraph(
            int _agent_id,
            const TimePointPath& path
        );
        
        const ACIDNode& get_node(size_t idx) const;
        size_t size(void) const;
    private:
        int agent_id;
        std::vector<ACIDNode> nodes;
};

class ACIDSearchNode
{
    /*
    * A search node which is dynamically changed
    */
   public:
    size_t node_id; // Position in a constrained graph

    TimeInterval interval; // ! Safe interval
    int time; // Earliest arrival time
    
    int parent = -1; // Parent search node

    ACIDSearchNode(
        size_t _node_id,
        const TimeInterval& _interval,
        int _time,
        int _parent = -1
    ) : 
    node_id(_node_id),
    interval(_interval),
    time(_time),
    parent(_parent)
    {}

};

class ACIDPlanner
{
    // TODO : incorporate SafeIntervalTable to check for collisions 
    // ! I was mkaing nonesence so I need basically implement a Dijkstra
    // ! Later implement a SIPPS
    // * all over the constraint graf
    public:
        ACIDPlanner(
            const SafeIntervalTable &hard_constraints
            //const SafeIntervalTable &soft_constraints
        );
        TimePointPath plan(const TimePointPath& original_path);
    private:
        std::optional<ACIDConstrainedGraph> constrained_graph;
        std::vector<ACIDSearchNode> search_graph;

        const SafeIntervalTable &hard_constraints;
        //const SafeIntervalTable &soft_constraints; 

        // Expand
        std::vector<ACIDSearchNode> get_successors(int current_idx);
        bool find_earliest_arrival_t(
            const int src,
            const int dst, 
            int& earliest_t, 
            const int latest_t
        );

        void buildConstrainedGraph(const TimePointPath& path);
        // Path reconstruction
        TimePointPath retrieve_path(int goal_idx);

};

class ACIDPostProcessor
{
    // TODO : make a more reasonable structure
    using Paths = std::vector<TimePointPath>;
    private:
        std::vector<ACIDConstrainedGraph> graphs;

    public:
        Paths solve(
            const Paths& paths,
            const SafeIntervalTable& hard_constraints
        );
};

} // acid