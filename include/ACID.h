#include <vector>

#include "utils.h"
#include "SafeIntervalTable.h"

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
                point.interval.t_max + 1
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
    int time; // current search time
    
    int delays = 0; // search cost 
    int fixed_collisions = 0; //with already planned agents 
    // ? are fixed acceptable?
    int original_collisions = 0; // original collisions that could be resolved
    int parent = -1; // Parent search node
    bool node_colided = false; // collision check flag

    ACIDSearchNode(size_t _node_id, int _time, int _parent = -1)
        :
        node_id(_node_id),
        time(_time),
        parent(_parent)
        {}

    bool operator>(const ACIDSearchNode& other) const;
};

class ACIDPlanner
{
    // TODO : incorporate SafeIntervalTable to check for collisions 
    // ? do  I need it if anyways i need to check only path that are currently planning
    public:
        ACIDPlanner(
            const ACIDConstrainedGraph& graph,
            const SafeIntervalTable &hard_constraints,
            const SafeIntervalTable &soft_constraints
        );
        TimePointPath plan( void );
    private:
        const ACIDConstrainedGraph& constrined_graph;
        std::vector<ACIDSearchNode> search_graph;

        const SafeIntervalTable &hard_constraints;
        const SafeIntervalTable &soft_constraints; 

        // expand
        std::vector<ACIDSearchNode> get_successors(int current_idx);
        TimePointPath retrieve_path(void);

};

class ACIDPostProcessor
{
    // TODO : make a more reasonable structure
    using Paths = std::vector<TimePointPath>;
    private:
        std::vector<ACIDConstrainedGraph> graphs;

        void buildConstrainedGraphs(const Paths& paths);

    public:
        Paths solve(
            const Paths& paths,
            const SafeIntervalTable& hard_constraints
        );
};