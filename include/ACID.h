#include <vector>

#include "utils.h"

class ACIDNode
{
    // * a vertex of a constrained graph
    public:
        size_t id;
        int location;
        int min_duration;

        ACIDNode(
            size_t _id,
            const TimePoint& point
        )
            : 
            id(_id), 
            location(point.location), 
            min_duration(
                point.interval.t_max == INT_MAX ?
                -1 : 
                point.interval.t_max - point.interval.t_min + 1
            ) 
            {}
};

class ACIDConstrainedGraph
{
    // * a constrained graph of 1 agent
    // it is static so just a an initial plan is stored
    public:
        ACIDConstrainedGraph(
            const TimePointPath& path
        );
    private:
        std::vector<ACIDNode> nodes;
};

class ACIDSearchNode
{
    /*
    * a search node which is dynamically changed
    */
   public:
    size_t node_id; // Position in a constrained graph
    int arrival_time; // time when entered this pose
    
    int delays = 0; // search cost 
    int colisions = 0;
    int parent = -1; // Parent search node

    ACIDSearchNode(size_t _node_id, int _arrival_time, int _parent = -1)
        :
        node_id(_node_id),
        arrival_time(_arrival_time),
        parent(parent)
        {}

    bool operator>(const ACIDSearchNode& other) const;

};

class ACIDPostProcessor
{
    // TODO : make a more reasonable structure
    using Paths = std::vector<TimePointPath>;
    private:
        std::vector<ACIDConstrainedGraph> graphs;
        void buildConstrainedGraphs(const Paths& paths);
    public:
        Paths solve(const Paths& paths);
};