#ifndef EDGE_H
#define EDGE_H

#include "Types.h"

class Edge {
public:
    // one of the two half edges associated with this edge
    HalfEdgeIter he;
    
    // id between 0 and |E|-1
    int index;
    
    // flag for removal
    bool remove;
    
    // collapse cost
    double cost;
    
    // vertex position after collapse
    Eigen::Vector3d position;
    
    // heap handle
    boost::heap::fibonacci_heap<Edge>::handle_type handle;
    
    // comparator
    bool operator<(const Edge& e) const;
    
    // computes edge collapse cost
    void computeCollapseCost();
    
    // checks if collapse is valid
    bool validCollapse() const;
    
    // collapses edge
    void collapse();
    
    // computes edge length
    double length() const;
};

#endif
