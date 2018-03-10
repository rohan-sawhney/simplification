#include "Face.h"
#include "HalfEdge.h"
#include "Vertex.h"

bool Face::isBoundary() const
{
    return he->onBoundary;
}

double Face::area() const
{
    if (isBoundary()) {
        return 0;
    }
    
    return 0.5 * normal().norm();
}

Eigen::Vector3d Face::normal() const
{
    const Eigen::Vector3d& a = he->vertex->position;
    const Eigen::Vector3d& b = he->next->vertex->position;
    const Eigen::Vector3d& c = he->next->next->vertex->position;
    
    Eigen::Vector3d v1 = b - a;
    Eigen::Vector3d v2 = c - a;
    
    return v1.cross(v2);
}

Eigen::Vector4d Face::plane() const
{
    const Eigen::Vector3d& a = he->vertex->position;
    
    Eigen::Vector3d n = normal();
    n.normalize();
        
    Eigen::Vector4d p;
    p << n.x(), n.y(), n.z(), -n.dot(a);
    
    return p;
}
