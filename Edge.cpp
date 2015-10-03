#include "Edge.h"
#include "HalfEdge.h"
#include "Vertex.h"

bool Edge::operator<(Edge const& e) const
{
    return cost > e.cost;
}

double error(const Eigen::Matrix4d& quadric, const double x, const double y, const double z)
{
    return quadric(0,0)*x*x + 2*quadric(0,1)*x*y + 2*quadric(0,2)*x*z + 2*quadric(0, 3)*x +
           quadric(1,1)*y*y + 2*quadric(1,2)*y*z + 2*quadric(1,3)*y +
           quadric(2,2)*z*z + 2*quadric(2,3)*z +
           quadric(3,3);
}

bool Edge::validCollapse()
{    
    HalfEdgeCIter flip = he->flip;
    
    VertexCIter v1 = he->vertex;
    VertexCIter v2 = flip->vertex;
    VertexCIter v3 = he->next->next->vertex;
    VertexCIter v4 = flip->next->next->vertex;
    
    if (v1->onBoundary() || v2->onBoundary()) return false;
    
    // check for one ring intersection
    HalfEdgeCIter h = he;
    do {
        VertexCIter v = h->flip->vertex;
        if (v != v2 && v != v3 && v != v4) {
            if (v->shareEdge(v2)) return false;
        }
        
        h = h->flip->next;
        
    } while (h != he);
    
    return true;
}

void Edge::computeCollapseCost()
{
    VertexIter v1 = he->vertex;
    if (!validCollapse()) {
        cost = INFINITY;
        position = he->vertex->position;
        return;
    }
    
    VertexIter v2 = he->flip->vertex;
    Eigen::Matrix4d quadric = v1->quadric + v2->quadric;
    
    // check if quadric is symmetric
    if (quadric(0,1) != quadric(1,0) || quadric(0,2) != quadric(2,0) || quadric(0,3) != quadric(3,0) ||
        quadric(1,2) != quadric(2,1) || quadric(1,3) != quadric(3,1) || quadric(2,3) != quadric(3,2)) {
        cost = INFINITY;
        position = v1->position;
    
    } else {
        Eigen::Matrix4d quadricDel = quadric;
        quadricDel.row(3).setZero();
        quadricDel(3,3) = 1;
        
        // check if matrix is invertible
        if (quadricDel.determinant() != 0) {            
            Eigen::Vector4d b;
            b << 0, 0, 0, 1;
            
            Eigen::Vector4d x = quadricDel.inverse() * b;
            position << x.x(), x.y(), x.z();
            cost = error(quadric, position.x(), position.y(), position.z());
            
        } else {
            
            Eigen::Vector3d p1 = v1->position;
            Eigen::Vector3d p2 = v2->position;
            Eigen::Vector3d p3 = (v1->position + v2->position) * 0.5;
            
            double e1 = error(quadric, p1.x(), p1.y(), p1.z());
            double e2 = error(quadric, p2.x(), p2.y(), p2.z());
            double e3 = error(quadric, p3.x(), p3.y(), p3.z());
            
            cost = std::min(e1, std::min(e2, e3));
            if (cost == e1) position = p1;
            else if (cost == e2) position = p2;
            else position = p3;
        }
    }
}

double Edge::length() const
{
    Eigen::Vector3d a = he->vertex->position;
    Eigen::Vector3d b = he->flip->vertex->position;
    
    return (b-a).norm();
}