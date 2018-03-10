#include "Edge.h"
#include "HalfEdge.h"
#include "Vertex.h"
#include "Face.h"

bool Edge::operator<(const Edge& e) const
{
    return cost > e.cost;
}

double error(const Eigen::Matrix4d& quadric, const double x, const double y, const double z)
{
    return quadric(0, 0)*x*x + 2*quadric(0, 1)*x*y + 2*quadric(0, 2)*x*z + 2*quadric(0, 3)*x +
           quadric(1, 1)*y*y + 2*quadric(1, 2)*y*z + 2*quadric(1, 3)*y +
           quadric(2, 2)*z*z + 2*quadric(2, 3)*z +
           quadric(3, 3);
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
    VertexIter v2 = he->flip->vertex;
    Eigen::Matrix4d quadric = v1->quadric + v2->quadric;
    
    Eigen::Matrix4d quadricDel = quadric;
    quadricDel.row(3).setZero();
    quadricDel(3, 3) = 1;
    
    // check if matrix is invertible
    if (fabs(quadricDel.determinant()) > 1e-6) {
        Eigen::Vector4d b;
        b << 0, 0, 0, 1;
        
        Eigen::Vector4d x = quadricDel.inverse() * b;
        position << x.x(), x.y(), x.z();
        cost = fmax(0.0, error(quadric, position.x(), position.y(), position.z()));
        
    } else {
        const Eigen::Vector3d& p1 = v1->position;
        const Eigen::Vector3d& p2 = v2->position;
        const Eigen::Vector3d& p3 = (v1->position + v2->position) * 0.5;
        
        double e1 = error(quadric, p1.x(), p1.y(), p1.z());
        double e2 = error(quadric, p2.x(), p2.y(), p2.z());
        double e3 = error(quadric, p3.x(), p3.y(), p3.z());
        
        if (e1 < e2 && e2 < e3) {
            cost = fmax(0.0, e1);
            position = p1;
            
        } else if (e2 < e3) {
            cost = fmax(0.0, e2);
            position = p2;
            
        } else {
            cost = fmax(0.0, e3);
            position = p3;
        }
    }
}

void Edge::collapse()
{
    HalfEdgeIter heNext = he->next;
    HalfEdgeIter heNextNext = heNext->next;
    
    HalfEdgeIter flip = he->flip;
    HalfEdgeIter flipNext = flip->next;
    HalfEdgeIter flipNextNext = flipNext->next;
    
    VertexIter v1 = he->vertex;
    VertexIter v2 = flip->vertex;
    VertexIter v3 = heNextNext->vertex;
    VertexIter v4 = flipNextNext->vertex;
    
    EdgeIter e2 = heNextNext->edge;
    EdgeIter e3 = flipNext->edge;
    
    FaceIter f = he->face;
    FaceIter fFlip = flip->face;
    
    // set halfEdge vertex
    HalfEdgeIter h = flip;
    do {
        h->vertex = v1;
        
        h = h->flip->next;
    } while (h != flip);
    
    // set vertex halfEdge
    v1->he = heNext;
    v3->he = heNextNext->flip->next;
    v4->he = flipNextNext;
    
    // set halfEdge face and face halfEdge
    heNext->face = heNextNext->flip->face;
    heNext->face->he = heNext;
    
    flipNextNext->face = flipNext->flip->face;
    flipNextNext->face->he = flipNextNext;
    
    // set next halfEdge
    heNext->next = heNextNext->flip->next;
    heNext->next->next->next = heNext;
    
    flipNextNext->next = flipNext->flip->next;
    flipNextNext->next->next->next = flipNextNext;
    
    // mark for deletion
    v2->remove = true;
    remove = true;
    e2->remove = true;
    e3->remove = true;
    he->remove = true;
    flip->remove = true;
    heNextNext->remove = true;
    heNextNext->flip->remove = true;
    flipNext->remove = true;
    flipNext->flip->remove = true;
    f->remove = true;
    fFlip->remove = true;
}

double Edge::length() const
{
    const Eigen::Vector3d& a = he->vertex->position;
    const Eigen::Vector3d& b = he->flip->vertex->position;
    
    return (b - a).norm();
}
