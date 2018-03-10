#include "Mesh.h"
#include "MeshIO.h"

Mesh::Mesh()
{
    
}

Mesh::Mesh(const Mesh& mesh)
{
    *this = mesh;
}

bool Mesh::read(const std::string& fileName)
{
    std::ifstream in(fileName.c_str());

    if (!in.is_open()) {
        std::cerr << "Error: Could not open file for reading" << std::endl;
        return false;
    }
    
    bool readSuccessful = false;
    if ((readSuccessful = MeshIO::read(in, *this))) {
        normalize();
    }
    
    return readSuccessful;
}

bool Mesh::write(const std::string& fileName) const
{
    std::ofstream out(fileName.c_str());
    
    if (!out.is_open()) {
        std::cerr << "Error: Could not open file for writing" << std::endl;
        return false;
    }
    
    MeshIO::write(out, *this);
    
    return false;
}

Eigen::Matrix4d computeQuadric(const Eigen::Vector4d& plane)
{
    double a = plane[0];
    double b = plane[1];
    double c = plane[2];
    double d = plane[3];
    
    Eigen::Matrix4d quadric;
    quadric << a*a, a*b, a*c, a*d,
               b*a, b*b, b*c, b*d,
               c*a, c*b, c*c, c*d,
               d*a, d*b, d*c, d*d;
    
    return quadric;
}

void Mesh::computeQuadrics()
{
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->quadric.setZero();
    }
    
    for (FaceCIter f = faces.begin(); f != faces.end(); f++) {
        if (!f->isBoundary()) {
            Eigen::Vector4d plane = f->plane();
            
            HalfEdgeCIter he = f->he;
            do {
                VertexIter v = he->vertex;
                v->quadric += computeQuadric(plane);
                
                he = he->next;
            } while (he != f->he);
        }
    }
}

void Mesh::computeEdgeCollapseCost()
{
    for (EdgeIter e = edges.begin(); e != edges.end(); e++) {
        e->computeCollapseCost();
    }
}

template <typename T>
void swapMarkedRemove(std::vector<T>& vec, int& size)
{
    int n = (int)vec.size();
    
    if (n > 0) {
        int start = 0, end = n - 1;
        
        while (true) {
            // find first removed and valid T
            while (!vec[start].remove && start < end) start++;
            while (vec[end].remove && start < end) end--;
            
            if (start >= end) break;
            
            // swap
            std::swap(vec[start], vec[end]);
        }
        
        size = vec[start].remove ? start : start + 1;
    }
}

void Mesh::resetLists()
{
    int nV, nE, nF, nHE;
    
    swapMarkedRemove(vertices, nV);
    swapMarkedRemove(edges, nE);
    swapMarkedRemove(halfEdges, nHE);
    swapMarkedRemove(faces, nF);
    
    // reassign iterators
    for (int i = 0; i < nV; i++) {
        if (!vertices[i].isIsolated()) {
            vertices[i].he = halfEdges.begin() + vertices[i].he->index;
        }
    }
    
    for (int i = 0; i < nE; i++) {
        edges[i].he = halfEdges.begin() + edges[i].he->index;
    }
    
    for (int i = 0; i < nHE; i++) {
        halfEdges[i].vertex = vertices.begin() + halfEdges[i].vertex->index;
        halfEdges[i].edge = edges.begin() + halfEdges[i].edge->index;
        halfEdges[i].flip = halfEdges.begin() + halfEdges[i].flip->index;
        halfEdges[i].next = halfEdges.begin() + halfEdges[i].next->index;
        halfEdges[i].face = faces.begin() + halfEdges[i].face->index;
    }
    
    for (int i = 0; i < nF; i++) {
        faces[i].he = halfEdges.begin() + faces[i].he->index;
    }
    
    // erase
    vertices.resize(nV);
    edges.resize(nE);
    halfEdges.resize(nHE);
    faces.resize(nF);
    
    // reindex
    MeshIO::indexElements(*this);
}

void Mesh::simplify(int target)
{
    // 1
    computeQuadrics();
    
    // 2
    computeEdgeCollapseCost();

    // 3
    for (EdgeIter e = edges.begin(); e != edges.end(); e++) {
        boost::heap::fibonacci_heap<Edge>::handle_type h = heap.push(*e);
        handles.push_back(h);
        (*h).handle = h;
    }
    
    // 4
    int nF = (int)faces.size();
    while (nF > target) {
        const Edge& minE = heap.top();
        EdgeIter e = edges.begin() + minE.index;
        
        if (!e->remove && e->validCollapse()) {
            VertexIter v1 = e->he->vertex;
            VertexIter v2 = e->he->flip->vertex;
            
            // update vertex position and quadric
            v1->position = e->position;
            v1->quadric = v1->quadric + v2->quadric;
            
            // collapse edge
            e->collapse();
            
            // update edge collapse cost
            HalfEdgeIter he = v1->he;
            do {
                EdgeIter e = he->edge;
                e->computeCollapseCost();
                (*handles[e->index]).cost = e->cost;
                heap.update(handles[e->index]);
                
                he = he->flip->next;
            } while (he != v1->he);
            
            nF -= 2;
            std::cout << "nF: " << nF << std::endl;
        
        } else if (e->remove) {
            heap.pop();
        
        } else {
            (*handles[e->index]).cost = INFINITY;
            heap.update(handles[e->index]);
            
            const Edge& topE = heap.top();
            EdgeIter e2 = edges.begin() + topE.index;
            
            if (e == e2) break;
        }
    }
    
    // clean up
    resetLists();
    heap.clear();
    handles.clear();
}

void Mesh::normalize()
{
    // compute center of mass
    Eigen::Vector3d cm = Eigen::Vector3d::Zero();
    for (VertexCIter v = vertices.begin(); v != vertices.end(); v++) {
        cm += v->position;
    }
    cm /= (double)vertices.size();
    
    // translate to origin and determine radius
    double rMax = 0;
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->position -= cm;
        rMax = std::max(rMax, v->position.norm());
    }
    
    // rescale to unit sphere
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->position /= rMax;
    }
}
