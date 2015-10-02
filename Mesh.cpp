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

void Mesh::collapseEdge(EdgeIter& e)
{
    HalfEdgeIter he = e->he;
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
    
    // set next halfEdge
    heNext->next = heNextNext->flip->next;
    heNextNext->flip->next->next->next = heNext;
    
    flipNextNext->next = flipNext->flip->next;
    flipNext->flip->next->next->next = flipNextNext;
    
    // set halfEdge face
    heNext->face = heNextNext->flip->face;
    flipNextNext->face = flipNext->flip->face;
    
    // set face halfEdge
    if (heNextNext->flip->face->he == heNextNext->flip) heNextNext->flip->face->he = heNext;
    if (flipNext->flip->face->he == flipNext->flip) flipNext->flip->face->he = flipNextNext;
    
    // mark for deletion
    v2->remove = true;
    e->remove = true;
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

template <typename T>
void swapMarkedRemove(std::vector<T>& vec, int& size)
{
    int n = (int)vec.size();
    
    if (n > 0) {
        int start = 0, end = n-1;
        
        while (true) {
            // find first removed and valid T
            while (!vec[start].remove && start < end) start++;
            while (vec[end].remove && start < end) end--;
            
            if (start >= end) break;
            
            // swap
            std::swap(vec[start], vec[end]);
        }
        
        size = vec[start].remove ? start : start+1;
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

void Mesh::setQuadrics()
{
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->quadric.setZero();
    }
    
    for (FaceCIter f = faces.begin(); f != faces.end(); f++) {
        Eigen::Vector4d plane = f->plane();
        
        HalfEdgeCIter he = f->he;
        do {
            VertexIter v = he->vertex;
            v->quadric += computeQuadric(plane);
            
            he = he->next;
        } while (he != f->he);
    }
}

void Mesh::setEdgeCollapseCost()
{
    for (EdgeIter e = edges.begin(); e != edges.end(); e++) {
        e->computeCollapseCost();
    }
}

void Mesh::simplify(const double ratio)
{
    // 1
    setQuadrics();
    
    // 2
    setEdgeCollapseCost();
    
    // 3
    int target = faces.size() * ratio;
    while (faces.size() > target) {
        
        EdgeIter minE = edges.begin();
        for (EdgeIter e = edges.begin(); e != edges.end(); e++) {
            if (e->cost < minE->cost) {
                minE = e;
            }
        }
        
        VertexIter v1 = minE->he->vertex;
        VertexIter v2 = minE->he->flip->vertex;
        
        // update vertex position and quadric
        v1->position = minE->position;
        v1->quadric = v1->quadric + v2->quadric;
        
        // collapse edge
        collapseEdge(minE);
        resetLists();
        
        // update edge collapse cost
        HalfEdgeIter he = v1->he;
        do {
            EdgeIter e = he->edge;
            e->computeCollapseCost();
                
            he = he->flip->next;
        } while (he != v1->he);
    }
}

void Mesh::normalize()
{
    // compute center of mass
    Eigen::Vector3d cm = Eigen::Vector3d::Zero();
    for (VertexCIter v = vertices.begin(); v != vertices.end(); v++) {
        cm += v->position;
    }
    cm /= (double)vertices.size();
    
    // translate to origin
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->position -= cm;
    }
    
    // determine radius
    double rMax = 0;
    for (VertexCIter v = vertices.begin(); v != vertices.end(); v++) {
        rMax = std::max(rMax, v->position.norm());
    }
    
    // rescale to unit sphere
    for (VertexIter v = vertices.begin(); v != vertices.end(); v++) {
        v->position /= rMax;
    }
}
