#ifndef MESH_H
#define MESH_H

#include "Types.h"
#include "Vertex.h"
#include "Edge.h"
#include "Face.h"
#include "HalfEdge.h"

class Mesh {
public:
    // default constructor
    Mesh();
    
    // copy constructor
    Mesh(const Mesh& mesh);
        
    // read mesh from file
    bool read(const std::string& fileName);
    
    // write mesh to file
    bool write(const std::string& fileName) const;
    
    // simplifies mesh
    void simplify(const double ratio);
    
    // member variables
    std::vector<HalfEdge> halfEdges;
    std::vector<Vertex> vertices;
    std::vector<Eigen::Vector3d> uvs;
    std::vector<Eigen::Vector3d> normals;
    std::vector<Edge> edges;
    std::vector<Face> faces;
    std::vector<HalfEdgeIter> boundaries;

private:
    // center mesh about origin and rescale to unit radius
    void normalize();
    
    // initializes quadrics
    void setQuadrics();
    
    // sets edge collapse cost
    void setEdgeCollapseCost();
    
    // checks valid flip
    bool validCollapse(EdgeIter& e);
    
    // collapses edge and removes adjacent faces
    void collapseEdge(EdgeIter& e);
    
    // removes edges marked for deletion
    void resetLists();
};

#endif