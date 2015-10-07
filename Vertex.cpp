#include "Vertex.h"
#include "HalfEdge.h"

std::vector<HalfEdge> isolated;

bool Vertex::isIsolated() const
{
    return he == isolated.begin();
}

bool Vertex::onBoundary() const
{
    HalfEdgeCIter h = he;
    do {
        if (h->onBoundary) {
            return true;
        }
        h = h->flip->next;
        
    } while (h != he);
    
    return false;
}

bool Vertex::shareEdge(VertexCIter& v) const
{
    HalfEdgeCIter h = he;
    do {
        if (h->flip->vertex == v) {
            return true;
        }
        
        h = h->flip->next;
        
    } while (h != he);
    
    return false;
}