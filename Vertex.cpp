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
    HalfEdgeCIter h1 = he;
    do {
        HalfEdgeCIter h2 = v->he;
        do {
            if (h1 == h2->flip) return true;
            h2 = h2->flip->next;
            
        } while (h2 != v->he);
        
        h1 = h1->flip->next;
        
    } while (h1 != he);
    
    return false;
}