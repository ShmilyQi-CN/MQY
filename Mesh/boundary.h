#ifndef _BOUNDARY_H_
#define _BOUNDARY_H_

#include <vector>
#include <list>
#include <set>

#include "../Mesh/mesh.h"
#include "../Mesh/iterators.h"

namespace MeshLib
{
class CLoop
{
public:
	 CLoop( CMesh * pMesh, CHalfEdge * pH );
	~CLoop();
	std::list<CHalfEdge*>  & halfedges() 
	{
		return m_halfedges;
	}
protected:

	CMesh     * m_pMesh;
  double        m_length;
	CHalfEdge * m_pHalfedge;
	std::list<CHalfEdge*> m_halfedges;

};

class CBoundary
{
public:
	CBoundary( CMesh * pMesh );
	~CBoundary();
	std::list<CLoop*> loops() {
		return m_loops;
	};
protected:
	CMesh * m_pMesh;
	std::list<CLoop*> m_loops;
};

}
#endif