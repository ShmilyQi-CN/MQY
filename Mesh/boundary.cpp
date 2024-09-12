#include "Boundary.h"

using namespace MeshLib;

CLoop::CLoop( CMesh * pMesh, CHalfEdge * pH )
{
	m_pMesh     = pMesh;
	m_pHalfedge = pH;

	m_length = 0;
	CHalfEdge * he = pH;
	do{
		CVertex * v = he->target();
		he = m_pMesh->vertexMostClwOutHalfEdge( v );
		m_halfedges.push_back( he );
    m_length += m_pMesh->edgeLength( he->edge() );
	}while( he != m_pHalfedge );
}

CLoop::~CLoop()
{
	m_halfedges.clear();
}

CBoundary::CBoundary( CMesh * pMesh )
{
	m_pMesh = pMesh;

	std::set<CHalfEdge*> boundary_hes;
	for( MeshEdgeIterator eiter( m_pMesh); !eiter.end(); eiter ++ )
	{
		CEdge * e = *eiter;
		if( !m_pMesh->isBoundary(e) ) continue;
		
		CHalfEdge * he = e->halfedge(0);
		boundary_hes.insert( he );
	}

	while( !boundary_hes.empty() )
	{
		std::set<CHalfEdge*>::iterator siter = boundary_hes.begin();
		CHalfEdge * he = *siter;
		CLoop * pL = new CLoop( m_pMesh, he );
		assert(pL);
		m_loops.push_back( pL );

		for( std::list<CHalfEdge*>::iterator hiter = pL->halfedges().begin(); 
			hiter != pL->halfedges().end(); hiter ++ )
		{
			CHalfEdge * he = *hiter;
			siter = boundary_hes.find( he );
			if( siter == boundary_hes.end() ) continue;
			boundary_hes.erase( siter );
		}
	}
}

CBoundary::~CBoundary()
{
	for( std::list<CLoop*>::iterator liter = m_loops.begin(); 
		liter != m_loops.end(); liter++ )
	{
		CLoop * pL = *liter;
		delete pL;
	}

	m_loops.clear();
}
