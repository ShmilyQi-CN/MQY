#ifndef _MESHLIB_EDGE_H_
#define _MESHLIB_EDGE_H_

#include <assert.h>
#include <stdlib.h>
#include <math.h>
#include <string>
#include "../Trait/Trait.h"

namespace MeshLib{

class CHalfEdge;
class CVertex;

class CEdge
{
public:
	CEdge(){ m_halfedge[0] = NULL; m_halfedge[1] = NULL; m_trait = NULL;};
	~CEdge(){};
	
	CHalfEdge * & halfedge( int i ) { assert( 0<=i && i < 2 ); return m_halfedge[i];};
	bool		 boundary() { return (m_halfedge[0] == NULL && m_halfedge[1] != NULL ) || (m_halfedge[0] != NULL && m_halfedge[1] == NULL ); };
	CHalfEdge * & other( CHalfEdge * he ) { return (he != m_halfedge[0] )?m_halfedge[0]:m_halfedge[1]; };
  std::string & string() { return m_string; };
  CTrait * & trait()  { return m_trait;   };

private:

	CHalfEdge  * m_halfedge[2];
  std::string      m_string;
  CTrait         * m_trait; 
};

class CEdgeKey
{
public:
	CEdgeKey( CVertex * v1, CVertex * v2 );
	~CEdgeKey(){};
	bool operator< ( const CEdgeKey & key ) const;
	bool operator==( const CEdgeKey & key ) const;


	CVertex * m_verts[2];
};

}//name space MeshLib

#endif //_MESHLIB_EDGE_H_ defined