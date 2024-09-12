#ifndef _MESHLIB_HALFEDGE_H_
#define _MESHLIB_HALFEDGE_H_

#include  <assert.h>
#include <math.h>
#include <string>
#include "Edge.h"
#include "../Trait/Trait.h"

namespace MeshLib{

class CVertex;
class CEdge;
class CFace;

class CHalfEdge
{
public:

	CHalfEdge(){ m_edge = NULL; m_vertex = NULL; m_prev = NULL; m_next = NULL; m_face = NULL; m_trait = NULL;};
	~CHalfEdge(){};

	CEdge       *   &  edge()    { return m_edge;   };
	CVertex     *   &  vertex()  { return m_vertex; };
	CVertex     *   &  target()  { return m_vertex; };
	CVertex     *   &  source()  { return m_prev->vertex();};
	CHalfEdge *  &  he_prev() { return m_prev;};
	CHalfEdge *  &  he_next() { return m_next;};

	CHalfEdge * & he_sym()  { return m_edge->other( this ); };
	CFace     * & face()    { return m_face;};
	
	CHalfEdge *   ccw_rotate_about_target();
	CHalfEdge *   clw_rotate_about_target();

	CHalfEdge *   ccw_rotate_about_source();
	CHalfEdge *   clw_rotate_about_source();

  std::string & string() { return m_string; };
  CTrait *   & trait()    { return m_trait;  };
private:

	CEdge       *     m_edge;
	CFace        *     m_face;
	CVertex     *     m_vertex;		//target vertex
	CHalfEdge *	   m_prev;
	CHalfEdge *     m_next;
	std::string          m_string;
  CTrait        *     m_trait;
};


}//namespace MeshLib

#endif //_MESHLIB_HALFEDGE_H_ defined