#ifndef  _MESHLIB_VERTEX_H_
#define _MESHLIB_VERTEX_H_

#include <stdlib.h>
#include <string>
#include "../Geometry/Point.h"
#include "../Geometry/Point2.h"
#include "../Trait/Trait.h"

namespace MeshLib{

  class CHalfEdge;

  class CVertex
  {
  public:
      CVertex(){ m_halfedge = NULL; m_boundary = false; m_trait = NULL;};
    ~CVertex(){};

    CPoint & point()    { return m_point;};
    CPoint & normal()   { return m_normal; };
	  CPoint2 & uv()       { return m_uv; };

    CHalfEdge * most_ccw_out_halfedge();
    CHalfEdge * most_clw_out_halfedge();
    CHalfEdge * most_ccw_in_halfedge();
    CHalfEdge * most_clw_in_halfedge();

    CHalfEdge * & halfedge() { return m_halfedge; };
	  std::string & string() { return m_string;};

    int  & id() { return m_id; };
    bool & boundary() { return m_boundary;};
    
    CTrait * & trait() { return m_trait; };

  private:
    int  m_id ;
    
    CPoint m_point;
    CPoint m_normal;
	  CPoint2 m_uv;

    CHalfEdge * m_halfedge;
    bool              m_boundary;
	  std::string     m_string;
    CTrait       *  m_trait;

  }; //class CVertex

}//name space MeshLib

#endif //_MESHLIB_VERTEX_H_defined