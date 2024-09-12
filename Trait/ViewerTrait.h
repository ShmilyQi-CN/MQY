#ifndef  _VIEWER_TRAIT_H_
#define  _VIEWER_TRAIT_H_

#include <map>
#include <vector>

#include "../Mesh/mesh.h"
#include "../Trait/Trait.h"
#include "../Geometry/Point.h"
#include "../Geometry/Point2.h"
#include "../Parser/parser.h"

namespace MeshLib
{
  class CFaceTrait: public CTrait
  {
  public:
    double m_area;
    CPoint m_rgb;
    CPoint m_normal;
    CFaceTrait() {  m_area = 0;};
    ~CFaceTrait() {};
  };

  class CVertexTrait : public  CTrait
  {
  public:
    CPoint   m_normal;
    CPoint   m_rgb;
    CPoint2 m_uv;
    std::string m_string;
    //CVertexTrait():CTrait() {};
    CVertexTrait() {};
    ~CVertexTrait() {};

	  void read()
	  {
		  CParser parser( m_string );
		
		  for( std::list<CToken*>::iterator iter = parser.tokens().begin() ; iter != parser.tokens().end(); ++ iter )
		  {
			  CToken * token = *iter;
			  if( token->m_key == "uv" )
			  {
				  float w[2];
				  sscanf( token->m_value.c_str(), "(%g %g)",&w[0], &w[1]);
			  	  m_uv = CPoint2(w[0],w[1]);
			  }
			  if( token->m_key == "rgb" )
			  {
				  float w[3];
				  sscanf( token->m_value.c_str(), "(%g %g %g)",&w[0], &w[1], &w[2]);
			  	  m_rgb = CPoint(w[0],w[1],w[2]);
			  }
              if (token->m_key == "normal")
              {
                  float w[3];
                  sscanf(token->m_value.c_str(), "(%g %g %g)", &w[0], &w[1], &w[2]);
                  m_normal = CPoint(w[0], w[1], w[2]);
                    
              }
		  }
	  }

  };

class CEdgeTrait : public  CTrait
  {
  public:
    bool m_sharp;
    CEdgeTrait() { m_sharp = false; };
    ~CEdgeTrait(){};
  };

class CHalfEdgeTrait : public  CTrait
{
public:
  double m_angle;
  CHalfEdgeTrait() { m_angle = 0.0; };
  ~CHalfEdgeTrait() {};
};


inline double & c_a( CHalfEdge * c )
{
    CHalfEdgeTrait * pHT = (CHalfEdgeTrait*) c->trait();
    return pHT->m_angle;
};

inline CPoint2 & v_uv( CVertex* v ) 
{
    CVertexTrait * pVT = (CVertexTrait*) v->trait();
    return pVT->m_uv;
};

inline CPoint & v_rgb( CVertex* v ) 
{
    CVertexTrait * pVT = (CVertexTrait*) v->trait();
    return pVT->m_rgb;
};

inline std::string & v_string( CVertex* v ) 
{
    CVertexTrait * pVT = (CVertexTrait*) v->trait();
    return pVT->m_string;
};

inline CPoint & v_normal( CVertex* v ) 
{
    CVertexTrait * pVT = (CVertexTrait*) v->trait();
    return pVT->m_normal;
};

inline CPoint & f_normal( CFace* f ) 
{
    CFaceTrait * pFT = (CFaceTrait*) f->trait();
    return pFT->m_normal;
};

inline CPoint & f_rgb( CFace* f ) 
{
    CFaceTrait * pFT = (CFaceTrait*) f->trait();
    return pFT->m_rgb;
};

inline double & f_area( CFace* f ) 
{
    CFaceTrait * pFT = (CFaceTrait*) f->trait();
    return pFT->m_area;
};

class CViewerTrait
{
public:
	CViewerTrait( CMesh * pMesh );
	~CViewerTrait();

protected:
    CMesh * m_pMesh;

    CVertexTrait        * m_vertex_traits;
    CEdgeTrait          * m_edge_traits;
    CHalfEdgeTrait   * m_halfedge_traits;
    CFaceTrait           *m_face_traits;

  private:
    //allocate and dellocate traits

    void _allocate_vertex_trait();
    void _allocate_edge_trait();
    void _allocate_halfedge_trait();
    void _allocate_face_trait();
 
    void _dellocate_vertex_trait();
    void _dellocate_edge_trait();
    void _dellocate_halfedge_trait();
    void _dellocate_face_trait();

};

}
#endif  _VIEWER_TRAIT_H_