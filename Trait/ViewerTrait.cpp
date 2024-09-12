#include "ViewerTrait.h"

using namespace MeshLib;

CViewerTrait::CViewerTrait( CMesh * pMesh )
{
    m_pMesh = pMesh;

    _allocate_vertex_trait();
    _allocate_edge_trait();
    _allocate_halfedge_trait();
    _allocate_face_trait();

};

CViewerTrait::~CViewerTrait( )
{
    _dellocate_vertex_trait();
    _dellocate_edge_trait();
    _dellocate_halfedge_trait();
    _dellocate_face_trait();
};


void CViewerTrait::_allocate_vertex_trait()
{
    m_vertex_traits = new CVertexTrait[m_pMesh->numVertices()];
    assert( m_vertex_traits );

    int id = 0;
    for( std::list<CVertex*>::iterator viter = m_pMesh->vertices().begin();viter != m_pMesh->vertices().end(); viter ++ )
    {
      CVertex * v = *viter;
       v->trait() = (CTrait*)  &m_vertex_traits[id++];
    }
}

void CViewerTrait::_allocate_edge_trait()
{
    m_edge_traits = new CEdgeTrait[m_pMesh->numEdges()];
    assert( m_edge_traits );

    int id = 0;
    for( std::list<CEdge*>::iterator eiter = m_pMesh->edges().begin(); eiter != m_pMesh->edges().end(); eiter ++ )
    {
      CEdge * e = *eiter;
       e->trait() = (CTrait*)  &m_edge_traits[id++];
    }
}
    
void CViewerTrait::_allocate_halfedge_trait()
{
    int he_num = m_pMesh->numFaces() * 3;

    m_halfedge_traits = new CHalfEdgeTrait[he_num];
    assert( m_halfedge_traits );
    
    int id = 0;
    for( std::list<CFace*>::iterator fiter = m_pMesh->faces().begin(); fiter != m_pMesh->faces().end(); fiter ++ )
    {
      CFace * f = *fiter;
      
      CHalfEdge * he = f->halfedge();
      for( int k = 0; k < 3; k ++ )
      {
        he->trait() = (CTrait*)&m_halfedge_traits[id++];
        he = he->he_next();
      }
    }
}

void CViewerTrait::_allocate_face_trait()
{
    int num = m_pMesh->numFaces() ;

    m_face_traits = new CFaceTrait[num];
    assert( m_face_traits );
    
    int id = 0;
    for( std::list<CFace*>::iterator fiter = m_pMesh->faces().begin(); fiter != m_pMesh->faces().end(); fiter ++ )
    {
      CFace * f = *fiter;
      
        f->trait() = (CTrait*)&m_face_traits[id++];
    }
}

void CViewerTrait::_dellocate_vertex_trait()
{
    for( std::list<CVertex*>::iterator viter = m_pMesh->vertices().begin();viter != m_pMesh->vertices().end(); viter ++ )
    {
      CVertex * v = *viter;
       v->trait() = NULL;
    }
    delete []m_vertex_traits;
}

void CViewerTrait::_dellocate_edge_trait()
{
    for( std::list<CEdge*>::iterator eiter = m_pMesh->edges().begin(); eiter != m_pMesh->edges().end(); eiter ++ )
    {
      CEdge * e = *eiter;
       e->trait() = NULL;
    }
    delete []m_edge_traits;
}
    
void CViewerTrait::_dellocate_halfedge_trait()
{

    for( std::list<CFace*>::iterator fiter = m_pMesh->faces().begin(); fiter != m_pMesh->faces().end(); fiter ++ )
    {
      CFace * f = *fiter;
      CHalfEdge * he = f->halfedge();
      for( int k = 0; k < 3; k ++ )
      {
        he->trait() = NULL;
        he = he->he_next();
      }
    }
    delete []m_halfedge_traits;
}

void CViewerTrait::_dellocate_face_trait()
{

    for( std::list<CFace*>::iterator fiter = m_pMesh->faces().begin(); fiter != m_pMesh->faces().end(); fiter ++ )
    {
      CFace * f = *fiter;
      f->trait() = NULL;
    }
    delete []m_face_traits;
}
