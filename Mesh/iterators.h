#ifndef  _ITERATORS_H_
#define  _ITERATORS_H_

#include "mesh.h"

namespace MeshLib{


class CVertex;
class CHalfEdge;
class CEdge;
class CFace;
class CMesh;


//sequencial iterators

//v->out halfedge


class VertexOutHalfedgeIterator
{
public:
	VertexOutHalfedgeIterator( CMesh *  pMesh, CVertex *  v )
	{ m_pMesh = pMesh; m_vertex = v; m_halfedge = m_pMesh->vertexMostClwOutHalfEdge(v); };

	~VertexOutHalfedgeIterator(){};
	void operator++()
	{assert( m_halfedge != NULL ); 
	 if( m_halfedge == m_pMesh->vertexMostCcwOutHalfEdge(m_vertex) ) 
		 m_halfedge = NULL;
	 else
	 	 m_halfedge = m_pMesh->vertexNextCcwOutHalfEdge(m_halfedge); };

	 CHalfEdge * value() { return m_halfedge; };
	 bool end(){ return m_halfedge == NULL; };
	 CHalfEdge * operator*() { return value(); };

private:
	CMesh *        m_pMesh;
	CVertex *      m_vertex;
	CHalfEdge * m_halfedge;
};

//v->in halfedge
class VertexInHalfedgeIterator
{
public:
	VertexInHalfedgeIterator( CMesh *  pMesh, CVertex * v )
	{ m_pMesh = pMesh; m_vertex = v; m_halfedge = m_pMesh->vertexMostClwInHalfEdge(v); };

	~VertexInHalfedgeIterator(){};
	void operator++()
	{
	 assert( m_halfedge != NULL ); 
	 if( m_halfedge == m_pMesh->vertexMostCcwInHalfEdge(m_vertex) ) 
		 m_halfedge = NULL; 
	 else
		m_halfedge = m_pMesh->vertexNextCcwInHalfEdge(m_halfedge); 
	};

	 CHalfEdge * value() { return m_halfedge; };
	 bool end(){ return m_halfedge == NULL; };
	 CHalfEdge * operator*() { return value(); };

private:
	CMesh *        m_pMesh;
	CVertex *      m_vertex;
	CHalfEdge * m_halfedge;
};


//v -> v

class VertexVertexIterator
{
public:

	VertexVertexIterator( CVertex *  v )
	{ 
		m_vertex = v; 
		m_halfedge = m_vertex->most_clw_out_halfedge();
	};

	~VertexVertexIterator(){};

	void operator++()
	{
		assert( m_halfedge != NULL ); 
		
		if( !m_vertex->boundary() )
		{
			if( m_halfedge != m_vertex->most_ccw_out_halfedge() )
			{
				m_halfedge = m_halfedge->ccw_rotate_about_source();
			}
			else
			{
				m_halfedge = NULL;
			}
			return;
		}

		if( m_vertex->boundary() )
		{
			if( m_halfedge == m_vertex->most_ccw_in_halfedge() )
			{
				m_halfedge = NULL;
				return;
			}

			CHalfEdge * he = m_halfedge->ccw_rotate_about_source();

			if( he == NULL )
			{
				m_halfedge = m_vertex->most_ccw_in_halfedge();
			}
			else
			{
				m_halfedge = he;
			}
		}

		return;
	};


	 CVertex * value() 
	 { 
		 if( !m_vertex->boundary() )
		 {
			 return m_halfedge->target(); 
		 }

		 if( m_halfedge != m_vertex->most_ccw_in_halfedge() )
		 {
			 return m_halfedge->target();
		 }

		 if( m_halfedge == m_vertex->most_ccw_in_halfedge() )
		 {
			 return m_halfedge->source();
		 }
		 return NULL;
	 };

	 CVertex * operator*() { return value(); };

	 bool end(){ return m_halfedge == NULL; };

	 void reset()	{ m_halfedge = m_vertex->most_clw_out_halfedge(); };

private:
	CVertex *   m_vertex;
	CHalfEdge * m_halfedge;
};


//v -> edge

class VertexEdgeIterator
{
public:

	VertexEdgeIterator( CVertex *  v )
	{ 
		m_vertex = v; 
		m_halfedge = m_vertex->most_clw_out_halfedge();
	};

	~VertexEdgeIterator(){};

	void operator++()
	{
		assert( m_halfedge != NULL ); 
		
		if( !m_vertex->boundary() )
		{
			if( m_halfedge != m_vertex->most_ccw_out_halfedge() )
			{
				m_halfedge = m_halfedge->ccw_rotate_about_source();
			}
			else
			{
				m_halfedge = NULL;
			}
			return;
		}

		if( m_vertex->boundary() )
		{
			if( m_halfedge == m_vertex->most_ccw_in_halfedge() )
			{
				m_halfedge = NULL;
				return;
			}

			CHalfEdge * he = m_halfedge->ccw_rotate_about_source();

			if( he == NULL )
			{
				m_halfedge = m_vertex->most_ccw_in_halfedge();
			}
			else
			{
				m_halfedge = he;
			}
		}

		return;
	};


	 CEdge * value() 
	 { 
		 assert( m_halfedge != NULL );
		 return m_halfedge->edge();
	 };

	 CEdge * operator*() { return value(); };

	 bool end(){ return m_halfedge == NULL; };

	 void reset()	{ m_halfedge = m_vertex->most_clw_out_halfedge(); };

private:
	CVertex *   m_vertex;
	CHalfEdge * m_halfedge;
};



// v->face
class VertexFaceIterator
{
public:
	VertexFaceIterator( CVertex *  v )
	{ 
		m_vertex = v; 
		m_halfedge = m_vertex->most_clw_out_halfedge(); 
	};

	~VertexFaceIterator(){};

	void operator++()
	{
		assert( m_halfedge != NULL );  

		if( m_halfedge == m_vertex->most_ccw_out_halfedge() ) 
		{
			m_halfedge = NULL;
			return;
		}
		m_halfedge = m_halfedge->ccw_rotate_about_source();
	};

	 CFace * value() { return m_halfedge->face(); };
	 CFace * operator*() { return value(); };
	 bool end(){ return m_halfedge == NULL; };
	 void reset()	{ m_halfedge = m_vertex->most_clw_out_halfedge(); };

private:
	CVertex *   m_vertex;
	CHalfEdge * m_halfedge;
};

// f -> halfedge
class FaceHalfedgeIterator
{
public:

	FaceHalfedgeIterator( CFace * f )
	{ 
		m_face = f; 
		m_halfedge = f->halfedge(); 
	};

	~FaceHalfedgeIterator(){};

	void operator++()
	{
		assert( m_halfedge != NULL );
		m_halfedge = m_halfedge->he_next();

		if( m_halfedge == m_face->halfedge() )
		{
			 m_halfedge = NULL;
			return;
		};
	}

	CHalfEdge * value() { return m_halfedge; };
	CHalfEdge * operator*() { return value(); };

	bool end(){ return m_halfedge == NULL; };

private:
	CFace *        m_face;
	CHalfEdge * m_halfedge;
};


// f -> edge
class FaceEdgeIterator
{
public:

	FaceEdgeIterator( CFace * f )
	{ 
		m_face = f; 
		m_halfedge = f->halfedge(); 
	};

	~FaceEdgeIterator(){};

	void operator++()
	{
		assert( m_halfedge != NULL );
		m_halfedge = m_halfedge->he_next();

		if( m_halfedge == m_face->halfedge() )
		{
			 m_halfedge = NULL;
			return;
		};
	}

	CEdge * value() { return m_halfedge->edge(); };
	CEdge * operator*() { return value(); };

	bool end(){ return m_halfedge == NULL; };

private:
	CFace  *       m_face;
	CHalfEdge * m_halfedge;
};


// f -> vertex
class FaceVertexIterator
{
public:

	FaceVertexIterator( CFace * f )
	{ 
		m_face = f; 
		m_halfedge = f->halfedge(); 
	};

	~FaceVertexIterator(){};

	void operator++()
	{
		assert( m_halfedge != NULL );
		m_halfedge = m_halfedge->he_next();

		if( m_halfedge == m_face->halfedge() )
		{
			 m_halfedge = NULL;
			return;
		};
	}

	CVertex * value() { return m_halfedge->target(); };
	CVertex * operator*() { return value(); };

	bool end(){ return m_halfedge == NULL; };

private:
	CFace         * m_face;
	CHalfEdge * m_halfedge;
};


// soild vertices
class MeshVertexIterator
{
public:
	MeshVertexIterator( CMesh * pMesh )
	{
		m_pMesh = pMesh;
		m_iter = m_pMesh->vertices().begin();
	}

	CVertex * value() { return *m_iter; };

	void operator++()	 { ++ m_iter; };
	void operator++(int) { ++ m_iter; };

	bool end() { return m_iter == m_pMesh->vertices().end(); }

	CVertex * operator*(){ return value(); };

private:
	CMesh * m_pMesh;
	std::list<CVertex*> ::iterator m_iter;
};

// soild faces
class MeshFaceIterator
{
public:
	MeshFaceIterator( CMesh * pMesh )
	{
      m_pMesh = pMesh;
      m_iter = pMesh->faces().begin();
	}

	CFace * value() { return *m_iter; };

	void operator++() { ++ m_iter; };
	void operator++(int) { ++ m_iter; };

	bool end() { return m_iter == m_pMesh->faces().end(); }

	CFace * operator*(){ return value(); };

private:
	CMesh * m_pMesh;
  std::list<CFace*>::iterator  m_iter;
};

// soild edges
class MeshEdgeIterator
{
public:
	MeshEdgeIterator( CMesh * pMesh )
	{
    m_pMesh = pMesh;
    m_iter = m_pMesh->edges().begin();
	}

	CEdge * value() { return *m_iter; };

	void operator++() { ++ m_iter; };
	void operator++(int) {m_iter++; };

	bool end() { return m_iter == m_pMesh->edges().end(); }

	CEdge * operator*(){ return value(); };

private:
	CMesh * m_pMesh;
  std::list<CEdge*>::iterator m_iter;
};

// soild halfedges
class MeshHalfEdgeIterator
{
public:
	MeshHalfEdgeIterator( CMesh * pMesh )
	{
     m_pMesh = pMesh;
     m_iter = m_pMesh->edges().begin();
     m_id = 0;
	}

	CHalfEdge * value() { CEdge * e = *m_iter; return e->halfedge(m_id); };

	void operator++() 
	{ 
		++m_id;

		switch( m_id )
		{
		case 1:
			{
				CEdge * e = *m_iter;
				if( e->halfedge(m_id) == NULL )
				{
					m_id = 0;
					++ m_iter;
				}
			}
			break;
		case 2:
			m_id = 0;
			++m_iter;
			break;
		}
	};

	bool end() { return m_iter == m_pMesh->edges().end(); }

	CHalfEdge * operator*(){ return value(); };

private:
	CHalfEdge * m_he;
	CMesh *	 m_pMesh;
  std::list<CEdge*>::iterator m_iter;
	int  m_id;
};


} //solid 

#endif
