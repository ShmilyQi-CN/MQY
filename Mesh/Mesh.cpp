#include "Vertex.h"
#include "HalfEdge.h"
#include "Edge.h"
#include "Face.h"
#include "Mesh.h"
#include <fstream>
#include <map>

using namespace MeshLib;

//access e->v
CMesh::tVertex CMesh::edgeVertex1( CMesh::tEdge  e )
{
	assert( e->halfedge(0 ) != NULL );
	return e->halfedge(0)->source();
};

//access e->v
CMesh::tVertex CMesh::edgeVertex2( CMesh::tEdge  e )
{
	assert( e->halfedge(0 ) != NULL );
	return e->halfedge(0)->target();
};

//access e->f
CMesh::tFace CMesh::edgeFace1( CMesh::tEdge  e )
{
	assert( e->halfedge(0) != NULL );
	return e->halfedge(0)->face();
};

//access e->f
CMesh::tFace CMesh::edgeFace2( CMesh::tEdge  e )
{
	assert( e->halfedge(1) != NULL );
	return e->halfedge(1)->face();
};

//access he->f
CMesh::tFace CMesh::halfedgeFace( CMesh::tHalfEdge  he )
{
	return he->face();
};


//access he->v
CMesh::tVertex CMesh::halfedgeVertex( CMesh::tHalfEdge  he )
{
	return he->vertex();
};

bool CMesh::isBoundary( CMesh::tVertex  v )
{
	return v->boundary();
};

bool CMesh::isBoundary( CMesh::tEdge  e )
{
	if( e->halfedge(0) == NULL || e->halfedge(1) == NULL ) return true;
	return false;
};

bool CMesh::isBoundary( CMesh::tHalfEdge  he )
{
	if( he->he_sym() == NULL ) return true;
	return false;
};

int CMesh::numVertices() 
{
	return (int) m_verts.size();
};

int CMesh::numEdges() 
{
	return (int) m_edges.size();
};

int CMesh::numFaces() 
{
	return (int) m_faces.size();
};


//Euler operation

CMesh::tHalfEdge CMesh::vertexMostClwOutHalfEdge( CMesh::tVertex  v )
{
	return v->most_clw_out_halfedge();
};

CMesh::tHalfEdge CMesh::vertexMostCcwOutHalfEdge( CMesh::tVertex  v )
{
	return v->most_ccw_out_halfedge();
};

CMesh::tHalfEdge CMesh::corner( tVertex v, tFace f)
{
	CMesh::tHalfEdge he = f->halfedge();
	do{
		if( he->vertex() == v )
			return he;
		he = he->he_next();
	}while( he != f->halfedge() );
	return NULL;
};

CMesh::tHalfEdge CMesh::vertexNextCcwOutHalfEdge( CMesh::tHalfEdge  he )
{
	return he->ccw_rotate_about_source();
};

CMesh::tHalfEdge CMesh::vertexNextClwOutHalfEdge( CMesh::tHalfEdge  he )
{
	assert( he->he_sym() != NULL );
	return he->clw_rotate_about_source();
};

CMesh::tHalfEdge CMesh::vertexMostClwInHalfEdge( CMesh::tVertex  v )
{
	return v->most_clw_in_halfedge();
};

CMesh::tHalfEdge CMesh::vertexMostCcwInHalfEdge( CMesh::tVertex  v )
{
	return v->most_ccw_in_halfedge();
};

CMesh::tHalfEdge CMesh::vertexNextCcwInHalfEdge( CMesh::tHalfEdge  he )
{
	assert( he->he_sym() != NULL );
	return he->ccw_rotate_about_target();
};

CMesh::tHalfEdge CMesh::vertexNextClwInHalfEdge( CMesh::tHalfEdge  he )
{
	return he->clw_rotate_about_target();
};

CMesh::tHalfEdge CMesh::faceNextClwHalfEdge( CMesh::tHalfEdge  he )
{
	return he->he_prev();
};

CMesh::tHalfEdge CMesh::faceNextCcwHalfEdge( CMesh::tHalfEdge  he )
{
	return he->he_next();
};

CMesh::tHalfEdge CMesh::faceMostCcwHalfEdge( CMesh::tFace  face )
{
	return face->halfedge();
};

CMesh::tHalfEdge CMesh::faceMostClwHalfEdge( CMesh::tFace  face )
{
	return face->halfedge()->he_next();
};


CMesh::~CMesh()
{
	//remove vertices

  for( std::list<CVertex*>::iterator viter = m_verts.begin(); viter != m_verts.end(); viter ++ )
  {
      CVertex * pV = *viter;
      delete pV;
  }
  m_verts.clear();

	//remove faces

  for( std::list<CFace*>::iterator fiter = m_faces.begin(); fiter != m_faces.end(); fiter ++ )
  {
      CFace * pF = *fiter;

      tHalfEdge he = pF->halfedge();
 
      std::list<CHalfEdge*> hes;
      do{
        he = he->he_next();
        hes.push_back( he );
      }while( he != pF->halfedge() );

      for( std::list<CHalfEdge*>::iterator hiter = hes.begin(); hiter != hes.end(); hiter ++)
      {
          CHalfEdge * pH = *hiter;
          delete pH;
      }
      hes.clear();

      delete pF;
  }
  m_faces.clear();
	
  //remove edges
  for( std::list<CEdge*>::iterator eiter = m_edges.begin(); eiter != m_edges.end(); eiter ++ )
  {
      CEdge * pE = *eiter;
      delete pE;
  }

  m_edges.clear();

  m_map_vert.clear();
  m_map_face.clear();
  m_map_edge.clear();
};


/*
void Solid::write( std::ostream & os )
{

	//remove vertices
	AVL::TreeIterator<Vertex> viter( m_verts );
	for( ; !viter.end() ; ++ viter )
	{
		tVertex v = *viter;

		os << "Vertex " << v->id();
		
		for( int i = 0; i < 3; i ++ )
		{
			os << " " << v->point()[i];  
		}

		if( v->string().size() > 0 )
		{
			os << " {" << v->string() << "}";
		}

		os << std::endl;
	}

	for( AVL::TreeIterator<Face> fiter(m_faces); !fiter.end(); ++ fiter )
	{
		tFace f = *fiter;
		os << "Face " << f->id();
		tHalfEdge he = f->halfedge();
		do{
			os << " " << he->target()->id();
			he = he->he_next();
		}while( he != f->halfedge() );

		if( f->string().size() > 0 )
		{
			os << " {" << f->string() << "}";
		}
		os << std::endl;
	}


	for( AVL::TreeIterator<Edge> eiter(m_edges); !eiter.end(); ++ eiter )
	{
		tEdge e = *eiter;
		if( e->string().size() > 0 )
		{
			os << "Edge " << edgeVertex1(e)->id()<< " " << edgeVertex2(e)->id() << " ";
			os << "{" << e->string() << "}" << std::endl;
		}
	}

	for( AVL::TreeIterator<Face> pfiter(m_faces); !pfiter.end(); ++ pfiter )
	{
		tFace f = *pfiter;
		tHalfEdge he = f->halfedge();
		do{
			if( he->string().size() > 0 )
			{
				os << "Corner " << he->vertex()->id() << " " << he->face()->id() << " ";
				os <<"{" << he->string() << "}" << std::endl;
			}
			he = he->he_next();
		}while( he != f->halfedge() );
	}

};

void Solid::read( std::istream & is )
{
	char line[MAX_LINE];
	int id;

	while( is && !is.eof() && is.getline(line, MAX_LINE) )
	{		
		if( strlen( line ) == 0 ) continue;

		std::string s(line);

		string_token_iterator iter(s, " \n"); 
		
		std::string str = *iter;

		if( str == "Vertex" ) 
		{
			
			str = *(++iter);

			id = atoi( str.c_str() );


			Point p;
			for( int i = 0 ; i < 3; i ++ )
			{
				str = *(++iter);
				p[i] = atof( str.c_str() );
			}
		
			tVertex v  = createVertex( id );
			v->point() = p;
			v->id()    = id;

			int sp = s.find("{");
			int ep = s.find("}");

			if( sp >= 0 && ep >= 0 )
			{
				v->string() = s.substr( sp+1, ep-sp-1 );
			}

			continue;

		}

		if( str == "Face" ) 
		{
			str = *(++iter);

			id = atoi( str.c_str() );
	
			int v[3];
			for( int i = 0; i < 3; i ++ )
			{
				str = *(++iter);
				v[i] = atoi( str.c_str() );
			}

			tFace f = createFace( v, id );

			int sp = s.find("{");
			int ep = s.find("}");

			if( sp >= 0 && ep >= 0 )
			{
				f->string() = s.substr( sp+1, ep-sp-1 );
			}
			continue;
		}

		//read in edge attributes
		if( str == "Edge" )
		{
			str = *(++iter);
			int id0 = atoi( str.c_str() );

			str = *(++iter);
			int id1 = atoi( str.c_str() );

			tEdge edge = idEdge( id0, id1 );

			str = *(++iter);

			int sp = s.find("{");
			int ep = s.find("}");

			if( sp >= 0 && ep >= 0 )
			{
				edge->string() = s.substr( sp+1, ep-sp-1 );
			}
			
			continue;

		}

		//read in edge attributes
		if( str == "Corner" ) 
		{
			str = *(++iter);
			int id0 = atoi( str.c_str() );
				
			str = *(++iter);
			int id1 = atoi( str.c_str() );


			Vertex * v = idVertex( id0 );
			Face   * f = idFace( id1 );
			tHalfEdge he = corner( v, f );

			str = *(++iter);

			int sp = s.find("{");
			int ep = s.find("}");

			if( sp >= 0 && ep >= 0 )
			{
				he->string() = s.substr( sp+1, ep-sp-1 );
			}
			continue;
		}


	}

	//Label boundary edges
	for( AVL::TreeIterator<Edge> eiter( m_edges ); ! eiter.end() ; ++ eiter )
	{
		Solid::tEdge     edge = *eiter;
		Solid::tHalfEdge he[2];

		he[0] = edge->halfedge(0);
		he[1] = edge->halfedge(1);
		
		assert( he[0] != NULL );
		

		if( he[1] != NULL )
		{
			assert( he[0]->target() == he[1]->source() && he[0]->source() == he[1]->target() );

			if( he[0]->target()->id() < he[0]->source()->id() )
			{
				edge->halfedge(0 ) = he[1];
				edge->halfedge(1 ) = he[0];
			}

			assert( edgeVertex1(edge)->id() < edgeVertex2(edge)->id() );
		}
		else
		{
			he[0]->vertex()->boundary() = true;
			he[0]->he_prev()->vertex()->boundary()  = true;
		}

	}

	List<Vertex> dangling_verts;
	//Label boundary edges
	for( AVL::TreeIterator<Vertex> viter( m_verts ); ! viter.end() ; ++ viter )
	{
		Solid::tVertex     v = *viter;
		if( v->halfedge() != NULL ) continue;
		dangling_verts.Append( v );
	}

	for( ListIterator<Vertex> iter( dangling_verts ); !iter.end(); ++ iter )
	{
		Solid::tVertex v = *iter;
		m_verts.remove( v );
		delete v;
		v = NULL;
	}
};








void Solid::copy( Solid & solid )
{	
	for(AVL::TreeIterator<Vertex> viter(m_verts); !viter.end(); ++viter)
	{
		Vertex * v = *viter;
		Vertex * w = solid.createVertex( v->id() );

		w->point() = v->point();
		w->string()= v->string();
		w->boundary() = v->boundary();
	}

	for(AVL::TreeIterator<Face> fiter(m_faces); !fiter.end(); ++fiter)
	{
		Face * f = *fiter;

		HalfEdge * he = f->halfedge();
		int v[3];

		for( int i = 0; i < 3; i ++ )
		{
			v[i] = he->vertex()->id();
			he = he->he_next();
		}

		Face * F = solid.createFace(v, f->id() );
		F->string() = f->string();
	}

	//Label boundary edges
	for( AVL::TreeIterator<Edge> eiter( solid.m_edges ); ! eiter.end() ; ++ eiter )
	{
		Solid::tEdge     edge = *eiter;
		Solid::tHalfEdge he[2];

		he[0] = edge->halfedge(0);
		he[1] = edge->halfedge(1);
		
		assert( he[0] != NULL );
		

		if( he[1] != NULL )
		{
			assert( he[0]->target() == he[1]->source() && he[0]->source() == he[1]->target() );

			if( he[0]->target()->id() < he[0]->source()->id() )
			{
				edge->halfedge(0 ) = he[1];
				edge->halfedge(1 ) = he[0];
			}

			assert( edgeVertex1(edge)->id() < edgeVertex2(edge)->id() );
		}
		else
		{
			he[0]->vertex()->boundary() = true;
			he[0]->he_prev()->vertex()->boundary()  = true;
		}

	}


};

Vertex * Solid::edgeSplit( Edge * e )
{
	//find the max_id for vertices;

	AVL::TreeIterator<Vertex> viter(m_verts);

	int max_vid = -1;

	for( ; !viter.end(); ++viter )
	{
		max_vid = ( max_vid > (*viter)->id() )? max_vid: (*viter)->id();
	}

	AVL::TreeIterator<Face> fiter(m_faces);

	int max_fid = -1;

	for( ; !fiter.end(); ++fiter )
	{
		max_fid = ( max_fid > (*fiter)->id() )? max_fid: (*fiter)->id();
	}

	//create a new vertex

	tVertex nv = createVertex( ++max_vid );

	tHalfEdge hes[2];
	hes[0] = e->halfedge(0);
	hes[1] = e->halfedge(1);

	nv->boundary() = (hes[1]==NULL);

	m_edges.remove( e );


	List<Face> new_faces;

	for( int j = 0; j < 2; j ++ )
	{
		tVertex v[3];
		tHalfEdge he = hes[j];

		if( he == NULL ) continue;

		tFace  f = he->face();
		m_faces.remove( f );
		delete f;

		for( int i = 0; i < 3; i ++ )
		{
			v[i] = he->target();
			
			tEdge te = he->edge();
			if( te->halfedge(0) == he )
			{
				te->halfedge(0) = te->halfedge(1);
			}
			te->halfedge(1) = NULL;

			he = he->he_next();
		}

		for( int k = 0; k < 3; k ++ )
		{
			tHalfEdge ne = he->he_next();
			delete he;
			he = ne;
		}

		int vid[3];
		tVertex w[3];
		w[0] = nv; w[1] = v[0]; w[2] = v[1];
		for( k = 0; k < 3; k ++ )
		{
			vid[k] = w[k]->id(); 
		}		

		Face * nf = createFace( vid, ++max_fid );
		new_faces.Append( nf );

		w[0] = nv; w[1] = v[1]; w[2] = v[2];
		for( k = 0; k < 3; k ++ )
		{
			vid[k] = w[k]->id(); 
		}	
		nf = createFace( vid, ++ max_fid );
		new_faces.Append( nf );
	}	


	for( ListIterator<Face> iter( new_faces ); !iter.end(); ++ iter )
	{
		Face * f = *iter;
		HalfEdge * he = f->halfedge();

		do{
			Edge     * e = he->edge();
			if( e->halfedge(1) != NULL )
			{
				HalfEdge * h = e->halfedge(0);
				if( h->target()->id() < h->source()->id() )
				{
					e->halfedge(0) = e->halfedge(1);
					e->halfedge(1) = h;
				}
			}
			he = he->he_next();
		}while( he != f->halfedge() );
	}

	delete e;
	return nv;	
};


*/
double CMesh::edgeLength( CMesh::tEdge e )
{
	CVertex * v1 = edgeVertex1(e);
	CVertex * v2 = edgeVertex2(e);

	return ( v1->point() - v2->point() ).norm();
}

/*
bool Solid::import( const char * filename )
{
//	TRACE("load obj file %s\n",filename);

	FILE* f = fopen(filename, "r");
	if( f == NULL )
	{
		return false;
	}

	char cmd[1024];
	char seps[] = " /,\t\n";
	int  vid = 1;
	int  fid = 1;

	while (true)
    {
		if (fgets(cmd, 1024, f) == NULL)
			break;

		char *token = strtok(cmd, seps);
		
		if (token == NULL)
			continue;


		if (strcmp(token, "v") == 0)
		{
			Point p;
			for( int i = 0; i < 3; i ++ )
			{
				token = strtok( NULL, seps );
				p[i] = atof( token );
			}
			
			Vertex * v = createVertex( vid++ );
			v->point() = p;
			continue;
		}

		if (strcmp(token, "vt") == 0)
		{
			Point p;
			for( int i = 0; i < 2; i ++ )
			{
				token = strtok( NULL, seps );
				p[i] = atof( token );
			}
			continue;
		}

		if (strcmp(token, "vn") == 0)
		{
			Point p;
			for( int i = 0; i < 3; i ++ )
			{
				token = strtok( NULL, seps );
				p[i] = atof( token );
			}
			continue;
		}

		if (strcmp(token, "f") == 0)
		{
			int v[3];
			for( int i = 0 ; i < 3; i ++ )
			{
				token  = strtok( NULL, seps );
				v[i] = atoi(token);
			}
			createFace( v, fid++ );
		}
	}
	fclose(f);
	
	//Label boundary edges
	for( AVL::TreeIterator<Edge> eiter( m_edges ); ! eiter.end() ; ++ eiter )
	{
		Solid::tEdge     edge = *eiter;
		Solid::tHalfEdge he[2];

		he[0] = edge->halfedge(0);
		he[1] = edge->halfedge(1);
		
		assert( he[0] != NULL );
		

		if( he[1] != NULL )
		{
			assert( he[0]->target() == he[1]->source() && he[0]->source() == he[1]->target() );

			if( he[0]->target()->id() < he[0]->source()->id() )
			{
				edge->halfedge(0 ) = he[1];
				edge->halfedge(1 ) = he[0];
			}

			assert( edgeVertex1(edge)->id() < edgeVertex2(edge)->id() );
		}
		else
		{
			he[0]->vertex()->boundary() = true;
			he[0]->he_prev()->vertex()->boundary()  = true;
		}

	}

	List<Vertex> dangling_verts;
	//Label boundary edges
	for( AVL::TreeIterator<Vertex> viter( m_verts ); ! viter.end() ; ++ viter )
	{
		Solid::tVertex     v = *viter;
		if( v->halfedge() != NULL ) continue;
		dangling_verts.Append( v );
	}

	for( ListIterator<Vertex> iter( dangling_verts ); !iter.end(); ++ iter )
	{
		Solid::tVertex v = *iter;
		m_verts.remove( v );
		delete v;
		v = NULL;
	}

	return true;
}
*/



//create new gemetric simplexes

CMesh::tVertex CMesh::createVertex( int id )
{
	tVertex v = new CVertex();
	assert( v != NULL );
	v->id() = id;
	m_verts.push_back( v );
	m_map_vert.insert( std::pair<int,CVertex*>(id,v));
	return v;//insert a new vertex, with id as the key
};



void CMesh::read_obj( const char * filename )
{
//	TRACE("load obj file %s\n",filename);

	FILE* f = fopen(filename, "r");
	if( f == NULL ) return;

	char cmd[1024];
	char seps[] = " /,\t\n";
	int  vid = 1;
	int  fid = 1;


	while (true)
    {
		if (fgets(cmd, 1024, f) == NULL)
			break;

		char *token = strtok(cmd, seps);
		
		if (token == NULL)
			continue;


		if (strcmp(token, "v") == 0)
		{
			CPoint p;
			for( int i = 0; i < 3; i ++ )
			{
				token = strtok( NULL, seps );
				p[i] = atof( token );
			}
			
			CVertex * v = createVertex( vid);
			v->point() = p;
			vid ++;
			continue;
		}

		if (strcmp(token, "vt") == 0)
		{
			CPoint p;
			for( int i = 0; i < 2; i ++ )
			{
				token = strtok( NULL, seps );
				p[i] = atof( token );
			}
			continue;
		}

		if (strcmp(token, "vn") == 0)
		{
			CPoint p;
			for( int i = 0; i < 3; i ++ )
			{
				token = strtok( NULL, seps );
				p[i] = atof( token );
			}
			continue;
		}




		if (strcmp(token, "f") == 0)
		{
			CVertex* v[3];
			for( int i = 0 ; i < 3; i ++ )
			{
				token  = strtok( NULL, seps );
				int id  = atoi(token);
				v[i] = m_map_vert[id];
			}
			createFace( v, fid++ );
		}
	}
	fclose(f);
	
	//Label boundary edges
	for(std::list<CEdge*>::iterator eiter= m_edges.begin() ; eiter != m_edges.end() ; ++ eiter )
	{
		CMesh::tEdge     edge = *eiter;
		CMesh::tHalfEdge he[2];

		he[0] = edge->halfedge(0);
		he[1] = edge->halfedge(1);
		
		assert( he[0] != NULL );
		

		if( he[1] != NULL )
		{
			assert( he[0]->target() == he[1]->source() && he[0]->source() == he[1]->target() );

			if( he[0]->target()->id() < he[0]->source()->id() )
			{
				edge->halfedge(0 ) = he[1];
				edge->halfedge(1 ) = he[0];
			}

			assert( edgeVertex1(edge)->id() < edgeVertex2(edge)->id() );
		}
		else
		{
			he[0]->vertex()->boundary() = true;
			he[0]->he_prev()->vertex()->boundary()  = true;
		}

	}

	std::list<CVertex*> dangling_verts;
	//Label boundary edges
	for(std::list<CVertex*>::iterator viter = m_verts.begin();  viter != m_verts.end() ; ++ viter )
	{
		CMesh::tVertex     v = *viter;
		if( v->halfedge() != NULL ) continue;
		dangling_verts.push_back( v );
	}

	for( std::list<CVertex*>::iterator  viter = dangling_verts.begin() ; viter != dangling_verts.end(); ++ viter )
	{
		CMesh::tVertex v = *viter;
		m_verts.remove( v );
		delete v;
		v = NULL;
	}

	//Arrange the boundary half_edge of boundary vertices, to make its halfedge
	//to be the most ccw in half_edge

	for(std::list<CVertex*>::iterator viter = m_verts.begin();  viter != m_verts.end() ; ++ viter )
	{
		CMesh::tVertex     v = *viter;
		if( !v->boundary() ) continue;

		CHalfEdge * he = v->halfedge();
		while( he->he_sym() != NULL )
		{
			he = he->ccw_rotate_about_target();
		}

		v->halfedge() = he;
	}

}

CMesh::tFace CMesh::createFace( CVertex * v[] , int id )
{
	  tFace f = new CFace();
	  assert( f != NULL );
	  f->id() = id;
	  m_faces.push_back( f );
	  m_map_face.insert( std::pair<int,tFace>(id,f) );

		//create halfedges
		tHalfEdge hes[3];

		for(int i = 0; i < 3; i ++ )
		{
			hes[i] = new CHalfEdge;
			assert( hes[i] );
			CVertex * vert =  v[i];
			hes[i]->vertex() = vert;
			vert->halfedge() = hes[i];
		}

		//linking to each other
		for(int i = 0; i < 3; i ++ )
		{
			hes[i]->he_next() = hes[(i+1)%3];
			hes[i]->he_prev() = hes[(i+2)%3];
		}

		//linking to face
		for(int i = 0; i < 3; i ++ )
		{
			hes[i]->face()   = f;
			f->halfedge()    = hes[i];
		}

		//connecting with edge
		for(int i = 0; i < 3; i ++ )
		{
			tEdge e = createEdge( v[i], v[(i+2)%3] );
			if( e->halfedge(0)  == NULL || e->halfedge(0)->source() == hes[i]->source())
			{
				e->halfedge(0) = hes[i];
			}
			else if (e->halfedge(1) == NULL || e->halfedge(1)->source() == hes[i]->source())
			{
				e->halfedge(1) = hes[i];
			}
			hes[i]->edge() = e;
		}

		return f;
};


//access id->v
CMesh::tVertex CMesh::idVertex( int id ) 
{
	return m_map_vert[id];
};

//access v->id
int CMesh::vertexId( CMesh::tVertex  v )
{
	return v->id();
};

//access id->f
CMesh::tFace CMesh::idFace( int id )
{
	return m_map_face[id];
};

//acess f->id
int CMesh::faceId( CMesh::tFace  f )
{
	return f->id();
};

CMesh::tEdge CMesh::createEdge( CMesh::tVertex v1, CMesh::tVertex v2 )
{
	CEdgeKey key(v1,v2);

	tEdge e = NULL;

	if( m_map_edge.find( key ) != m_map_edge.end() )
	{
		e = m_map_edge[key];
		return e;
	}

	e = new CEdge;

	assert( e != NULL );
	m_map_edge.insert( std::pair<CEdgeKey,CMesh::tEdge>(key,e) );
	m_edges.push_back( e );

	return e;

};



//access vertex->edge
CMesh::tEdge CMesh::vertexEdge( CMesh::tVertex v0, CMesh::tVertex v1 )
{
	CEdgeKey key(v0,v1);
	return m_map_edge[key];
};



//access vertex->edge
CMesh::tHalfEdge CMesh::vertexHalfedge( tVertex v0, tVertex v1 )
{
	tEdge e = vertexEdge( v0, v1 );
	assert( e != NULL );
	tHalfEdge he = e->halfedge(0);
	if( he->vertex() == v1 && he->he_prev()->vertex() == v0 ) return he;
	he = e->halfedge(1);
	assert( he->vertex() == v1 && he->he_prev()->vertex() == v0 );
	return he;
};



void CMesh::read_m( const char * input )
{
	FILE * is = fopen( input, "r" );
	if( is == NULL )
	{
		fprintf(stderr,"Error is opening file %s\n", input );
		return;
	}

	char line[MAX_LINE];
	int id;

	while( !feof(is)  )
	{		
		if( fgets( line, MAX_LINE, is ) == NULL ) break;
		if( strlen( line )< 4 ) break;

		char * str = strtok( line, " \r\n");
		if( strcmp(str, "Vertex" ) == 0 ) 
		{
			
			str = strtok(NULL," \r\n");
			id = atoi( str );
			CPoint p;
			for( int i = 0 ; i < 3; i ++ )
			{
				str = strtok(NULL," \r\n");
				p[i] = atof( str );
			}
		
			tVertex v  = createVertex( id );
			v->point() = p;
			v->id()    = id;

			str = strtok( NULL, "\r\n");
			if( str == NULL ) continue;

			std::string s(str);
			int sp = (int) s.find("{");
			int ep = (int) s.find("}");

			if( sp >= 0 && ep >= 0 )
			{
				v->string() = s.substr( sp+1, ep-sp-1 );
			}
			continue;
		}
		

		if( strcmp(str,"Face") == 0 ) 
		{
			str = strtok(NULL, " \r\n");
			id = atoi( str );
	
			CVertex * v[3];
			for( int i = 0; i < 3; i ++ )
			{
				str = strtok(NULL," \r\n");
				v[i] = idVertex( atoi( str ) );
			}

			tFace f = createFace( v, id );

			str = strtok( NULL, "\r\n");
      if( str == NULL )       continue;
			
			std::string s(str);

			int sp = (int) s.find("{");
			int ep = (int) s.find("}");

			if( sp >= 0 && ep >= 0 )
			{
				f->string() = s.substr( sp+1, ep-sp-1 );
			}
			continue;
		}

		//read in edge attributes
		if( strcmp(str,"Edge")==0 )
		{
			str = strtok(NULL, " \r\n");
			int id0 = atoi( str );

			str = strtok(NULL, " \r\n");
			int id1 = atoi( str );

      CVertex * v0 = idVertex( id0 );
      CVertex * v1 = idVertex( id1 );

			tEdge edge = vertexEdge( v0, v1 );

			str = strtok(NULL, "\r\n");

			std::string s(str);
			int sp = (int) s.find("{");
			int ep = (int) s.find("}");

			if( sp >= 0 && ep >= 0 )
			{
				  edge->string() = s.substr( sp+1, ep-sp-1 );
			}
			continue;//即赋予其标记从而与其它边区分
		}

		//read in edge attributes
		if( strcmp(str,"Corner")==0 ) 
		{
			str = strtok(NULL," \r\n");
			int id0 = atoi( str );
				
			str = strtok(NULL," \r\n");
			int id1 = atoi( str );


			CVertex * v = idVertex( id0 );
			CFace   * f = idFace( id1 );
			tHalfEdge he = corner( v, f );

			str = strtok(NULL,"\r\n");
			std::string s(str);

			int sp = (int) s.find("{");
			int ep = (int) s.find("}");

			if( sp >= 0 && ep >= 0 )
			{
				he->string() = s.substr( sp+1, ep-sp-1 );
			}
			continue;
		}
	}

	//Label boundary edges
	for(std::list<CEdge*>::iterator eiter= m_edges.begin() ; eiter != m_edges.end() ; ++ eiter )
	{
		CMesh::tEdge     edge = *eiter;
		CMesh::tHalfEdge he[2];

		he[0] = edge->halfedge(0);
		he[1] = edge->halfedge(1);
		
		assert( he[0] != NULL );
		

		if( he[1] != NULL )
		{
			assert( he[0]->target() == he[1]->source() && he[0]->source() == he[1]->target() );

			if( he[0]->target()->id() < he[0]->source()->id() )
			{
				edge->halfedge(0 ) = he[1];
				edge->halfedge(1 ) = he[0];
			}

			assert( edgeVertex1(edge)->id() < edgeVertex2(edge)->id() );
		}
		else
		{
			he[0]->vertex()->boundary() = true;
			he[0]->he_prev()->vertex()->boundary()  = true;
		}

	}

	std::list<CVertex*> dangling_verts;
	//Label boundary edges
	for(std::list<CVertex*>::iterator viter = m_verts.begin();  viter != m_verts.end() ; ++ viter )
	{
		CMesh::tVertex     v = *viter;
		if( v->halfedge() != NULL ) continue;
		dangling_verts.push_back( v );
	}

	for( std::list<CVertex*>::iterator  viter = dangling_verts.begin() ; viter != dangling_verts.end(); ++ viter )
	{
		CMesh::tVertex v = *viter;
		m_verts.remove( v );
		delete v;
		v = NULL;
	}

	//Arrange the boundary half_edge of boundary vertices, to make its halfedge
	//to be the most ccw in half_edge

	for(std::list<CVertex*>::iterator viter = m_verts.begin();  viter != m_verts.end() ; ++ viter )
	{
		CMesh::tVertex     v = *viter;
		if( !v->boundary() ) continue;

		CHalfEdge * he = v->halfedge();
		while( he->he_sym() != NULL )
		{
			he = he->ccw_rotate_about_target();
		}
		v->halfedge() = he;
	}


};


void CMesh::write_m( const char * output )
{
	FILE * _os = fopen( output,"w");
	assert( _os );

	//remove vertices
  for( std::list<CVertex*>::iterator viter = m_verts.begin(); viter != m_verts.end(); viter ++)
  {
		tVertex v = *viter;

		fprintf(_os,"Vertex %d ", v->id() );
		
		for( int i = 0; i < 3; i ++ )
		{
			fprintf(_os,"%g ",v->point()[i]);
		}
		if( v->string().size() > 0 )
		{
			fprintf(_os,"{%s}", v->string().c_str() );
		}
		fprintf(_os,"\n");
	}

  for( std::list<CFace*>::iterator fiter = m_faces.begin(); fiter != m_faces.end(); fiter ++ )
	{
		tFace f = *fiter;
		fprintf(_os,"Face %d",f->id());

		tHalfEdge he = f->halfedge();
		do{
			fprintf(_os," %d", he->target()->id() );
			he = he->he_next();
		}while( he != f->halfedge() );

		if( f->string().size() > 0 )
		{
			fprintf(_os,"{%s}", f->string().c_str() );
		}

		fprintf(_os,"\n");
	}

  for( std::list<CEdge*>::iterator eiter = m_edges.begin(); eiter != m_edges.end(); eiter ++ )
	{
		tEdge e = *eiter;
		if( e->string().size() > 0 )
		{
			fprintf(_os,"Edge %d %d {%s} \n", edgeVertex1(e)->id(), edgeVertex2(e)->id(), e->string().c_str() );
		}
	}

  for( std::list<CFace*>::iterator fiter = m_faces.begin(); fiter != m_faces.end(); fiter ++  )
	{
		tFace f = *fiter;

		tHalfEdge he = f->halfedge();

    do{
  			if( he->string().size() > 0 )
			  {
				  fprintf(_os,"Corner %d %d {%s}\n", he->vertex()->id(), f->id(), he->string().c_str() );
			  }
			  he = he->he_next();
		}while( he != f->halfedge() );

  }

	fclose(_os);
};

void CMesh::edgeSwap( CMesh::tEdge edge )
{

  CHalfEdge * he_left   = edge->halfedge( 0 );
  CHalfEdge * he_right = edge->halfedge( 1 );

  if( he_right == NULL )  return;
  
  CHalfEdge * ph[6];

  ph[0] = he_left;
  ph[1] = ph[0]->he_next();
  ph[2] = ph[1]->he_next();


  ph[3] = he_right;
  ph[4] = ph[3]->he_next();
  ph[5] = ph[4]->he_next();

  CVertex * pv[4];

  pv[0] = ph[0]->target();
  pv[1] = ph[1]->target();
  pv[2] = ph[2]->target();
  pv[3] = ph[4]->target();

  bool boundary[4];

  for( int i = 0; i < 4; i ++ )
  {
    boundary[i] = pv[0]->boundary();
  }

  int pi[6];
  CEdge * pe[6];
  for( int i = 0; i < 6; i ++ )
  {
    CHalfEdge * he = ph[i];
    CEdge        *   e= he->edge();
    pe[i] = e;

    if( e->halfedge(0) == he )
    {
      pi[i] = 0;
    }
    else
    {
      assert( e->halfedge(1) == he );
      pi[i] = 1;
    }
  }

  //relink the vertices

  ph[0]->target() = pv[1];
  if( !boundary[1] )
  {
    pv[1]->halfedge() = ph[0];
  }
  ph[1]->target() = pv[2];
  if( !boundary[2] )
  {
    pv[2]->halfedge() = ph[1];
  }
  
  ph[2]->target() = pv[3];
  if( !boundary[3] )
  {
    pv[3]->halfedge() = ph[2];
  }
  
  ph[3]->target() = pv[3];
  if( !boundary[3] )
  {
    pv[3]->halfedge() = ph[3];
  }
  
  ph[4]->target() = pv[0];
  if( !boundary[0] )
  {
    pv[0]->halfedge() = ph[4];
  }
  
  ph[5]->target() = pv[1];
  if( !boundary[1] )
  {
    pv[1]->halfedge() = ph[5];
  }
  
  //relink the edge-halfedge pointers

  ph[1]->edge() = pe[2];
  pe[2]->halfedge( pi[2] ) = ph[1];

  ph[2]->edge() = pe[4];
  pe[4]->halfedge( pi[4] ) = ph[2];

  ph[4]->edge() = pe[5];
  pe[5]->halfedge( pi[5] ) = ph[4];

  ph[5]->edge() = pe[1];
  pe[1]->halfedge( pi[1] ) = ph[5];
  
	CEdgeKey key(pv[0],pv[2]);
  m_map_edge.erase( key );
  
  CEdgeKey nkey(pv[1],pv[3]);
  m_map_edge.insert( std::pair<CEdgeKey,CEdge*>( nkey, pe[0] ) );

  for(int i = 0; i < 6; i ++ )
  {
      CHalfEdge * he = ph[i];
      CHalfEdge * sh = he->he_sym();
      assert( he->target() == sh->he_prev()->target() );
      assert( he->he_prev()->target() == sh->target() );
  }
}
