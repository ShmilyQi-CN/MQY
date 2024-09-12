
#include "Vertex.h"
#include "HalfEdge.h"

using namespace MeshLib;

//most counter clockwise in halfedge

CHalfEdge *  CVertex::most_ccw_in_halfedge()  
{ 
	if( !m_boundary )
	{
		return m_halfedge; //current half edge is the most ccw in halfedge 
	}

	CHalfEdge * he = m_halfedge->ccw_rotate_about_target();
	
	while( he != NULL )
	{
		m_halfedge = he;
		he = m_halfedge->ccw_rotate_about_target();
	}

	return m_halfedge;
};

//most clockwise in halfedge

CHalfEdge *  CVertex::most_clw_in_halfedge()  
{ 
	if( !m_boundary )
	{
		return most_ccw_in_halfedge()->ccw_rotate_about_target(); 
	}

	CHalfEdge * he = m_halfedge->clw_rotate_about_target();
	
	while( he != NULL )
	{
		m_halfedge = he;
		he = m_halfedge->clw_rotate_about_target();
	}

	return m_halfedge;
};

//most counter clockwise out halfedge

CHalfEdge *  CVertex::most_ccw_out_halfedge()  
{ 
	if( !m_boundary )
	{
		return most_ccw_in_halfedge()->he_sym(); 
	}

	CHalfEdge * he = m_halfedge->he_next();
	CHalfEdge * ne = he->ccw_rotate_about_source();

	while( ne != NULL )
	{
		he = ne;
		ne = he->ccw_rotate_about_source();
	}

	return he;
};

//most clockwise out halfedge

CHalfEdge * CVertex::most_clw_out_halfedge()  
{ 
	if( !m_boundary )
	{
		return most_ccw_out_halfedge()->ccw_rotate_about_source(); 
	}

	CHalfEdge * he = m_halfedge->he_next();
	CHalfEdge * ne = he->clw_rotate_about_source();
	
	while( ne != NULL )
	{
		he = ne;
		ne = he->clw_rotate_about_source();
	}

	return he;
};


