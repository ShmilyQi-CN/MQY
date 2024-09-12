#include "HalfEdge.h"

using namespace MeshLib;


CHalfEdge * CHalfEdge::ccw_rotate_about_target()
{
	CHalfEdge * he_dual = he_sym();
	if( he_dual == NULL ) return NULL;

	return he_dual->he_prev();
};

CHalfEdge * CHalfEdge::clw_rotate_about_target()
{
	CHalfEdge * he = he_next()->he_sym();
	return he;
};

CHalfEdge * CHalfEdge::ccw_rotate_about_source()
{

	CHalfEdge * he = he_prev()->he_sym();
	return he;
};

CHalfEdge * CHalfEdge::clw_rotate_about_source()
{
	CHalfEdge * he = he_sym();
	if( he == NULL ) return NULL;
	return he->he_next();
};
