#include <stdio.h>
#include <stdlib.h>
#include<GL/glut.h>


#define _USE_MATH_DEFINES
#include <math.h>
#include "..\mesh/mesh.h"
#include "..\mesh/iterators.h"
#include "..\Trait/ViewerTrait.h"
#include "..\viewer/Arcball.h"                           /*  Arc Ball  Interface         */
#include<iostream>
#include <Eigen/Dense>
#include "..\FeatureComputation/FeatureComputation.h"

using namespace MeshLib;


bool getIntersectionPoint(CMesh::tVertex A_v, CMesh::tVertex B_v, CMesh::tVertex C_v, CMesh::tVertex D_v,
    CMesh::tVertex& intersection_v_2D, CMesh::tVertex& intersection_v_3D, CMesh& mesh_2D, CMesh& mesh_3D)
{
    CPoint A = A_v->point();
    CPoint B = B_v->point();
    CPoint C = C_v->point();
    CPoint D = D_v->point();

    CMesh::tVertex C_v_3D = mesh_3D.idVertex(C_v->id());
    CMesh::tVertex D_v_3D = mesh_3D.idVertex(D_v->id());
    CPoint C_3D = C_v_3D->point();
    CPoint D_3D = D_v_3D->point();


    //Get the RGB value of C_v and D_v
    CVertexTrait* pT = (CVertexTrait*)C_v->trait();
    double r_1 = pT->m_rgb[0];
    double g_1 = pT->m_rgb[1];
    double b_1 = pT->m_rgb[2];
    pT = (CVertexTrait*)D_v->trait();
    double r_2 = pT->m_rgb[0];
    double g_2 = pT->m_rgb[1];
    double b_2 = pT->m_rgb[2];

    CPoint C_r(r_1, g_1, b_1);
    CPoint D_r(r_2, g_2, b_2);

    //Get the RGB value of C_v_3D and D_v_3D
    pT = (CVertexTrait*)C_v_3D->trait();
    r_1 = pT->m_rgb[0];
    g_1 = pT->m_rgb[1];
    b_1 = pT->m_rgb[2];
    pT = (CVertexTrait*)D_v_3D->trait();
    r_2 = pT->m_rgb[0];
    g_2 = pT->m_rgb[1];
    b_2 = pT->m_rgb[2];

    CPoint C_r_3D(r_1, g_1, b_1);
    CPoint D_r_3D(r_2, g_2, b_2);



    CPoint intersection;
    CPoint intersection_r;

    //AC × AD
    double A_CD = ((A - C) ^ (A - D))[2];
    //BC × BD
    double B_CD = ((B - C) ^ (B - D))[2];
    //CA × CB
    double C_AB = ((C - A) ^ (C - B))[2];
    //DA × DB
    double D_AB = ((D - A) ^ (D - B))[2];
    if ((A_CD * B_CD) <= 0 && (C_AB * D_AB) <= 0) 
    {
        //CAB/(DAB+CAB)
        double t = std::abs(C_AB) / (std::abs(D_AB) + std::abs(C_AB));
        intersection_v_2D->point() = C + (D - C) * t;
        insert_RGB(intersection_v_2D,C_r+ (D_r - C_r) * t);

        intersection_v_3D->point() = C_3D + (D_3D - C_3D) * t;
        insert_RGB_3D(intersection_v_3D, C_r_3D + (D_r_3D - C_r_3D) * t, intersection_v_2D->point());
        return true;
    }
    else 
    {
		return false;
    }
}


void UnitReconstruction(CHalfEdge* halfedge1, CHalfEdge* halfedge2, CMesh& mesh_2D, CMesh& mesh_3D, int id0, int id1)
{
	CMesh::tVertex vertex0_2D = mesh_2D.idVertex(id0);
	CMesh::tVertex vertex1_2D = mesh_2D.idVertex(id1);
	CMesh::tVertex vertex0_3D = mesh_3D.idVertex(id0);
	CMesh::tVertex vertex1_3D = mesh_3D.idVertex(id1);

	//Initialize the cross edge
	CMesh::tEdge feature_line = mesh_2D.createEdge(vertex0_2D, vertex1_2D);
	CMesh::tEdge feature_line_3D = mesh_3D.createEdge(vertex0_3D, vertex1_3D);
	feature_line->string() = "sharp";
	feature_line_3D->string() = "sharp";

	//Find halfedges in 3D mesh
	int id_1 = halfedge1->source()->id();
	int id_2 = halfedge1->target()->id();
	CHalfEdge* halfedge1_3D = mesh_3D.vertexHalfedge(mesh_3D.idVertex(id_1), mesh_3D.idVertex(id_2));
	id_1 = halfedge2->source()->id();
	id_2 = halfedge2->target()->id();
	CHalfEdge* halfedge2_3D = mesh_3D.vertexHalfedge(mesh_3D.idVertex(id_1), mesh_3D.idVertex(id_2));

	if (halfedge1->target() == halfedge2->source())
	{
		//std::cout << "Type 1" << std::endl;
		//delete(halfedge2->he_next());
		halfedge2->he_next()->edge()->other(halfedge2->he_next()->he_sym()) = NULL;
		halfedge2_3D->he_next()->edge()->other(halfedge2_3D->he_next()->he_sym()) = NULL;

		//Create new faces in 2D mesh
		CVertex* v[3];
		v[0] = halfedge1->source();
		v[1] = vertex0_2D;
		v[2] = halfedge2->target();
		CMesh::tFace face1 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		//FaceInformation(face1);

		v[0] = vertex0_2D;
		v[1] = halfedge1->target();
		v[2] = vertex1_2D;
		CMesh::tFace face2 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		//FaceInformation(face2);

		v[0] = vertex1_2D;
		v[1] = halfedge2->target();
		v[2] = vertex0_2D;
		CMesh::tFace face3 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		//FaceInformation(face3);

		//Create new faces in 3D mesh
		v[0] = halfedge1_3D->source();
		v[1] = vertex0_3D;
		v[2] = halfedge2_3D->target();
		CMesh::tFace face1_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);
		//FaceInformation(face1_3D);

		v[0] = vertex0_3D;
		v[1] = halfedge1_3D->target();
		v[2] = vertex1_3D;
		CMesh::tFace face2_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);
		//FaceInformation(face2_3D);

		v[0] = vertex1_3D;
		v[1] = halfedge2_3D->target();
		v[2] = vertex0_3D;
		CMesh::tFace face3_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);
		//FaceInformation(face3_3D);

		if (halfedge1->edge()->string().find("sharp") != halfedge1->edge()->string().npos)
		{
			mesh_2D.vertexEdge(halfedge1->source(), vertex0_2D)->string() = "sharp";
			mesh_2D.vertexEdge(halfedge1->target(), vertex0_2D)->string() = "sharp";
		}
		if (halfedge1_3D->edge()->string().find("sharp") != halfedge1_3D->edge()->string().npos)
		{
			mesh_3D.vertexEdge(halfedge1_3D->source(), vertex0_3D)->string() = "sharp";
			mesh_3D.vertexEdge(halfedge1_3D->target(), vertex0_3D)->string() = "sharp";
		}

		//std::cout << "original face" << std::endl;
		//FaceInformation(halfedge1->face());
		//FaceInformation(halfedge1_3D->face());
		//销毁相应的边和面
		mesh_2D.edges().remove(halfedge1->edge());
		mesh_2D.map_edge().erase(CEdgeKey(halfedge1->source(), halfedge1->target()));
		mesh_2D.faces().remove(halfedge1->face());
		mesh_2D.map_face().erase(halfedge1->face()->id());

		mesh_3D.edges().remove(halfedge1_3D->edge());
		mesh_3D.map_edge().erase(CEdgeKey(halfedge1_3D->source(), halfedge1_3D->target()));
		mesh_3D.faces().remove(halfedge1_3D->face());
		mesh_3D.map_face().erase(halfedge1_3D->face()->id());


		if (halfedge2->edge()->boundary())
		{
			mesh_2D.edges().remove(halfedge2->edge());
			mesh_2D.map_edge().erase(CEdgeKey(halfedge2->source(), halfedge2->target()));
		}
		if (halfedge2_3D->edge()->boundary())
		{
			mesh_3D.edges().remove(halfedge2_3D->edge());
			mesh_3D.map_edge().erase(CEdgeKey(halfedge2_3D->source(), halfedge2_3D->target()));
		}
	}
	else
	{
		assert(halfedge1->source() == halfedge2->target());
		//std::cout << "Type2" << std::endl;
		//delete(halfedge1->he_next());
		halfedge1->he_next()->edge()->other(halfedge1->he_next()->he_sym()) = NULL;
		halfedge1_3D->he_next()->edge()->other(halfedge1_3D->he_next()->he_sym()) = NULL;

		CVertex* v[3];
		v[0] = halfedge1->source();
		v[1] = vertex0_2D;
		v[2] = vertex1_2D;
		CMesh::tFace face1 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		//FaceInformation(face1);

		v[0] = vertex0_2D;
		v[1] = halfedge1->target();
		v[2] = halfedge2->source();
		CMesh::tFace face2 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		//FaceInformation(face2);

		v[0] = vertex1_2D;
		v[1] = vertex0_2D;
		v[2] = halfedge2->source();
		CMesh::tFace face3 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		//FaceInformation(face3);

		v[0] = halfedge1_3D->source();
		v[1] = vertex0_3D;
		v[2] = vertex1_3D;
		CMesh::tFace face1_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);
		//FaceInformation(face1_3D);

		v[0] = vertex0_3D;
		v[1] = halfedge1_3D->target();
		v[2] = halfedge2_3D->source();
		CMesh::tFace face2_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);
		//FaceInformation(face2_3D);

		v[0] = vertex1_3D;
		v[1] = vertex0_3D;
		v[2] = halfedge2_3D->source();
		CMesh::tFace face3_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);
		//FaceInformation(face3_3D);


		if (halfedge1->edge()->string().find("sharp") != halfedge1->edge()->string().npos)
		{
			mesh_2D.vertexEdge(halfedge1->source(), vertex0_2D)->string() = "sharp";
			mesh_2D.vertexEdge(halfedge1->target(), vertex0_2D)->string() = "sharp";
		}
		if (halfedge1_3D->edge()->string().find("sharp") != halfedge1_3D->edge()->string().npos)
		{
			mesh_3D.vertexEdge(halfedge1_3D->source(), vertex0_3D)->string() = "sharp";
			mesh_3D.vertexEdge(halfedge1_3D->target(), vertex0_3D)->string() = "sharp";
		}

		//std::cout << "original face" << std::endl;
		//FaceInformation(halfedge1->face());
		//FaceInformation(halfedge1_3D->face());
		//销毁相应的边和面
		mesh_2D.edges().remove(halfedge1->edge());
		mesh_2D.map_edge().erase(CEdgeKey(halfedge1->source(), halfedge1->target()));
		mesh_2D.faces().remove(halfedge1->face());
		mesh_2D.map_face().erase(halfedge1->face()->id());
		mesh_3D.edges().remove(halfedge1_3D->edge());
		mesh_3D.map_edge().erase(CEdgeKey(halfedge1_3D->source(), halfedge1_3D->target()));
		mesh_3D.faces().remove(halfedge1_3D->face());
		mesh_3D.map_face().erase(halfedge1_3D->face()->id());


		if (halfedge2->edge()->boundary())
		{
			mesh_2D.edges().remove(halfedge2->edge());
			mesh_2D.map_edge().erase(CEdgeKey(halfedge2->source(), halfedge2->target()));
		}
		if (halfedge2_3D->edge()->boundary())
		{
			mesh_3D.edges().remove(halfedge2_3D->edge());
			mesh_3D.map_edge().erase(CEdgeKey(halfedge2_3D->source(), halfedge2_3D->target()));
		}
	}

	if (halfedge2->edge()->boundary())
	{
		vertex1_2D->boundary() = true;
		//std::cout << "boundary vertex" << std::endl;
		//std::cout << vertex2->id() << std::endl;
		CHalfEdge* he = vertex1_2D->halfedge();
		while (he->he_sym() != NULL)
		{
			he = he->ccw_rotate_about_target();
		}
		vertex1_2D->halfedge() = he;
	}

	if (halfedge1->edge()->boundary())
	{
		vertex0_2D->boundary() = true;
		CHalfEdge* he = vertex0_2D->halfedge();
		while (he->he_sym() != NULL)
		{
			he = he->ccw_rotate_about_target();
		}
		vertex0_2D->halfedge() = he;
	}


	//std::cout << "**************************************************" << std::endl;

}


bool parallel(CPoint A, CPoint B, CPoint C, CPoint D)
{
	CPoint AB = B - A;
	CPoint CD = D - C;
	if (AB[0] * CD[1] == AB[1] * CD[0])
	{
		return true;
	}
	else
	{
		return false;
	}
}
bool between(CMesh::tVertex vertex0, CMesh::tVertex vertex_temp, CMesh::tVertex vertex1)
{
	if ((vertex_temp->point()[0] - vertex0->point()[0]) * (vertex_temp->point()[0] - vertex1->point()[0]) < 0
		&& (vertex_temp->point()[1] - vertex0->point()[1]) * (vertex_temp->point()[1] - vertex1->point()[1]) < 0
		)
		return true;
	else
		return false;
}

void Link(CMesh::tVertex vertex0, CMesh::tVertex vertex1, CMesh& mesh_2D, CMesh& mesh_3D)
{
	if (vertex0 != vertex1)
	{
		CPoint A = vertex0->point();
		CPoint B = vertex1->point();
		std::list<CMesh::tFace> face_list;
		for (VertexFaceIterator vfiter(vertex0); !vfiter.end(); ++vfiter)
		{
			CMesh::tFace face = *vfiter;
			face_list.push_back(face);
		}

		std::list<CMesh::tEdge> edge_list;
		for (VertexEdgeIterator veiter(vertex0); !veiter.end(); ++veiter)
		{
			CMesh::tEdge edge = *veiter;
			/*CMesh::tVertex vertex_temp;
			if (edge->halfedge(0)->source() = vertex0)
			{
				vertex_temp = edge->halfedge(0)->target();
			}
			else
			{
				vertex_temp = edge->halfedge(0)->source();
			}
			if (parallel(vertex0->point(), vertex1->point(), vertex0->point(), vertex_temp->point()))
			{
				if (between(vertex0, vertex_temp, vertex1))
				{
					Link(vertex_temp, vertex1, mesh_2D, mesh_3D);
					return;
				}
			}*/
			edge_list.push_back(edge);
		}

		CHalfEdge* cross_halfedge;
		CMesh::tVertex vertex_intersection_2D = mesh_2D.createVertex(mesh_2D.vertices().back()->id() + 1);
		CMesh::tVertex vertex_intersection_3D = mesh_3D.createVertex(mesh_3D.vertices().back()->id() + 1);
		//找到与特征线相交的半边
		for (std::list<CMesh::tFace>::iterator it = face_list.begin(); it != face_list.end(); ++it)
		{
			CMesh::tFace face = *it;
			for (FaceHalfedgeIterator fhiter(face); !fhiter.end(); ++fhiter)
			{
				CHalfEdge* halfedge = *fhiter;
				if (std::find(edge_list.begin(), edge_list.end(), halfedge->edge()) == edge_list.end() &&
					getIntersectionPoint(vertex0, vertex1, halfedge->source(), halfedge->target(),
						vertex_intersection_2D, vertex_intersection_3D, mesh_2D, mesh_3D) == true)
				{
					cross_halfedge = halfedge;
					break;
				}
			}
		}
		assert(cross_halfedge != NULL);

		CHalfEdge* cross_halfedge_3D = mesh_3D.vertexHalfedge(mesh_3D.idVertex(cross_halfedge->source()->id()), mesh_3D.idVertex(cross_halfedge->target()->id()));
		//重构第一个三角形
		CMesh::tVertex vertex0_3D = mesh_3D.idVertex(vertex0->id());
		CMesh::tEdge feature_line = mesh_2D.createEdge(vertex0, vertex_intersection_2D);
		CMesh::tEdge feature_line_3D = mesh_3D.createEdge(vertex0_3D, vertex_intersection_3D);
		feature_line->string() = "sharp";
		feature_line_3D->string() = "sharp";

		//重构第一个面
		CVertex* v[3];
		v[0] = cross_halfedge->source();
		v[1] = vertex_intersection_2D;
		v[2] = vertex0;
		CMesh::tFace face1 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);

		v[0] = vertex_intersection_2D;
		v[1] = cross_halfedge->target();
		v[2] = vertex0;
		CMesh::tFace face2 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);

		v[0] = cross_halfedge_3D->source();
		v[1] = vertex_intersection_3D;
		v[2] = vertex0_3D;
		CMesh::tFace face1_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);

		v[0] = vertex_intersection_3D;
		v[1] = cross_halfedge_3D->target();
		v[2] = vertex0_3D;
		CMesh::tFace face2_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);

		mesh_2D.faces().remove(cross_halfedge->face());
		mesh_2D.map_face().erase(cross_halfedge->face()->id());
		mesh_3D.faces().remove(cross_halfedge_3D->face());
		mesh_3D.map_face().erase(cross_halfedge_3D->face()->id());

		if (cross_halfedge->edge()->boundary())
		{
			mesh_2D.edges().remove(cross_halfedge->edge());
			mesh_2D.map_edge().erase(CEdgeKey(cross_halfedge->source(), cross_halfedge->target()));
			mesh_3D.edges().remove(cross_halfedge_3D->edge());
			mesh_3D.map_edge().erase(CEdgeKey(cross_halfedge_3D->source(), cross_halfedge_3D->target()));

			vertex1->boundary() = true;

			CHalfEdge* he = vertex1->halfedge();
			while (he->he_sym() != NULL)
			{
				he = he->ccw_rotate_about_target();
			}
			vertex1->halfedge() = he;
		}
		cross_halfedge = cross_halfedge->he_sym();
		CHalfEdge* cross_halfedge_next;
		while (true)
		{
			if (cross_halfedge->he_next()->target() != vertex1)
			{
				vertex_intersection_2D = mesh_2D.createVertex(mesh_2D.vertices().back()->id() + 1);
				vertex_intersection_3D = mesh_3D.createVertex(mesh_3D.vertices().back()->id() + 1);
				int id = vertex_intersection_2D->id();
				cross_halfedge_next = cross_halfedge->he_next();
				if (getIntersectionPoint(vertex0, vertex1, cross_halfedge_next->source(), cross_halfedge_next->target(),
					vertex_intersection_2D, vertex_intersection_3D, mesh_2D, mesh_3D) == true)
				{
					UnitReconstruction(cross_halfedge, cross_halfedge_next, mesh_2D, mesh_3D, id - 1, id);
					cross_halfedge = cross_halfedge_next->he_sym();
				}
				else
				{
					cross_halfedge_next = cross_halfedge_next->he_next();
					getIntersectionPoint(vertex0, vertex1, cross_halfedge_next->source(), cross_halfedge_next->target(),
						vertex_intersection_2D, vertex_intersection_3D, mesh_2D, mesh_3D);
					UnitReconstruction(cross_halfedge, cross_halfedge_next, mesh_2D, mesh_3D, id - 1, id);
					cross_halfedge = cross_halfedge_next->he_sym();
				}
			}
			else
			{
				cross_halfedge_3D = mesh_3D.vertexHalfedge(mesh_3D.idVertex(cross_halfedge->source()->id()), mesh_3D.idVertex(cross_halfedge->target()->id()));
				//重构最后一个三角形
				CMesh::tVertex vertex1_3D = mesh_3D.idVertex(vertex1->id());
				feature_line = mesh_2D.createEdge(vertex1, vertex_intersection_2D);
				feature_line_3D = mesh_3D.createEdge(vertex1_3D, vertex_intersection_3D);
				feature_line->string() = "sharp";
				feature_line_3D->string() = "sharp";

				//重构第一个面

				v[0] = cross_halfedge->source();
				v[1] = vertex_intersection_2D;
				v[2] = vertex1;
				CMesh::tFace face1 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);

				v[0] = vertex_intersection_2D;
				v[1] = cross_halfedge->target();
				v[2] = vertex1;
				CMesh::tFace face2 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);

				v[0] = cross_halfedge_3D->source();
				v[1] = vertex_intersection_3D;
				v[2] = vertex1_3D;
				CMesh::tFace face1_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);

				v[0] = vertex_intersection_3D;
				v[1] = cross_halfedge_3D->target();
				v[2] = vertex1_3D;
				CMesh::tFace face2_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);

				mesh_2D.faces().remove(cross_halfedge->face());
				mesh_2D.map_face().erase(cross_halfedge->face()->id());
				mesh_3D.faces().remove(cross_halfedge_3D->face());
				mesh_3D.map_face().erase(cross_halfedge_3D->face()->id());

				break;
			}

		}
	}
    

}

void Extraction(std::list<int> Feature_Point_id, CMesh& mesh_2D, CMesh& mesh_3D)
{
	std::list<int>::iterator it = Feature_Point_id.begin();
    CMesh::tVertex vertex0 = mesh_2D.idVertex(*it);
    it++;

    for (; it != Feature_Point_id.end(); it++)
    {
        CMesh::tVertex vertex1 = mesh_2D.idVertex(*it);
        Link(vertex0, vertex1, mesh_2D, mesh_3D);
		vertex0 = vertex1;
    }
}


double getDegAngle3d(const Eigen::Vector3d v1, const Eigen::Vector3d v2)
{
	double radian = atan2(v1.cross(v2).norm(), v1.transpose() * v2); //弧度
	if (v1.cross(v2).z() < 0)
	{
		radian = 2 * M_PI - radian;
	}

	return radian * 180 / M_PI; //角度
}

CPoint vec(CMesh::tEdge edge, CMesh::tVertex vertex)
{
	CMesh::tVertex other;
	if (edge->halfedge(0)->source() == vertex)
	{
		other = edge->halfedge(0)->target();
	}
	else
	{
		other = edge->halfedge(0)->source();
	}
	CPoint vec = (other->point() - vertex->point());
	return vec;
}

std::list<double> AngleList(CMesh& mesh_3D)
{
	std::list<CMesh::tVertex> GridVertex;
	std::list<double> angle_list;
	for (MeshVertexIterator viter(&mesh_3D); !viter.end(); ++viter)
	{
		std::list<CMesh::tEdge> edge_list;
		CVertex* vertex = *viter;
		bool grid = false;
		for (VertexEdgeIterator veiter(vertex); !veiter.end(); ++veiter)
		{
			CEdge* edge = *veiter;
			if (edge->string().find("sharp") != edge->string().npos)
			{
				edge_list.push_back(edge);
			}

		}
		if (edge_list.size() == 4)
		{
			GridVertex.push_back(vertex);
			CMesh::tEdge edge1 = *edge_list.begin();
			CMesh::tEdge edge2 = *std::next(edge_list.begin());
			CPoint vec1 = vec(edge1, vertex);
			CPoint vec2 = vec(edge2, vertex);

			Eigen::Vector3d v1(vec1[0], vec1[1], vec1[2]);
			Eigen::Vector3d v2(vec2[0], vec2[1], vec2[2]);

			double angle = getDegAngle3d(v1, v2);
			angle_list.push_back(angle);
		}
	}
	return angle_list;
}