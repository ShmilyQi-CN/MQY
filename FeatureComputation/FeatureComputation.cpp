#include <stdio.h>
#include <stdlib.h>
#include<GL/glut.h>

#include <math.h>
#include "..\mesh/mesh.h"
#include "..\mesh/iterators.h"
#include "..\Trait/ViewerTrait.h"
#include "..\viewer/Arcball.h"                           /*  Arc Ball  Interface         */
#include<iostream>
#include <Eigen/Dense>

using namespace MeshLib;


void create_featureline_3D(CMesh& mesh_2D, CMesh& mesh_3D, double ftr, CHalfEdge* initial_halfbound, CMesh::tVertex& vertex1, const std::string& string);
void extend_featureline_3D(CVertex*& vertex, CMesh& mesh_2D, CMesh& mesh_3D, double ftr, CMesh::tEdge& edge, const std::string& string);
void draw_featureline_3D(CMesh& mesh_2D, CMesh& mesh_3D, const std::string& string, double ftr);

//********************************************************************************
//Auxiliary function
void FaceInformation(const CMesh::tFace& face)
{
	std::cout << "face id: " << face->id() << std::endl;
	//std::cout << "face string: " << face->string() << std::endl;
	//std::cout << "face halfedge: " << face->halfedge()->source()->id() << " " << face->halfedge()->target()->id() << " " << face->halfedge()->he_next()->target()->id() << std::endl;
}
bool judge(const CMesh::tEdge& edge, const std::string& string)
{
	if (string == "horizontal")
	{
		//return (fabs(edge->halfedge(0)->source()->point()[0] - edge->halfedge(0)->target()->point()[0]) < 0.0001);
		return (fabs(edge->halfedge(0)->source()->point()[0] - edge->halfedge(0)->target()->point()[0]) == 0);

	}
	else
	{
		assert(string == "vertical");
		//return fabs(edge->halfedge(0)->source()->point()[1] - edge->halfedge(0)->target()->point()[1]) < 0.0001;
		return fabs(edge->halfedge(0)->source()->point()[1] - edge->halfedge(0)->target()->point()[1]) == 0;

	}
}//Judge if the feature line is horizontal or vertical

bool check(double k, double j)
{
	if ((k > 0 && j < 0) || (k < 0 && j > 0))
		return true;
	else return false;

}
bool ifcross(CHalfEdge* halfedge, double k, const std::string& string)
{

	if (string == "horizontal")
	{
		double y_1 = halfedge->source()->point()[0];
		double y_2 = halfedge->target()->point()[0];
		return check(k - y_1, k - y_2);
	}
	else
	{
		assert(string == "vertical");
		double x_1 = halfedge->source()->point()[1];
		double x_2 = halfedge->target()->point()[1];
		return check(k - x_1, k - x_2);
	}
}//Judge if the edge across the k-feature line
std::list<double> PointSelection(std::list<double> list)
{
	std::list<double>::iterator it = list.begin();
	std::list<double> interval_list;
	double x_0 = *it;
	it++;
	while (it != list.end())
	{
		double x_1 = *it;
		int k = std::round((x_1 - x_0) * 100 /2.8);
		double interval = (x_1 - x_0) / k;
		for(int i=1; i < k; i++)
		{
			interval_list.push_back(x_0 + interval * i);
		}
		x_0 = x_1;
		it++;
	}

	return interval_list;
	
}
std::list<double> PointSelection_3D(std::list<double> list)
{
	std::list<double>::iterator it = list.begin();
	std::list<double> interval_list;
	double x_0 = *it;
	it++;
	while (it != list.end())
	{
		double x_1 = *it;
		int k = std::round((x_1 - x_0) / 0.05);
		double interval = (x_1 - x_0) / k;
		for (int i = 1; i < k; i++)
		{
			interval_list.push_back(x_0 + interval * i);
		}
		x_0 = x_1;
		it++;
	}

	return interval_list;

}
void findPreviousAndNext(std::vector<double> A, double a, double& prev, double& next) {
	// Find the position of 'a' in the list
	auto it = std::find(A.begin(), A.end(), a);

	// If 'a' is found in the list
	if (it != A.end()) {
		// Find the index of 'a'
		int index = std::distance(A.begin(), it);

		// Find the previous element if 'a' is not the first element
		if (index > 0) {
			//std::cout << "Previous element of " << a << " is: " << A[index - 1] << std::endl;
			prev = A[index - 1];
		}
		else {
			//std::cout << "There is no previous element of " << a << " as it is the first element." << std::endl;
			prev = a;
		}

		// Find the next element if 'a' is not the last element
		if (index < A.size() - 1) {
			//std::cout << "Next element of " << a << " is: " << A[index + 1] << std::endl;
			next = A[index + 1];
		}
		else {
			//std::cout << "There is no next element of " << a << " as it is the last element." << std::endl;
			next = a;
		}
	}
	else {
		//std::cout << "Element " << a << " not found in the list." << std::endl;
	}
}
double distance(CMesh::tVertex vertex0_2D, CMesh::tVertex vertex1_2D, CMesh& mesh_3D)
{
	int id1 = vertex0_2D->id();
	int id2 = vertex1_2D->id();
	
	CPoint a = mesh_3D.idVertex(id1)->point();
	CPoint b = mesh_3D.idVertex(id2)->point();
	return sqrt(pow(a[0] - b[0], 2) + pow(a[1] - b[1], 2) + pow(a[2] - b[2], 2));
}
void generate_quadmesh_3D(CMesh& mesh_2D, CMesh& mesh_3D,
	std::list<CHalfEdge*> halfedge_list, std::list<CMesh::tVertex> feature_vertex )
{
	//输出该条边界的顶点信息
	std::cout << "vertex id       coordinates" << std::endl;
	for (std::list<CHalfEdge*>::iterator it = halfedge_list.begin(); it != halfedge_list.end(); it++)
	{
		std::cout << (*it)->target()->id() << "       " << (*it)->target()->point()[0] << " " << (*it)->target()->point()[1] << std::endl;
	}



	struct data
	{
		CMesh::tVertex vertex;
		double distance;
	};


	std::list<double> feature_distance;
	std::list<data> vertex_data;
	CMesh::tVertex vertex0;
	CMesh::tVertex vertex1;
	double distance_sum = 0;
	std::string direction;

	std::list<CHalfEdge*>::iterator it = halfedge_list.begin();
	
	//判断边界的方向
	if (judge((*it)->edge(), "horizontal"))
		direction = "horizontal";
	else
		direction = "vertical";


	//计算三维上各点到起始点的距离
	feature_distance.push_back(0);
	vertex0 = (*it)->source();
	std::cout << "distance" << std::endl;
	for (; it != halfedge_list.end(); it++)
	{
		vertex1 = (*it)->target();
		distance_sum += distance(vertex0, vertex1, mesh_3D);
		std::cout << vertex0->id() << " " << vertex1->id() << " " << distance_sum << std::endl;
		
		vertex_data.push_back({ vertex1, distance_sum });
		if(std::find(feature_vertex.begin(), feature_vertex.end(), vertex1)!=feature_vertex.end())
			feature_distance.push_back(distance_sum);
		vertex0 = vertex1;
	}
	feature_distance.push_back(distance_sum);

	//输出feature_distance成员
	std::cout << "feature_distance" << std::endl;
	for (auto it = feature_distance.begin(); it != feature_distance.end(); it++)
	{
		std::cout << *it << std::endl;
	}


	//输出三维总长度
	std::cout << "sum" << std::endl;
	std::cout << distance_sum << std::endl;
	
	
	
	std::list<double> Selection = PointSelection_3D(feature_distance);



	//输出三维长度上的选点
	for (auto it = Selection.begin(); it != Selection.end(); it++)
	{
		std::cout << *it << std::endl;
	}

	//找到Selection中的点在2D中的坐标
	for (auto it = Selection.begin(); it != Selection.end(); it++)
	{
		double ratio;//插值比例
		bool find = false;
		std::list<data>::iterator it1 = vertex_data.begin();
		
		CMesh::tVertex vertex_temp0 = halfedge_list.front()->source();
		double distance_front = 0;

		for (; it1 != vertex_data.end(); it1++)
		{
			double distance_back = (*it1).distance;
			CMesh::tVertex vertex_temp1 = (*it1).vertex;
			if (find == false && distance_back > *it)
			{
				std::cout << vertex_temp0->id() << " " << vertex_temp1->id() << std::endl;
				ratio = (*it - distance_front) / (distance_back - distance_front);
				find = true;
				if (direction == "horizontal")
				{
					double v_0 = vertex_temp0->point()[1];
					double v_1 = vertex_temp1->point()[1];
					double cor = v_0 + ratio * (v_1 - v_0);
					std::cout << "cor" << std::endl;
					std::cout << cor << std::endl;
					draw_featureline_3D(mesh_2D, mesh_3D, "vertical", cor);
				}
				else
				{
					double v_0 = vertex_temp0->point()[0];
					double v_1 = vertex_temp1->point()[0];
					double cor = v_0 + ratio * (v_1 - v_0);
					std::cout << "cor" << std::endl;
					std::cout << cor << std::endl;
					draw_featureline_3D(mesh_2D, mesh_3D, "horizontal", cor);
				}
			}
			distance_front = distance_back;
			vertex_temp0 = vertex_temp1;
		}
	}
}


//********************************************************************************


CPoint crosspoint(CHalfEdge* halfedge, double k, const std::string& string)//添加新的顶点
{
	double y_1 = halfedge->source()->point()[0];
	double x_1 = halfedge->source()->point()[1];

	double y_2 = halfedge->target()->point()[0];
	double x_2 = halfedge->target()->point()[1];

	if (string == "horizontal")
	{
		double value = fabs(y_1 - y_2);
		double value_1 = fabs(k - y_1);
		double value_2 = fabs(k - y_2);
		CPoint point;
		point[0] = k;
		point[1] = x_1 * (value_2 / value) + x_2 * (value_1 / value);
		point[2] = 0;
		return point;
	}
	if (string == "vertical")
	{
		double value = fabs(x_1 - x_2);
		double value_1 = fabs(k - x_1);
		double value_2 = fabs(k - x_2);
		CPoint point;
		point[0] = y_1 * (value_2 / value) + y_2 * (value_1 / value);
		point[1] = k;
		point[2] = 0;
		return point;
	}
}
CPoint crosspoint_3D(CMesh& mesh_3D, CHalfEdge* halfedge, double k, const std::string& string)
{

	int id_1 = halfedge->source()->id();
	int id_2 = halfedge->target()->id();

	CMesh::tVertex vertex1 = mesh_3D.idVertex(id_1);
	CMesh::tVertex vertex2 = mesh_3D.idVertex(id_2);
	double y_1 = halfedge->source()->point()[0];
	double x_1 = halfedge->source()->point()[1];

	double y_2 = halfedge->target()->point()[0];
	double x_2 = halfedge->target()->point()[1];

	if (string == "horizontal")
	{
		double value = fabs(y_1 - y_2);
		double value_1 = fabs(k - y_1);
		double value_2 = fabs(k - y_2);
		CPoint point;
		point[0] = vertex1->point()[0] * (value_2 / value) + vertex2->point()[0] * (value_1 / value);
		point[1] = vertex1->point()[1] * (value_2 / value) + vertex2->point()[1] * (value_1 / value);
		point[2] = vertex1->point()[2] * (value_2 / value) + vertex2->point()[2] * (value_1 / value);
		return point;
	}
	if (string == "vertical")
	{
		double value = fabs(x_1 - x_2);
		double value_1 = fabs(k - x_1);
		double value_2 = fabs(k - x_2);
		CPoint point;
		point[0] = vertex1->point()[0] * (value_2 / value) + vertex2->point()[0] * (value_1 / value);
		point[1] = vertex1->point()[1] * (value_2 / value) + vertex2->point()[1] * (value_1 / value);
		point[2] = vertex1->point()[2] * (value_2 / value) + vertex2->point()[2] * (value_1 / value);
		return point;
	}
}


CPoint crosspoint_RGB(CHalfEdge* halfedge, double k, const std::string& string)
{
	double y_1 = halfedge->source()->point()[0];
	double y_2 = halfedge->target()->point()[0];
	double x_1 = halfedge->source()->point()[1];
	double x_2 = halfedge->target()->point()[1];
	//读取RGB值
	CVertexTrait* pT = (CVertexTrait*)halfedge->source()->trait();
	double r_1 = pT->m_rgb[0];
	double g_1 = pT->m_rgb[1];
	double b_1 = pT->m_rgb[2];
	pT = (CVertexTrait*)halfedge->target()->trait();
	double r_2 = pT->m_rgb[0];
	double g_2 = pT->m_rgb[1];
	double b_2 = pT->m_rgb[2];
	//插值
	if (string == "horizontal")
	{
		CPoint point;
		double value = fabs(y_1 - y_2);
		double value_1 = fabs(k - y_1);
		double value_2 = fabs(k - y_2);
		point[0] = r_1 * (value_2 / value) + r_2 * (value_1 / value);
		point[1] = g_1 * (value_2 / value) + g_2 * (value_1 / value);
		point[2] = b_1 * (value_2 / value) + b_2 * (value_1 / value);
		return point;
	}
	if (string == "vertical")
	{
		CPoint point;
		double value = fabs(x_1 - x_2);
		double value_1 = fabs(k - x_1);
		double value_2 = fabs(k - x_2);
		point[0] = r_1 * (value_2 / value) + r_2 * (value_1 / value);
		point[1] = g_1 * (value_2 / value) + g_2 * (value_1 / value);
		point[2] = b_1 * (value_2 / value) + b_2 * (value_1 / value);
		return point;
	}


}
CPoint crosspoint_RGB_3D(CMesh& mesh_3D, CHalfEdge* halfedge, double k, const std::string& string)
{
	int id_1 = halfedge->source()->id();
	int id_2 = halfedge->target()->id();
	CMesh::tVertex vertex1 = mesh_3D.idVertex(id_1);
	CMesh::tVertex vertex2 = mesh_3D.idVertex(id_2);

	double y_1 = halfedge->source()->point()[0];
	double y_2 = halfedge->target()->point()[0];
	double x_1 = halfedge->source()->point()[1];
	double x_2 = halfedge->target()->point()[1];
	//读取RGB值
	CVertexTrait* pT = (CVertexTrait*)vertex1->trait();
	double r_1 = pT->m_rgb[0];
	double g_1 = pT->m_rgb[1];
	double b_1 = pT->m_rgb[2];
	pT = (CVertexTrait*)vertex2->trait();
	double r_2 = pT->m_rgb[0];
	double g_2 = pT->m_rgb[1];
	double b_2 = pT->m_rgb[2];
	//插值
	if (string == "horizontal")
	{
		CPoint point;
		double value = fabs(y_1 - y_2);
		double value_1 = fabs(k - y_1);
		double value_2 = fabs(k - y_2);
		point[0] = r_1 * (value_2 / value) + r_2 * (value_1 / value);
		point[1] = g_1 * (value_2 / value) + g_2 * (value_1 / value);
		point[2] = b_1 * (value_2 / value) + b_2 * (value_1 / value);
		return point;
	}
	if (string == "vertical")
	{
		CPoint point;
		double value = fabs(x_1 - x_2);
		double value_1 = fabs(k - x_1);
		double value_2 = fabs(k - x_2);
		point[0] = r_1 * (value_2 / value) + r_2 * (value_1 / value);
		point[1] = g_1 * (value_2 / value) + g_2 * (value_1 / value);
		point[2] = b_1 * (value_2 / value) + b_2 * (value_1 / value);
		return point;
	}


}

void insert_RGB(CMesh::tVertex& vertex, CPoint RGB)
{
	CVertexTrait* vertex_trait = new CVertexTrait;
	vertex->trait() = (CTrait*)vertex_trait;
	v_rgb(vertex) = RGB;
	vertex->string() = "rgb=(" + std::to_string(RGB[0]) + " " + std::to_string(RGB[1]) + " " + std::to_string(RGB[2]) + ")";
}
void insert_RGB_3D(CMesh::tVertex& vertex, CPoint RGB, CPoint point)
{
	CVertexTrait* vertex_trait = new CVertexTrait;
	vertex->trait() = (CTrait*)vertex_trait;
	v_rgb(vertex) = RGB;
	CPoint2 uv;
	uv[0] = point[0];
	uv[1] = point[1];
	v_uv(vertex) = uv;
	vertex->string() = "rgb=(" + std::to_string(RGB[0]) + " " + std::to_string(RGB[1]) + " " + std::to_string(RGB[2]) + ") " +
		"uv=(" + std::to_string(point[0]) + " " + std::to_string(point[1]) + ")";
}




CMesh::tVertex create_new_mesh(CHalfEdge* halfedge1, CHalfEdge* halfedge2, double k, CMesh& mesh_2D, CMesh::tVertex& vertex1, const std::string& string)
{
	//初始化下一个交点
	std::cout << "**************************************************" << std::endl;
	std::cout << "create_new_mesh" << std::endl;
	CMesh::tVertex vertex2 = mesh_2D.createVertex(mesh_2D.vertices().back()->id() + 1);
	vertex2->point() = crosspoint(halfedge2, k, string);


	CPoint vertex2_rgb = crosspoint_RGB(halfedge2, k, string);
	insert_RGB(vertex2, vertex2_rgb);


	CMesh::tEdge feature_line = mesh_2D.createEdge(vertex1, vertex2);
	feature_line->string() = "sharp";

	if (halfedge1->target() == halfedge2->source())
	{
		std::cout << "type 1" << std::endl;
		//delete(halfedge2->he_next());
		halfedge2->he_next()->edge()->other(halfedge2->he_next()->he_sym()) = NULL;
		CVertex* v[3];
		v[0] = halfedge1->source();
		v[1] = vertex1;
		v[2] = halfedge2->target();
		CMesh::tFace face1 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		FaceInformation(face1);

		v[0] = vertex1;
		v[1] = halfedge1->target();
		v[2] = vertex2;
		CMesh::tFace face2 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		FaceInformation(face2);

		v[0] = vertex2;
		v[1] = halfedge2->target();
		v[2] = vertex1;
		CMesh::tFace face3 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		FaceInformation(face3);

		if (halfedge1->edge()->string().find("sharp") != halfedge1->edge()->string().npos)
		{
			mesh_2D.vertexEdge(halfedge1->source(), vertex1)->string() = "sharp";
			mesh_2D.vertexEdge(halfedge1->target(), vertex1)->string() = "sharp";
		}

		std::cout << "original face" << std::endl;
		FaceInformation(halfedge1->face());
		//销毁相应的边和面
		mesh_2D.edges().remove(halfedge1->edge());
		mesh_2D.map_edge().erase(CEdgeKey(halfedge1->source(), halfedge1->target()));
		mesh_2D.faces().remove(halfedge1->face());
		mesh_2D.map_face().erase(halfedge1->face()->id());
		//map也要删

		if (halfedge2->edge()->boundary())
		{
			mesh_2D.edges().remove(halfedge2->edge());
			mesh_2D.map_edge().erase(CEdgeKey(halfedge2->source(), halfedge2->target()));

		}
	}
	else
	{
		assert(halfedge1->source() == halfedge2->target());
		std::cout << "type2" << std::endl;
		//delete(halfedge1->he_next());
		halfedge1->he_next()->edge()->other(halfedge1->he_next()->he_sym()) = NULL;
		CVertex* v[3];
		v[0] = halfedge1->source();
		v[1] = vertex1;
		v[2] = vertex2;
		CMesh::tFace face1 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		FaceInformation(face1);

		v[0] = vertex1;
		v[1] = halfedge1->target();
		v[2] = halfedge2->source();
		CMesh::tFace face2 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		FaceInformation(face2);

		v[0] = vertex2;
		v[1] = vertex1;
		v[2] = halfedge2->source();
		CMesh::tFace face3 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		FaceInformation(face3);

		if (halfedge1->edge()->string().find("sharp") != halfedge1->edge()->string().npos)
		{
			mesh_2D.vertexEdge(halfedge1->source(), vertex1)->string() = "sharp";
			mesh_2D.vertexEdge(halfedge1->target(), vertex1)->string() = "sharp";
		}

		std::cout << "original face" << std::endl;
		FaceInformation(halfedge1->face());
		//销毁相应的边和面
		mesh_2D.edges().remove(halfedge1->edge());
		mesh_2D.map_edge().erase(CEdgeKey(halfedge1->source(), halfedge1->target()));
		mesh_2D.faces().remove(halfedge1->face());
		mesh_2D.map_face().erase(halfedge1->face()->id());
		if (halfedge2->edge()->boundary())
		{
			mesh_2D.edges().remove(halfedge2->edge());
			mesh_2D.map_edge().erase(CEdgeKey(halfedge2->source(), halfedge2->target()));
		}
	}


	std::cout << "**************************************************" << std::endl;
	return vertex2;
}


//vertex1 a 2D-vertex
CMesh::tVertex create_new_mesh_3D(CHalfEdge* halfedge1, CHalfEdge* halfedge2, double k, CMesh& mesh_2D, CMesh& mesh_3D, CMesh::tVertex& vertex1, const std::string& string)
{
	//初始化下一个交点
	//std::cout << "**************************************************" << std::endl;
	//std::cout << "create_new_mesh" << std::endl;



	CMesh::tVertex vertex1_3D = mesh_3D.idVertex(vertex1->id());
	CMesh::tVertex vertex2 = mesh_2D.createVertex(mesh_2D.vertices().back()->id() + 1);
	CMesh::tVertex vertex2_3D = mesh_3D.createVertex(mesh_3D.vertices().back()->id() + 1);

	//Initialize the new vertices
	vertex2->point() = crosspoint(halfedge2, k, string);
	vertex2_3D->point() = crosspoint_3D(mesh_3D, halfedge2, k, string);
	CPoint vertex2_rgb = crosspoint_RGB(halfedge2, k, string);
	CPoint vertex2_3D_rgb = crosspoint_RGB_3D(mesh_3D, halfedge2, k, string);
	insert_RGB(vertex2, vertex2_rgb);
	insert_RGB_3D(vertex2_3D, vertex2_3D_rgb, vertex2->point());

	//Initialize the cross edge
	CMesh::tEdge feature_line = mesh_2D.createEdge(vertex1, vertex2);
	CMesh::tEdge feature_line_3D = mesh_3D.createEdge(vertex1_3D, vertex2_3D);
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
		v[1] = vertex1;
		v[2] = halfedge2->target();
		CMesh::tFace face1 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		//FaceInformation(face1);

		v[0] = vertex1;
		v[1] = halfedge1->target();
		v[2] = vertex2;
		CMesh::tFace face2 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		//FaceInformation(face2);

		v[0] = vertex2;
		v[1] = halfedge2->target();
		v[2] = vertex1;
		CMesh::tFace face3 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		//FaceInformation(face3);

		//Create new faces in 3D mesh
		v[0] = halfedge1_3D->source();
		v[1] = vertex1_3D;
		v[2] = halfedge2_3D->target();
		CMesh::tFace face1_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);
		//FaceInformation(face1_3D);

		v[0] = vertex1_3D;
		v[1] = halfedge1_3D->target();
		v[2] = vertex2_3D;
		CMesh::tFace face2_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);
		//FaceInformation(face2_3D);

		v[0] = vertex2_3D;
		v[1] = halfedge2_3D->target();
		v[2] = vertex1_3D;
		CMesh::tFace face3_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);
		//FaceInformation(face3_3D);

		if (halfedge1->edge()->string().find("sharp") != halfedge1->edge()->string().npos)
		{
			mesh_2D.vertexEdge(halfedge1->source(), vertex1)->string() = "sharp";
			mesh_2D.vertexEdge(halfedge1->target(), vertex1)->string() = "sharp";
		}
		if (halfedge1_3D->edge()->string().find("sharp") != halfedge1_3D->edge()->string().npos)
		{
			mesh_3D.vertexEdge(halfedge1_3D->source(), vertex1_3D)->string() = "sharp";
			mesh_3D.vertexEdge(halfedge1_3D->target(), vertex1_3D)->string() = "sharp";
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
		v[1] = vertex1;
		v[2] = vertex2;
		CMesh::tFace face1 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		//FaceInformation(face1);

		v[0] = vertex1;
		v[1] = halfedge1->target();
		v[2] = halfedge2->source();
		CMesh::tFace face2 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		//FaceInformation(face2);

		v[0] = vertex2;
		v[1] = vertex1;
		v[2] = halfedge2->source();
		CMesh::tFace face3 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);
		//FaceInformation(face3);

		v[0] = halfedge1_3D->source();
		v[1] = vertex1_3D;
		v[2] = vertex2_3D;
		CMesh::tFace face1_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);
		//FaceInformation(face1_3D);

		v[0] = vertex1_3D;
		v[1] = halfedge1_3D->target();
		v[2] = halfedge2_3D->source();
		CMesh::tFace face2_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);
		//FaceInformation(face2_3D);

		v[0] = vertex2_3D;
		v[1] = vertex1_3D;
		v[2] = halfedge2_3D->source();
		CMesh::tFace face3_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);
		//FaceInformation(face3_3D);


		if (halfedge1->edge()->string().find("sharp") != halfedge1->edge()->string().npos)
		{
			mesh_2D.vertexEdge(halfedge1->source(), vertex1)->string() = "sharp";
			mesh_2D.vertexEdge(halfedge1->target(), vertex1)->string() = "sharp";
		}
		if (halfedge1_3D->edge()->string().find("sharp") != halfedge1_3D->edge()->string().npos)
		{
			mesh_3D.vertexEdge(halfedge1_3D->source(), vertex1_3D)->string() = "sharp";
			mesh_3D.vertexEdge(halfedge1_3D->target(), vertex1_3D)->string() = "sharp";
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
		vertex2->boundary() = true;
		//std::cout << "boundary vertex" << std::endl;
		//std::cout << vertex2->id() << std::endl;
		CHalfEdge* he = vertex2->halfedge();
		while (he->he_sym() != NULL)
		{
			he = he->ccw_rotate_about_target();
		}
		vertex2->halfedge() = he;
	}

	if (halfedge1->edge()->boundary())
	{
		vertex1->boundary() = true;
		CHalfEdge* he = vertex1->halfedge();
		while (he->he_sym() != NULL)
		{
			he = he->ccw_rotate_about_target();
		}
		vertex1->halfedge() = he;
	}


	//std::cout << "**************************************************" << std::endl;
	return vertex2;
}

void create_featureline(CMesh& mesh_2D, double ftr, CHalfEdge* initial_halfbound, CMesh::tVertex& vertex1, const std::string& string)
{
	//由初始边开始向特征线方向查找

	CHalfEdge* halfedge = initial_halfbound;



	while (halfedge != NULL)
	{
		CHalfEdge* halfedge2 = halfedge->he_next();
		if (!ifcross(halfedge2, ftr, string))
		{
			halfedge2 = halfedge2->he_next();
			//std::cout<<"不相交"<<std::endl;
		}


		vertex1 = create_new_mesh(halfedge, halfedge2, ftr, mesh_2D, vertex1, string);
		std::cout << "halfedge: " << halfedge->source()->id() << " " << halfedge->target()->id() << std::endl;
		std::cout << "halfedge2: " << halfedge2->source()->id() << " " << halfedge2->target()->id() << std::endl;
		//delete(halfedge);
		halfedge = halfedge2->he_sym();
		//delete(halfedge2);
	}

}

//vertex1 a 2D-vertex
void create_featureline_3D(CMesh& mesh_2D, CMesh& mesh_3D, double ftr, CHalfEdge* initial_halfbound, CMesh::tVertex& vertex1, const std::string& string)
{
	//由初始边开始向特征线方向查找

	CHalfEdge* halfedge = initial_halfbound;



	while (halfedge != NULL)
	{
		//std::cout << "********************************************" << std::endl;
		CHalfEdge* halfedge2 = halfedge->he_next();
		CMesh::tVertex vertex2 = halfedge2->target();
		if ((string == "horizontal" && vertex2->point()[0] == ftr) || (string == "vertical" && vertex2->point()[1] == ftr))
		{
			CHalfEdge* cross_halfedge = halfedge;
			CHalfEdge* cross_halfedge_3D = mesh_3D.vertexHalfedge(mesh_3D.idVertex(cross_halfedge->source()->id()), mesh_3D.idVertex(cross_halfedge->target()->id()));
			CMesh::tVertex vertex1_3D = mesh_3D.idVertex(vertex1->id());
			//重构最后一个三角形
			CMesh::tVertex vertex2_3D = mesh_3D.idVertex(vertex2->id());
			CMesh::tEdge feature_line = mesh_2D.createEdge(vertex1, vertex2);
			CMesh::tEdge feature_line_3D = mesh_3D.createEdge(vertex1_3D, vertex2_3D);
			feature_line->string() = "sharp";
			feature_line_3D->string() = "sharp";

			//重构第一个面
			CVertex* v[3];
			v[0] = cross_halfedge->source();
			v[1] = vertex1;
			v[2] = vertex2;
			CMesh::tFace face1 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);

			v[0] = vertex1;
			v[1] = cross_halfedge->target();
			v[2] = vertex2;
			CMesh::tFace face2 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);

			v[0] = cross_halfedge_3D->source();
			v[1] = vertex1_3D;
			v[2] = vertex2_3D;
			CMesh::tFace face1_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);

			v[0] = vertex1_3D;
			v[1] = cross_halfedge_3D->target();
			v[2] = vertex2_3D;
			CMesh::tFace face2_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);

			mesh_2D.faces().remove(cross_halfedge->face());
			mesh_2D.map_face().erase(cross_halfedge->face()->id());
			mesh_3D.faces().remove(cross_halfedge_3D->face());
			mesh_3D.map_face().erase(cross_halfedge_3D->face()->id());

			
			extend_featureline_3D(vertex2, mesh_2D, mesh_3D, ftr, cross_halfedge->edge(), string);
			break;
		}
		else
		{
			if (!ifcross(halfedge2, ftr, string))
			{
				//std::cout << halfedge2->source()->id() << " " << halfedge2->target()->id() << std::endl;
				halfedge2 = halfedge2->he_next();

			}
			//std::cout << halfedge2->source()->id() << " " << halfedge2->target()->id() << std::endl;

			//assert(ifcross(halfedge2, ftr, string));
			//std::cout << vertex1->id() << std::endl;
			vertex1 = create_new_mesh_3D(halfedge, halfedge2, ftr, mesh_2D, mesh_3D, vertex1, string);
			//std::cout << "halfedge: " << halfedge->source()->id() << " " << halfedge->target()->id() << std::endl;
			//std::cout << "halfedge2: " << halfedge2->source()->id() << " " << halfedge2->target()->id() << std::endl;
			//delete(halfedge);
			halfedge = halfedge2->he_sym();
			//delete(halfedge2);
			//std::cout << "********************************************" << std::endl;
		}
		
	}

}

void extend_featureline(CVertex*& vertex, CMesh& mesh_2D, double ftr, CMesh::tEdge& edge, const std::string& string)
{
	std::cout << "******************************" << std::endl;
	std::cout << "extend_featureline" << std::endl;
	std::cout << "starting edge: " << edge->halfedge(0)->source()->id() << " " << edge->halfedge(0)->target()->id() << std::endl;
	std::cout << "starting vertex: " << vertex->id() << std::endl;
	if (!vertex->boundary())
	{
		//找到该顶点的所有面
		std::list<CMesh::tFace> face_list;
		for (VertexFaceIterator vfiter(vertex); !vfiter.end(); ++vfiter)
		{
			CMesh::tFace face = *vfiter;
			face_list.push_back(face);
		}
		//删去原有特征线所在面
		face_list.remove(edge->halfedge(0)->face());
		face_list.remove(edge->halfedge(1)->face());


		CHalfEdge* cross_halfedge;
		//找到与特征线相交的半边
		for (std::list<CMesh::tFace>::iterator it = face_list.begin(); it != face_list.end(); ++it)
		{
			CMesh::tFace face = *it;
			for (FaceHalfedgeIterator fhiter(face); !fhiter.end(); ++fhiter)
			{
				CHalfEdge* halfedge = *fhiter;
				if (ifcross(halfedge, ftr, string))
				{
					std::cout << "cross halfedge: " << halfedge->source()->id() << " " << halfedge->target()->id() << std::endl;
					cross_halfedge = halfedge;
					break;
				}
			}
		}
		assert(cross_halfedge != NULL);


		//初始化交点
		CMesh::tVertex vertex2 = mesh_2D.createVertex(mesh_2D.vertices().back()->id() + 1);
		vertex2->point() = crosspoint(cross_halfedge, ftr, string);
		CPoint vertex2_rgb = crosspoint_RGB(cross_halfedge, ftr, string);
		insert_RGB(vertex2, vertex2_rgb);


		//初始化第一条特征边
		CMesh::tEdge feature_line = mesh_2D.createEdge(vertex, vertex2);
		feature_line->string() = "sharp";

		//重构第一个面
		CVertex* v[3];
		v[0] = cross_halfedge->source();
		v[1] = vertex2;
		v[2] = vertex;
		CMesh::tFace face1 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);

		v[0] = vertex2;
		v[1] = cross_halfedge->target();
		v[2] = vertex;
		CMesh::tFace face2 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);

		mesh_2D.faces().remove(cross_halfedge->face());
		mesh_2D.map_face().erase(cross_halfedge->face()->id());


		if (cross_halfedge->edge()->boundary())
		{
			mesh_2D.edges().remove(cross_halfedge->edge());
			mesh_2D.map_edge().erase(CEdgeKey(cross_halfedge->source(), cross_halfedge->target()));
		}
		else
		{
			cross_halfedge = cross_halfedge->he_sym();
			create_featureline(mesh_2D, ftr, cross_halfedge, vertex2, string);
		}


	}
	std::cout << "******************************" << std::endl;
}
void extend_featureline_3D(CVertex*& vertex, CMesh& mesh_2D, CMesh& mesh_3D, double ftr, CMesh::tEdge& edge, const std::string& string)
{
	if (!vertex->boundary())
	{
		//排除特殊情况
		std::list<CMesh::tEdge> edge_list;
		for (VertexEdgeIterator veiter(vertex); !veiter.end(); ++veiter)
		{
			CMesh::tEdge edge = *veiter;
			edge_list.push_back(edge);
		}
		edge_list.remove(edge);
		for (std::list<CMesh::tEdge>::iterator it = edge_list.begin(); it != edge_list.end(); ++it)
		{
			CMesh::tEdge edge_temp = *it;
			CMesh::tVertex vertex_temp;
			if (edge_temp->halfedge(0)->source() == vertex)
				vertex_temp = edge_temp->halfedge(0)->target();
			else
				vertex_temp = edge_temp->halfedge(0)->source();
			if ((string == "horizontal" && vertex_temp->point()[0] == ftr) || (string == "vertical" && vertex_temp->point()[1] == ftr))
			{
				edge_temp->string() = "sharp";
				mesh_3D.vertexEdge(vertex, vertex_temp)->string() = "sharp";
				extend_featureline_3D(vertex_temp, mesh_2D, mesh_3D, ftr, edge_temp, string);
				return;
			}
		}




		//找到该顶点的所有面
		std::list<CMesh::tFace> face_list;
		for (VertexFaceIterator vfiter(vertex); !vfiter.end(); ++vfiter)
		{
			CMesh::tFace face = *vfiter;
			face_list.push_back(face);
		}
		//删去原有特征线所在面
		face_list.remove(edge->halfedge(0)->face());
		face_list.remove(edge->halfedge(1)->face());
		std::cout << vertex->id() << std::endl;
		std::cout<<edge->halfedge(0)->source()->id()<<" "<<edge->halfedge(0)->target()->id()<<std::endl;
		std::cout<<edge->halfedge(0)->source()->point()[0]<<" "<<edge->halfedge(0)->target()->point()[0]<<std::endl;
		std::cout<<edge->halfedge(0)->source()->point()[1]<<" "<<edge->halfedge(0)->target()->point()[1]<<std::endl;

		CHalfEdge* cross_halfedge;
		//找到与特征线相交的半边
		for (std::list<CMesh::tFace>::iterator it = face_list.begin(); it != face_list.end(); ++it)
		{
			CMesh::tFace face = *it;
			for (FaceHalfedgeIterator fhiter(face); !fhiter.end(); ++fhiter)
			{
				CHalfEdge* halfedge = *fhiter;
				std::cout << halfedge->source()->id() << " " << halfedge->target()->id() << std::endl;
				std::cout<<halfedge->source()->point()[0]<<" "<<halfedge->target()->point()[0]<<std::endl;
				std::cout<<halfedge->source()->point()[1]<<" "<<halfedge->target()->point()[1]<<std::endl;

				if (ifcross(halfedge, ftr, string))
				{
					std::cout << "cross halfedge: " << halfedge->source()->id() << " " << halfedge->target()->id() << std::endl;
					cross_halfedge = halfedge;
					break;
				}
			}
		}
		assert(cross_halfedge != NULL);


		//初始化交点
		CMesh::tVertex vertex2 = mesh_2D.createVertex(mesh_2D.vertices().back()->id() + 1);
		vertex2->point() = crosspoint(cross_halfedge, ftr, string);
		CPoint vertex2_rgb = crosspoint_RGB(cross_halfedge, ftr, string);
		insert_RGB(vertex2, vertex2_rgb);
		

		CHalfEdge* cross_halfedge_3D = mesh_3D.vertexHalfedge(mesh_3D.idVertex(cross_halfedge->source()->id()), mesh_3D.idVertex(cross_halfedge->target()->id()));
		CMesh::tVertex vertex1_3D = mesh_3D.idVertex(vertex->id());
		CMesh::tVertex vertex2_3D = mesh_3D.createVertex(mesh_3D.vertices().back()->id() + 1);
		vertex2_3D->point() = crosspoint_3D(mesh_3D, cross_halfedge, ftr, string);
		CPoint vertex2_3D_rgb = crosspoint_RGB_3D(mesh_3D, cross_halfedge, ftr, string);
		insert_RGB_3D(vertex2_3D, vertex2_3D_rgb, vertex2->point());

		
		//初始化第一条特征边
		CMesh::tEdge feature_line = mesh_2D.createEdge(vertex, vertex2);
		CMesh::tEdge feature_line_3D = mesh_3D.createEdge(vertex1_3D, vertex2_3D);
		feature_line->string() = "sharp";
		feature_line_3D->string() = "sharp";

		//重构第一个面
		CVertex* v[3];
		v[0] = cross_halfedge->source();
		v[1] = vertex2;
		v[2] = vertex;
		CMesh::tFace face1 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);

		v[0] = vertex2;
		v[1] = cross_halfedge->target();
		v[2] = vertex;
		CMesh::tFace face2 = mesh_2D.createFace(v, mesh_2D.faces().back()->id() + 1);

		v[0] = cross_halfedge_3D->source();
		v[1] = vertex2_3D;
		v[2] = vertex1_3D;
		CMesh::tFace face1_3D = mesh_3D.createFace(v, mesh_3D.faces().back()->id() + 1);

		v[0] = vertex2_3D;
		v[1] = cross_halfedge_3D->target();
		v[2] = vertex1_3D;
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

			vertex2->boundary() = true;

			CHalfEdge* he = vertex2->halfedge();
			while (he->he_sym() != NULL)
			{
				he = he->ccw_rotate_about_target();
			}
			vertex2->halfedge() = he;
		}
		else
		{
			cross_halfedge = cross_halfedge->he_sym();
			create_featureline_3D(mesh_2D, mesh_3D, ftr, cross_halfedge, vertex2, string);
		}


	}
	//std::cout << "******************************" << std::endl;
}

void find_next_featureline(CVertex*& vertex, std::list<CMesh::tEdge>& edge_list, CMesh::tEdge& edge, const std::string& string)
{//迭代到特征线末端，并储存遍历过的特征边
	if (vertex->boundary())
		return;
	//找到该顶点的所有边
	CEdge* edge_origin = NULL;
	std::list<CMesh::tEdge> edge_list_temp;//储存该顶点的符合条件的特征边
	//std::cout<<"******************************"<<std::endl;
	//std::cout<<"find_next_featureline"<<std::endl;
	//std::cout<<"string:"<<string<<std::endl;
	//std::cout<<"original vertex id:"<<vertex->id()<<std::endl;



	while (edge_origin != edge)
	{
		edge_origin = edge;
		//std::cout << "edge.id: " << edge->halfedge(0)->source()->id() << " " << edge->halfedge(0)->target()->id() << std::endl;
		VertexEdgeIterator veiter(vertex);
		bool find = false;
		while (!veiter.end())
		{
			CMesh::tEdge edge_iter = *veiter;
			//std::cout<<"edge_iter.id: "<<edge_iter->halfedge(0)->source()->id()<<" "<<edge_iter->halfedge(0)->target()->id()<<std::endl;
			if (edge_iter->string().find("sharp") != edge_iter->string().npos && judge(edge_iter, string) == true)
			{
				edge_list_temp.push_back(edge_iter);
				++veiter;
			}
			else
				++veiter;
		}
		//std::cout<<"end"<<std::endl;
		edge_list_temp.remove(edge);//删去原有的特征边，则表中只剩下新的特征边
		if (edge_list_temp.size() != 0)//如果有新的特征边，则将该边置为edge，并将该边的另一个顶点作为新的顶点
		{
			edge = *edge_list_temp.begin();
			CHalfEdge* halfedge = edge->halfedge(0);
			if (halfedge->target() == vertex)
				vertex = halfedge->source();
			else
				vertex = halfedge->target();
			edge_list_temp.clear();

		}
		edge_list.push_back(edge_origin);
		if (vertex->boundary())
			break;

	}
	
}

void draw_featureline(CMesh& mesh_2D, const std::string& string, double ftr)//完整划线
{
	//找到初始半边
	CHalfEdge* initial_halfbound;
	for (MeshEdgeIterator eiter(&mesh_2D); !eiter.end(); ++eiter)
	{
		CEdge* edge = *eiter;
		if (edge->boundary())
		{
			CHalfEdge* halfedge = edge->halfedge(0);
			if (halfedge == NULL)
				halfedge = edge->other(halfedge);
			if (ifcross(halfedge, ftr, string))
			{
				std::cout << "initial_halfbound: " << halfedge->source()->id() << " " << halfedge->target()->id() << std::endl;
				initial_halfbound = halfedge;
				break;
			}
		}
	}
	//初始化第一个交点
	CMesh::tVertex vertex1 = mesh_2D.createVertex(mesh_2D.vertices().back()->id() + 1);
	vertex1->point() = crosspoint(initial_halfbound, ftr, string);

	//为新顶点添加RGB值
	CPoint vertex1_rgb = crosspoint_RGB(initial_halfbound, ftr, string);
	insert_RGB(vertex1, vertex1_rgb);
	std::cout << "vertex1_rgb: " << vertex1_rgb[0] << " " << vertex1_rgb[1] << " " << vertex1_rgb[2] << std::endl;
	create_featureline(mesh_2D, ftr, initial_halfbound, vertex1, string);
}
void draw_featureline_3D(CMesh& mesh_2D, CMesh& mesh_3D, const std::string& string, double ftr)//完整划线
{
	std::cout << "New line***************************************************************************" << std::endl;
	//std::cout << string << std::endl;
	//找到初始半边
	CHalfEdge* initial_halfbound;
	for (MeshEdgeIterator eiter(&mesh_2D); !eiter.end(); ++eiter)
	{
		CEdge* edge = *eiter;
		if (edge->boundary())
		{
			CHalfEdge* halfedge = edge->halfedge(0);
			if (halfedge == NULL)
				halfedge = edge->other(halfedge);
			if (ifcross(halfedge, ftr, string))
			{
				//std::cout << "initial_halfbound: " << halfedge->source()->id() << " " << halfedge->target()->id() << std::endl;
				initial_halfbound = halfedge;
				break;
			}
		}
	}
	//初始化第一个交点
	assert(initial_halfbound != NULL);
	CMesh::tVertex vertex1 = mesh_2D.createVertex(mesh_2D.vertices().back()->id() + 1);
	vertex1->point() = crosspoint(initial_halfbound, ftr, string);




	CPoint vertex1_rgb = crosspoint_RGB(initial_halfbound, ftr, string);
	insert_RGB(vertex1, vertex1_rgb);
	//std::cout << "vertex1_rgb: " << vertex1_rgb[0] << " " << vertex1_rgb[1] << " " << vertex1_rgb[2] << std::endl;

	//Find crosspoint in 3D
	int id1 = initial_halfbound->source()->id();
	int id2 = initial_halfbound->target()->id();
	CHalfEdge* initial_halfbound_3D = mesh_3D.vertexHalfedge(mesh_3D.idVertex(id1), mesh_3D.idVertex(id2));
	CMesh::tVertex vertex1_3D = mesh_3D.createVertex(mesh_3D.vertices().back()->id() + 1);
	vertex1_3D->point() = crosspoint_3D(mesh_3D, initial_halfbound, ftr, string);

	CPoint vertex1_3D_rgb = crosspoint_RGB_3D(mesh_3D, initial_halfbound, ftr, string);
	insert_RGB_3D(vertex1_3D, vertex1_3D_rgb, vertex1->point());

	create_featureline_3D(mesh_2D, mesh_3D, ftr, initial_halfbound, vertex1, string);
}

bool corner_detection(CVertex*& vertex_2D, CMesh& mesh_2D, double max_u, double max_v, double min_u, double min_v)
{
	if (vertex_2D->boundary())
	{
		if (fabs(vertex_2D->point()[0] - max_u) < 0.0001 && fabs(vertex_2D->point()[1] - max_v) < 0.01)
			return true;
		else if (fabs(vertex_2D->point()[0] - max_u) < 0.0001 && fabs(vertex_2D->point()[1] - min_v) < 0.01)
			return true;
		else if (fabs(vertex_2D->point()[0] - min_u) < 0.0001 && fabs(vertex_2D->point()[1] - min_v) < 0.01)
			return true;
		else if (fabs(vertex_2D->point()[0] - min_u) < 0.0001 && fabs(vertex_2D->point()[1] - max_v) < 0.01)
			return true;
		else
			return false;
	}
}
void corner_purification(CVertex*& corner_2D, CMesh& mesh_2D, CMesh& mesh_3D)
{
	for (VertexEdgeIterator iter(corner_2D); !iter.end(); ++iter)
	{
		CMesh::tEdge e = *iter;
		if (e->string().find("sharp") != e->string().npos && judge(e, "vertical") == false && judge(e, "horizontal") == false)
		{
			e->string().clear();
			int id0 = e->halfedge(0)->source()->id();
			int id1 = e->halfedge(0)->target()->id();
			CVertex* v0 = mesh_2D.idVertex(id0);
			CVertex* v1 = mesh_2D.idVertex(id1);
			CMesh::tEdge edge = mesh_3D.vertexEdge(v0, v1);
			edge->string().clear();
			//std::cout<<id0<<" "<<id1<<std::endl;
		}

	}
}




//********************************************************************************************************************
//Generate standard quad mesh
void generate_quadmesh(CMesh& mesh_2D, CMesh& mesh_3D, std::list<double>& horizontal_feature, std::list<double>& vertical_feature)
{
	std::list<double> newline_u = PointSelection(horizontal_feature);

	std::list<double> newline_v = PointSelection(vertical_feature);

	std::cout << (horizontal_feature.size() + newline_u.size() - 1) * (vertical_feature.size() + newline_v.size()-1) << std::endl;

	

	for(std::list<double> ::iterator it=newline_u.begin();it!=newline_u.end();++it)
	{
		draw_featureline_3D(mesh_2D, mesh_3D, "horizontal", *it);
		horizontal_feature.push_back(*it);
	}

	for(std::list<double> ::iterator it=newline_v.begin();it!=newline_v.end();++it)
	{
		draw_featureline_3D(mesh_2D, mesh_3D, "vertical", *it);
		vertical_feature.push_back(*it);
	}

	horizontal_feature.sort();
	vertical_feature.sort();
}
