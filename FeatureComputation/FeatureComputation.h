#pragma once
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
//********************************************************************************
//Auxiliary function
void FaceInformation(const CMesh::tFace& face);
bool judge(const CMesh::tEdge& edge, const std::string& string);
bool check(double k, double j);
bool ifcross(CHalfEdge* halfedge, double k, const std::string& string);
std::list<double> PointSelection(std::list<double> list);
void findPreviousAndNext(std::vector<double> A, double a, double& prev, double& next);
double distance(CMesh::tVertex vertex0_2D, CMesh::tVertex vertex1_2D, CMesh& mesh_3D);
//********************************************************************************
CPoint crosspoint(CHalfEdge* halfedge, double k, const std::string& string);
CPoint crosspoint_3D(CMesh& mesh_3D, CHalfEdge* halfedge, double k, const std::string& string);

CPoint crosspoint_RGB(CHalfEdge* halfedge, double k, const std::string& string);
CPoint crosspoint_RGB_3D(CMesh& mesh_3D, CHalfEdge* halfedge, double k, const std::string& string);

void insert_RGB(CMesh::tVertex& vertex, CPoint RGB);
void insert_RGB_3D(CMesh::tVertex& vertex, CPoint RGB, CPoint point);


CMesh::tVertex create_new_mesh(CHalfEdge* halfedge1, CHalfEdge* halfedge2, double k, CMesh& mesh_2D, CMesh::tVertex& vertex1, const std::string& string);
CMesh::tVertex create_new_mesh_3D(CHalfEdge* halfedge1, CHalfEdge* halfedge2, double k, CMesh& mesh_2D, CMesh& mesh_3D, CMesh::tVertex& vertex1, const std::string& string);


void create_featureline(CMesh& mesh_2D, double ftr, CHalfEdge* initial_halfbound, CMesh::tVertex& vertex1, const std::string& string);
void create_featureline_3D(CMesh& mesh_2D, CMesh& mesh_3D, double ftr, CHalfEdge* initial_halfbound, CMesh::tVertex& vertex1, const std::string& string);

void extend_featureline(CVertex*& vertex, CMesh& mesh_2D, double ftr, CMesh::tEdge& edge, const std::string& string);
void extend_featureline_3D(CVertex*& vertex, CMesh& mesh_2D, CMesh& mesh_3D, double ftr, CMesh::tEdge& edge, const std::string& string);

void find_next_featureline(CVertex*& vertex, std::list<CMesh::tEdge>& edge_list, CMesh::tEdge& edge, const std::string& string);

void draw_featureline(CMesh& mesh_2D, const std::string& string, double ftr);
void draw_featureline_3D(CMesh& mesh_2D, CMesh& mesh_3D, const std::string& string, double ftr);

bool corner_detection(CVertex*& vertex_2D, CMesh& mesh_2D, double max_u, double max_v, double min_u, double min_v);
void corner_purification(CVertex*& corner_2D, CMesh& mesh_2D, CMesh& mesh_3D);

void generate_quadmesh(CMesh& mesh_2D, CMesh& mesh_3D, std::list<double>& horizontal_feature, std::list<double>& vertical_feature);
void generate_quadmesh_3D(CMesh& mesh_2D, CMesh& mesh_3D,
	std::list<CHalfEdge*> halfedge_list, std::list<CMesh::tVertex> feature_vertex);