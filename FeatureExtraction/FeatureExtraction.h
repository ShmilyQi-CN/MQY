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
#include "..\FeatureComputation/FeatureComputation.h"


using namespace MeshLib;


bool getIntersectionPoint(CMesh::tVertex A_v, CMesh::tVertex B_v, CMesh::tVertex C_v, CMesh::tVertex D_v,
    CMesh::tVertex& intersection_v_2D, CMesh::tVertex& intersection_v_3D, CMesh& mesh_2D, CMesh& mesh_3D);


void UnitReconstruction(CHalfEdge* halfedge1, CHalfEdge* halfedge2, CMesh& mesh_2D, CMesh& mesh_3D, int id0, int id1);

bool parallel(CPoint A, CPoint B, CPoint C, CPoint D);
bool between(CMesh::tVertex vertex0, CMesh::tVertex vertex_temp, CMesh::tVertex vertex1);
void Link(CMesh::tVertex vertex0, CMesh::tVertex vertex1, CMesh& mesh_2D, CMesh& mesh_3D);


void Extraction(std::list<int> Feature_Point_id, CMesh& mesh_2D, CMesh& mesh_3D);

double getDegAngle3d(const Eigen::Vector3d v1, const Eigen::Vector3d v2);
CPoint vec(CMesh::tEdge edge, CMesh::tVertex vertex);
std::list<double> AngleList(CMesh& mesh_3D);