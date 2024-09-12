/*******************************************************************************
*      Viewer - 3D triangle mesh viewer
*
*       Copyright (c) CCGL
*
*    Purpose:
*       Display 3D triangle meshes
*
*       David Gu June 27, 2008
*
*******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include<GL/glut.h>
#include <time.h>

#define _USE_MATH_DEFINES
#include <math.h>
#include "mesh/mesh.h"
#include "mesh/boundary.h"
#include "mesh/iterators.h"
#include "Trait/ViewerTrait.h"
#include "viewer/Arcball.h"                           /*  Arc Ball  Interface         */
#include<iostream>
#include <Eigen/Dense>
#include "MeshGeneration/Interpolation.h"
#include "FeatureComputation/FeatureComputation.h"
#include "FeatureExtraction/FeatureExtraction.h"
#include < fstream >

using namespace MeshLib;


/* window width and height */
int win_width, win_height;
int gButton;
int startx, starty;
int shadeFlag = 0;

/* rotation quaternion and translation vector for the object */
CQrot       ObjRot(0, 0, 1, 0);
CPoint      ObjTrans(0, 0, 0);

/* global mesh */
CMesh mesh_3D;
CMesh mesh_2D;

/* arcball object */
CArcball arcball;

int textureFlag = 2;

GLuint texName;
#define	checkImageWidth  512
#define	checkImageHeight 512

static GLubyte checkImage[checkImageHeight][checkImageWidth][3];

void set_check_image(int size)
{
	for (int i = 0; i < checkImageWidth; i++)
	{
		int r = i / size;
		for (int j = 0; j < checkImageHeight; j++)
		{
			int c = j / size;
			unsigned char v;
			v = ((r + c) % 2) ? 255 : 0;
			for (int k = 0; k < 3; k++)
			{
				checkImage[i][j][k] = v;
			}
		}
	}
}


/* setup the object, transform from the world to the object coordinate system */
void setupObject(void)
{
	double rot[16];

	glTranslated(ObjTrans[0], ObjTrans[1], ObjTrans[2]);
	ObjRot.convert(rot);
	glMultMatrixd((GLdouble*)rot);
}

/* the eye is always fixed at world z = +5 */

void setupEye(void) {
	glLoadIdentity();
	gluLookAt(0, 0, 5, 0, 0, 0, 0, 1, 0);
}

/* setup light */
void setupLight()
{
	CPoint position(0, 0, 1);
	GLfloat lightOnePosition[4] = { position[0], position[1], position[2], 0 };
	glLightfv(GL_LIGHT1, GL_POSITION, lightOnePosition);
}

void draw_mesh()
{
	glBindTexture(GL_TEXTURE_2D, texName);
	glBegin(GL_TRIANGLES);
	for (MeshFaceIterator fiter(&mesh_3D); !fiter.end(); ++fiter)
	{
		CFace* pf = *fiter;
		CHalfEdge* he = pf->halfedge();
		for (FaceVertexIterator fviter(pf); !fviter.end(); ++fviter)
		{
			CVertex* v = *fviter;
			CPoint pt = v->point();
			CPoint n;
			switch (shadeFlag)
			{
			case 0:
				n = f_normal(pf);
				break;
			case 1:
				n = v_normal(v);
				break;
			}
			CPoint2 uv = v_uv(v);
			glNormal3d(n[0], n[1], n[2]);
			glTexCoord2d(uv[0], uv[1]);
			//glColor3f( v_rgb(v)[0], v_rgb(v)[1], v_rgb(v)[2] );
			glVertex3d(pt[0], pt[1], pt[2]);
		}
	}
	glEnd();
}

void display()
{
	/* clear frame buffer */
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	setupLight();
	/* transform from the eye coordinate system to the world system */
	setupEye();
	glPushMatrix();
	/* transform from the world to the ojbect coordinate system */
	setupObject();
	/* draw the mesh */
	draw_mesh();
	glPopMatrix();
	glutSwapBuffers();
}

/* Called when a "resize" event is received by the window. */

void reshape(int w, int h)
{
	float ar;

	win_width = w;
	win_height = h;

	ar = (float)(w) / h;
	glViewport(0, 0, w, h);               /* Set Viewport */
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	// magic imageing commands
	gluPerspective(40.0, /* field of view in degrees */
		ar, /* aspect ratio */
		1.0, /* Z near */
		100.0 /* Z far */);

	glMatrixMode(GL_MODELVIEW);

	glutPostRedisplay();
}

void help()
{
	printf("w - Wireframe Display\n");
	printf("f  -  Flat Shading \n");
	printf("s  -  Smooth Shading\n");
	printf("t  -  Texture Mapping\n");
	printf("? -  Help Information\n");
	printf("esc - quit\n");
}

void keyBoard(unsigned char key, int x, int y)
{

	switch (key)
	{
	case 'f':
		//Flat Shading
		glPolygonMode(GL_FRONT, GL_FILL);
		shadeFlag = 0;
		break;
	case 's':
		//Smooth Shading
		glPolygonMode(GL_FRONT, GL_FILL);
		shadeFlag = 1;
		break;
	case 'w':
		//Wireframe mode
		glPolygonMode(GL_FRONT, GL_LINE);
		break;
	case 't':
		textureFlag = (textureFlag + 1) % 3;
		switch (textureFlag)
		{
		case 0:
			glDisable(GL_TEXTURE_2D);
			break;
		case 1:
			glEnable(GL_TEXTURE_2D);
			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
			break;
		case 2:
			glEnable(GL_TEXTURE_2D);
			glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
			break;
		}
		break;
	case '?':
		help();
		break;

	case 27:
		exit(0);
		break;
	}
	glutPostRedisplay();
}

void setupGLstate() {

	GLfloat lightOneColor[] = { 1, 1, 1, 1 };
	GLfloat globalAmb[] = { .1, .1, .1, 1 };
	GLfloat lightOnePosition[] = { .0,  .0, 1, 0.0 };

	glEnable(GL_CULL_FACE);
	glFrontFace(GL_CCW);
	glEnable(GL_DEPTH_TEST);
	glClearColor(0, 0, 0, 0);
	glShadeModel(GL_SMOOTH);


	glEnable(GL_LIGHT1);
	glEnable(GL_LIGHTING);
	glEnable(GL_NORMALIZE);
	glEnable(GL_COLOR_MATERIAL);

	glLightfv(GL_LIGHT1, GL_DIFFUSE, lightOneColor);
	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, globalAmb);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);

	glLightfv(GL_LIGHT1, GL_POSITION, lightOnePosition);
}

void  mouseClick(int button, int state, int x, int y) {


	/* set up an arcball around the Eye's center
		switch y coordinates to right handed system  */

	if (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN)
	{
		gButton = GLUT_LEFT_BUTTON;
		arcball = CArcball(win_width, win_height, x - win_width / 2, win_height - y - win_height / 2);
	}

	if (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN) {
		startx = x;
		starty = y;
		gButton = GLUT_MIDDLE_BUTTON;
	}

	if (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN) {
		startx = x;
		starty = y;
		gButton = GLUT_RIGHT_BUTTON;
	}
	return;
}

void mouseMove(int x, int y)
{
	CPoint trans;
	CQrot       rot;

	/* rotation, call arcball */
	if (gButton == GLUT_LEFT_BUTTON)
	{
		rot = arcball.update(x - win_width / 2, win_height - y - win_height / 2);
		ObjRot = rot * ObjRot;
		glutPostRedisplay();
	}

	/*xy translation */
	if (gButton == GLUT_MIDDLE_BUTTON)
	{
		double scale = 10. / win_height;
		trans = CPoint(scale * (x - startx), scale * (starty - y), 0);
		startx = x;
		starty = y;
		ObjTrans = ObjTrans + trans;
		glutPostRedisplay();
	}

	/* zoom in and out */
	if (gButton == GLUT_RIGHT_BUTTON) {
		double scale = 10. / win_height;
		trans = CPoint(0, 0, scale * (starty - y));
		startx = x;
		starty = y;
		ObjTrans = ObjTrans + trans;
		glutPostRedisplay();
	}

}

void normalize_mesh(CMesh* pMesh)
{
	CPoint s(0, 0, 0);
	for (MeshVertexIterator viter(pMesh); !viter.end(); ++viter)
	{
		CVertex* v = *viter;
		s = s + v->point();
	}

	s = s / pMesh->numVertices();
	for (MeshVertexIterator viter(pMesh); !viter.end(); ++viter)
	{
		CVertex* v = *viter;
		CPoint p = v->point();
		p = p - s;
		v->point() = p;
	}
	double d = 0;

	for (MeshVertexIterator viter(pMesh); !viter.end(); ++viter)
	{
		CVertex* v = *viter;
		CPoint p = v->point();
		for (int k = 0; k < 3; k++)
		{
			d = (d > fabs(p[k])) ? d : fabs(p[k]);
		}
	}

	for (MeshVertexIterator viter(pMesh); !viter.end(); ++viter)
	{
		CVertex* v = *viter;
		CPoint p = v->point();
		p = p / d;
		v->point() = p;
	}
}

void compute_normal(CMesh* pMesh)
{
	for (MeshFaceIterator fiter(pMesh); !fiter.end(); ++fiter)
	{
		CFace* pF = *fiter;
		CPoint p[3];
		CHalfEdge* he = pF->halfedge();
		for (int k = 0; k < 3; k++)
		{
			p[k] = he->target()->point();
			he = he->he_next();
		}

		CPoint n = (p[1] - p[0]) ^ (p[2] - p[0]);
		f_area(pF) = n.norm();
		f_normal(pF) = n / n.norm();
	}

	for (MeshVertexIterator viter(pMesh); !viter.end(); ++viter)
	{
		CVertex* v = *viter;
		CPoint n(0, 0, 0);
		for (VertexFaceIterator vfiter(v); !vfiter.end(); ++vfiter)
		{
			CFace* pF = *vfiter;
			n += f_normal(pF) * f_area(pF);
		}
		n = n / n.norm();
		v_normal(v) = n;
	}

}

void initialize_texture()
{
	glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
	glGenTextures(1, &texName);
	glBindTexture(GL_TEXTURE_2D, texName);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	//glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,   GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);

	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
		checkImageWidth,
		checkImageHeight,
		0,
		GL_RGB,
		GL_UNSIGNED_BYTE,
		checkImage);

	if (textureFlag == 1)
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
	else if (textureFlag == 2)
		glTexEnvf(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
	glEnable(GL_TEXTURE_2D);
}

void load_uv(CMesh& mesh_3D)
{
	for (MeshVertexIterator viter(&mesh_3D); !viter.end(); ++viter)
	{
		CVertex* v = *viter;
		CVertexTrait* pT = (CVertexTrait*)v->trait();
		v_string(v) = v->string();
		pT->read();
	}
}

//Obtain the 2D mesh from the 3D mesh
CMesh& getMesh(CMesh& mesh_2D,CMesh& mesh_3D)
{
	//std::cout << "**************************************************" << std::endl;
	//std::cout << "extracting 2DMesh" << std::endl;

	for (MeshVertexIterator viter(&mesh_3D); !viter.end(); ++viter)
	{
		CVertex* v = *viter;
		CVertex* v_new = mesh_2D.createVertex(v->id());
		CVertexTrait* pT = (CVertexTrait*)v->trait();
		CPoint p;
		p[0] = pT->m_uv[0];
		p[1] = pT->m_uv[1];
		p[2] = 0;
		v_new->point() = p;
		//v_new->string() = v->string();
		CPoint RGB = v_rgb(v);
		CVertexTrait* pT_new = new CVertexTrait;
		v_new->trait() = (CTrait*)pT_new;
		v_rgb(v_new) = RGB;
		v_new->string() = "rgb=(" + std::to_string(RGB[0]) + " " + std::to_string(RGB[1]) + " " + std::to_string(RGB[2]) + ")";
		//std::cout<<"vertex "<<v_new->id()<<" "<<v_new->point()[0]<<v_new->point()[1]<<" "<<v_new->string()<<std::endl;
	}
	//std::cout << "vertex generated" << std::endl;

	for (MeshFaceIterator fiter(&mesh_3D); !fiter.end(); ++fiter)
	{
		CFace* f = *fiter;
		CMesh::tVertex v[3];
		int i = 0;
		for (FaceVertexIterator fviter(f); !fviter.end(); ++fviter)
		{
			CVertex* v_temp = *fviter;
			v[i] = mesh_2D.idVertex(v_temp->id());
			++i;
		}
		CMesh::tFace f_new = mesh_2D.createFace(v, f->id());
		f_new->string() = f->string();
		/*std::cout<<"face "<<f_new->id()<<" "<<v[0]->id()<<" "<<v[1]->id()<<" "<<v[2]->id()
			<<" "<<f_new->string()<<std::endl;*/
	}
	//std::cout << "face generated" << std::endl;


	for (MeshEdgeIterator eiter(&mesh_3D); !eiter.end(); ++eiter)
	{
		CEdge* e = *eiter;
		int id0 = e->halfedge(0)->source()->id();
		int id1 = e->halfedge(0)->target()->id();
		CVertex* v0 = mesh_2D.idVertex(id0);
		CVertex* v1 = mesh_2D.idVertex(id1);
		CMesh::tEdge edge = mesh_2D.vertexEdge(v0, v1);
		edge->string() = e->string();
		//std::cout<<"edge "<<id0<<" "<<id1 << " " << edge->string() << std::endl;
		CMesh::tHalfEdge he[2];

		he[0] = edge->halfedge(0);
		he[1] = edge->halfedge(1);

		assert(he[0] != NULL);


		if (he[1] != NULL)
		{
			assert(he[0]->target() == he[1]->source() && he[0]->source() == he[1]->target());

			if (he[0]->target()->id() < he[0]->source()->id())
			{
				edge->halfedge(0) = he[1];
				edge->halfedge(1) = he[0];
			}

			assert(mesh_2D.edgeVertex1(edge)->id() < mesh_2D.edgeVertex2(edge)->id());
		}
		else
		{
			he[0]->vertex()->boundary() = true;
			he[0]->he_prev()->vertex()->boundary() = true;
		}
	}
	//std::cout << "edge labeled" << std::endl;


	std::list<CVertex*> dangling_verts;
	//Label boundary edges
	for (std::list<CVertex*>::iterator viter = mesh_2D.vertices().begin(); viter != mesh_2D.vertices().end(); ++viter)
	{
		CMesh::tVertex     v = *viter;
		if (v->halfedge() != NULL) continue;
		dangling_verts.push_back(v);
	}

	for (std::list<CVertex*>::iterator viter = dangling_verts.begin(); viter != dangling_verts.end(); ++viter)
	{
		CMesh::tVertex v = *viter;
		mesh_2D.vertices().remove(v);
		delete v;
		v = NULL;
	}

	//Arrange the boundary half_edge of boundary vertices, to make its halfedge
	//to be the most ccw in half_edge

	for (std::list<CVertex*>::iterator viter = mesh_2D.vertices().begin(); viter != mesh_2D.vertices().end(); ++viter)
	{
		CMesh::tVertex     v = *viter;
		if (!v->boundary()) continue;

		CHalfEdge* he = v->halfedge();
		while (he->he_sym() != NULL)
		{
			he = he->ccw_rotate_about_target();
		}
		v->halfedge() = he;
	}


	//std::cout << "Done" << std::endl;
	//std::cout << "**************************************************" << std::endl;
	return mesh_2D;

}




//********************************************************************************************************************
//Calculate the area of the surface corresponding to the rectangle parameter domain
double areaCalculation(double u[1], double v[1], CMesh& mesh_2D, CMesh& mesh_3D)
{
	std::list<CMesh::tVertex> vert;
	std::list<int> face_id;
	for (MeshVertexIterator viter(&mesh_2D); !viter.end(); ++viter)
	{
		CVertex* vertex = *viter;
		if (
			vertex->point()[0] > u[0] &&
			vertex->point()[0] < u[1] &&
			vertex->point()[1] > v[0] &&
			vertex->point()[1] < v[1]
			)
		{
			vert.push_back(vertex);
		}
	}
	assert(vert.size() != 0);
	//std::cout<<"vert.size: "<<vert.size()<<std::endl;
	for (std::list<CMesh::tVertex>::iterator it = vert.begin(); it != vert.end(); ++it)
	{
		CVertex* vertex = *it;
		//std::cout<<"id"<<vertex->id()<<std::endl;
		for (VertexFaceIterator vfiter(vertex); !vfiter.end(); ++vfiter)
		{
			CFace* face = *vfiter;
			face_id.push_back(face->id());
		}
	}
	face_id.unique();
	double area = 0;
	for (std::list<int>::iterator it = face_id.begin(); it != face_id.end(); ++it)
	{
		CFace* face = mesh_3D.idFace(*it);
		double vec1[3], vec2[3];
		vec1[0] = face->halfedge()->target()->point()[0] - face->halfedge()->source()->point()[0];
		vec1[1] = face->halfedge()->target()->point()[1] - face->halfedge()->source()->point()[1];
		vec1[2] = face->halfedge()->target()->point()[2] - face->halfedge()->source()->point()[2];
		vec2[0] = face->halfedge()->he_next()->target()->point()[0] - face->halfedge()->he_next()->source()->point()[0];
		vec2[1] = face->halfedge()->he_next()->target()->point()[1] - face->halfedge()->he_next()->source()->point()[1];
		vec2[2] = face->halfedge()->he_next()->target()->point()[2] - face->halfedge()->he_next()->source()->point()[2];
		Eigen::Vector3d v1(vec1[0], vec1[1], vec1[2]);
		Eigen::Vector3d v2(vec2[0], vec2[1], vec2[2]);
		area += (v1.cross(v2)).norm();
	}
	return area;
}



///////////////////////////////////////////////////////////////////////////////////////
int main(int argc, char* argv[])
{
	clock_t start = clock();
	std::string path = "E:\\M.Q.Y\\课题\\实验六：Colon\\";
	std::string write_path = "E:\\M.Q.Y\\课题\\实验六：Colon\\";
	std::string name_origin = "colon_uvtest.m";
	
	std::string name_sharp = "mesh_2D_origin.m";
	std::string name_extension_3D = "mesh_3D_Extension.m";
	std::string name_extension_2D = "mesh_2D_Extension.m";
	std::string name_write_2D = "mesh_2D.m";
	std::string name_write_3D = "mesh_3D.m";
	std::string name_opt_2D = "mesh_AreaOptimization_2D.m";
	std::string name_opt_3D = "mesh_AreaOptimization_3D.m";

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Part 1: Read mesh and feature
	//std::cout << "Part 1: Read mesh and feature" << std::endl;
	//mesh.read_m( argv[1] );
	
	mesh_3D.read_m((path+name_origin).c_str());
	//mesh_3D.read_m((write_path + name_write_3D).c_str());

	//读取特征
	CViewerTrait vtrait(&mesh_3D);
	//normalize_mesh(&mesh);
	//compute_normal(&mesh);
	load_uv(mesh_3D);


	//Get angle list
	
	/*std::list<double> angle = AngleList(mesh_3D);
	
	std::ofstream dataFile;
	dataFile.open("E:\\M.Q.Y\\课题\\实验五：Evolution_tower\\test1\\Data.txt", std::ofstream::app);
	for (auto it = angle.begin(); it != angle.end(); ++it)
	{
		if (90 <= *it && *it < 180)
			*it = 180 - *it;
		else if (180 < *it && *it < 270)
			*it -= 180;
		else if (270 < *it)
			*it = 360 - *it;
		dataFile << *it << std::endl;
	}

	dataFile.close();*/


	mesh_2D = getMesh(mesh_2D, mesh_3D);
	//std::cout<<mesh_2D.edges().size()<<std::endl;
	//CViewerTrait vtrait(&mesh_2D);

	



	//读取参数域坐标极值
	double max_u = 0;
	double max_v = 0;
	double min_u = 0;
	double min_v = 0;
	for (MeshVertexIterator viter(&mesh_2D); !viter.end(); ++viter)
	{
		CVertex* vertex = *viter;
		CPoint point = vertex->point();
		if (point[0] > max_u)
			max_u = point[0];
		if (point[0] < min_u)
			min_u = point[0];
		if (point[1] > max_v)
			max_v = point[1];
		if (point[1] < min_v)
			min_v = point[1];
	}

	//删去角点判定sharp
	//for (MeshVertexIterator viter(&mesh_2D); !viter.end(); ++viter)
	//{
	//	CVertex* vertex = *viter;
	//	if (corner_detection(vertex, mesh_2D, max_u, max_v, min_u, min_v) == true)
	//	{
	//		//std::cout << vertex->id() << std::endl;
	//		corner_purification(vertex, mesh_2D, mesh_3D);
	//	}
	//}
	//mesh_2D.write_m((write_path+name_sharp).c_str());

	//连接散点算法
	//std::list<int> feature_points_1 = { 14174,2897,12661,6766,717,1169,10711,4293,9179,727,1708,13343,12719,11769,11852,10020,9110,1711,12722,3557,14978 };
	////{2940, 6599, 7111, 13040, 8770, 528, 1169, 4293, 5842, 5911, 3636, 9669};
	//std::list<int> feature_points_2 = { 14174,2897,12661,6766,717,1169,10711,4293,9179,727,1708,13343,12719,11769,11852,10020,9110,1711,12722,3557,14978 };
	////{ 10336, 9645, 12227, 14348, 14772, 3712, 4725, 3389, 3650 };

	////std::list<int> feature_points_3 = { 7699, 5312, 3167, 13955, 10688, 4839, 7753, 1983, 13783 };
	//Extraction(feature_points_1, mesh_2D, mesh_3D);
	//Extraction(feature_points_2, mesh_2D, mesh_3D);
	////Extraction(feature_points_3, mesh_2D, mesh_3D);
	//mesh_3D.write_m((path + name_extension_3D).c_str());
	//mesh_2D.write_m((path + name_extension_3D).c_str());


	std::list<double> horizontal_feature;
	std::list<double> vertical_feature;
	horizontal_feature.push_back(min_u);
	horizontal_feature.push_back(max_u);
	vertical_feature.push_back(min_v);
	vertical_feature.push_back(max_v);



	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Part 2: Extend feature line
	std::cout << "Part 2: Extend feature line" << std::endl;

	//识别原有的特征线，储存在list里
	std::list<CMesh::tEdge> feature_edge_horizontal;
	std::list<CMesh::tEdge> feature_edge_vertical;
	for (MeshEdgeIterator eiter(&mesh_2D); !eiter.end(); ++eiter)
	{
		CEdge* edge = *eiter;

		if (edge->string().find("sharp") != edge->string().npos && !edge->boundary())
		{
			if (judge(edge, "horizontal"))
			{
				feature_edge_horizontal.push_back(edge);
			}
				
			if (judge(edge, "vertical"))
			{
				feature_edge_vertical.push_back(edge);

			}
		}

	}
	feature_edge_horizontal.unique();
	feature_edge_vertical.unique();
	std::list<CMesh::tVertex> boundary_feature_vertex;




	//延伸特征线
	std::cout << "Dealing with vertical featureline*******************************" << std::endl;
	while (feature_edge_vertical.size() != 0)
	{
		CEdge* edge = *feature_edge_vertical.begin();
		CHalfEdge* halfedge = edge->halfedge(0);

		//将v值存入list
		vertical_feature.push_back(halfedge->source()->point()[1]);


		std::list<CMesh::tEdge> edge_list;//储存已经找到的特征边

		CVertex* vertex1 = halfedge->source();
		CVertex* vertex2 = halfedge->target();


		CEdge* edge_temp = edge;
		
		find_next_featureline(vertex1, edge_list, edge_temp, "vertical");

		//使特征线边界点成为格点
		if (!vertex1->boundary())
		{
			//horizontal_feature.push_back(vertex1->point()[0]);
			//draw_featureline_3D(mesh_2D, mesh_3D, "horizontal", vertex1->point()[0]);
		}
		extend_featureline_3D(vertex1, mesh_2D, mesh_3D, vertex1->point()[1], edge_temp, "vertical");

		//find_next_featureline(vertex1, edge_list, edge_temp, "vertical");//法一使用，再查找一次，找到新边界点
		//boundary_feature_vertex.push_back(vertex1);

		edge_temp = edge;

		find_next_featureline(vertex2, edge_list, edge_temp, "vertical");


		if (!vertex2->boundary())
		{
			//horizontal_feature.push_back(vertex2->point()[0]);
			//draw_featureline_3D(mesh_2D, mesh_3D, "horizontal", vertex2->point()[0]);
		}

		extend_featureline_3D(vertex2, mesh_2D, mesh_3D, vertex2->point()[1], edge_temp, "vertical");
		
		//find_next_featureline(vertex2, edge_list, edge_temp, "vertical");
		//boundary_feature_vertex.push_back(vertex2);

		edge_list.unique();

		for (std::list<CMesh::tEdge>::iterator it = edge_list.begin(); it != edge_list.end(); ++it)
		{
			feature_edge_vertical.remove(*it);
		}
	}
	std::cout << "Dealing with horizontal featureline*******************************" << std::endl;
	while (feature_edge_horizontal.size() != 0)
	{
		CEdge* edge = *feature_edge_horizontal.begin();
		CHalfEdge* halfedge = edge->halfedge(0);

		//将u值存入list
		horizontal_feature.push_back(halfedge->source()->point()[0]);

		std::list<CMesh::tEdge> edge_list;//储存已经找到的特征边

		CVertex* vertex1 = halfedge->source();
		CVertex* vertex2 = halfedge->target();


		CEdge* edge_temp = edge;//用以指向当前边
		find_next_featureline(vertex1, edge_list, edge_temp, "horizontal");
		
		if (!vertex1->boundary())
		{
			//vertical_feature.push_back(vertex1->point()[1]);
			//draw_featureline_3D(mesh_2D, mesh_3D, "vertical", vertex1->point()[1]);
		}
		extend_featureline_3D(vertex1, mesh_2D, mesh_3D, vertex1->point()[0], edge_temp, "horizontal");
		
		
		//find_next_featureline(vertex1, edge_list, edge_temp, "horizontal");//再查找一次，找到新边界点
		//boundary_feature_vertex.push_back(vertex1);

		edge_temp = edge;

		find_next_featureline(vertex2, edge_list, edge_temp, "horizontal");
		
		
		if (!vertex2->boundary())
		{
			//vertical_feature.push_back(vertex2->point()[1]);
			//draw_featureline_3D(mesh_2D, mesh_3D, "vertical", vertex2->point()[1]);
		}
		extend_featureline_3D(vertex2, mesh_2D, mesh_3D, vertex2->point()[0], edge_temp, "horizontal");
		
		
		//find_next_featureline(vertex2, edge_list, edge_temp, "horizontal");
		//boundary_feature_vertex.push_back(vertex2);

		edge_list.unique();

		for (std::list<CMesh::tEdge>::iterator it = edge_list.begin(); it != edge_list.end(); ++it)
		{
			feature_edge_horizontal.remove(*it);
		}
	}

	boundary_feature_vertex.unique();
	//std::cout<<boundary_feature_vertex.size()<<std::endl;




	//std::cout << "Done" << std::endl;

	//mesh_3D.write_m((write_path + name_extension_3D).c_str());
	//mesh_2D.write_m((write_path + name_extension_2D).c_str());

	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Part 3: Generate standard quad mesh    
	std::cout << "Part 3: Generate standard quad mesh" << std::endl;

	//法一：根据边界三维距离生成网格
	/*CBoundary mesh_boundary(&mesh_2D);
	CLoop* mesh_loops = *mesh_boundary.loops().begin();
	std::list<CHalfEdge*> mesh_boundary_he = mesh_loops->halfedges();
	std::set<CHalfEdge*> vc_corner;
	for (auto it = mesh_boundary_he.begin(); it != mesh_boundary_he.end(); ++it) {
		CHalfEdge* he = *it;
		CHalfEdge* he_next;

		if (he == *std::prev(mesh_boundary_he.end())) {
			he_next = *mesh_boundary_he.begin();
		}
		else he_next = *std::next(it);

		double product = std::abs(((he->target()->point() - he->source()->point()) ^
			(he_next->target()->point() - he_next->source()->point()))[2]);
		if (product >0.0001) {
			vc_corner.insert(he_next);
		}
	}
	std::list<CHalfEdge*>mesh_boundary_he_new = mesh_boundary_he;
	std::copy(mesh_boundary_he.begin(), mesh_boundary_he.end(), std::back_inserter(mesh_boundary_he_new));
	std::list<std::list<CHalfEdge*>>mesh_four_boundaries;
	bool start = false;
	for (auto it = mesh_boundary_he_new.begin(); it != mesh_boundary_he_new.end();) {
		if (vc_corner.find(*it) != vc_corner.end()) {
			std::list<CHalfEdge*> l;
			l.push_back((*it));
			it++;
			while (vc_corner.find(*it) == vc_corner.end()) {
				l.push_back((*it));
				it++;
			}
			mesh_four_boundaries.push_back(l);
			if (mesh_four_boundaries.size() == 4) {
				break;
			}
		}
		else {
			it++;
		}

	}

	std::list<std::list<CHalfEdge*>>::iterator it = mesh_four_boundaries.begin();*/

	/*generate_quadmesh_3D(mesh_2D, mesh_3D, *it, boundary_feature_vertex);
	it++;
	generate_quadmesh_3D(mesh_2D, mesh_3D, *it, boundary_feature_vertex);

	mesh_3D.write_m((write_path + name_write_3D).c_str());
	mesh_2D.write_m((write_path + name_write_2D).c_str());*/


	//法二：根据二维均匀选点生成网格
	horizontal_feature.sort();
	vertical_feature.sort();
	horizontal_feature.unique();
	vertical_feature.unique();
	//打印特征坐标
	std::cout << "horizontal feature list: " << std::endl;
	std::copy(horizontal_feature.cbegin(), horizontal_feature.cend(), std::ostream_iterator<double>(std::cout, " "));
	std::cout<<std::endl;

	std::cout << "vertical feature list: " << std::endl;
	std::copy(vertical_feature.cbegin(), vertical_feature.cend(), std::ostream_iterator<double>(std::cout, " "));
	std::cout << std::endl;

	std::list<double> u_feature;
	std::list<double> v_feature;
	std::copy(horizontal_feature.begin(), horizontal_feature.end(), std::back_inserter(u_feature));
	std::copy(vertical_feature.begin(), vertical_feature.end(), std::back_inserter(v_feature));
	//Preparitions for the area preserving algorithm

	generate_quadmesh(mesh_2D, mesh_3D, horizontal_feature, vertical_feature);

	//Examination
	std::cout << "u grid list: " << std::endl;
	std::copy(horizontal_feature.begin(), horizontal_feature.end(), std::ostream_iterator<double>(std::cout, " "));
	std::cout << std::endl;
	std::cout << "v grid list" << std::endl;
	std::copy(vertical_feature.begin(), vertical_feature.end(), std::ostream_iterator<double>(std::cout, " "));
	std::cout << std::endl;

	clock_t end = clock();
	std::cout << (double)(end - start) / CLOCKS_PER_SEC;

	mesh_3D.write_m((write_path + name_write_3D).c_str());
	mesh_2D.write_m((write_path + name_write_2D).c_str());




	









	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
	// Part 4: Excute area based optimization algorithm
	std::cout << "Part 4: Excute area based optimization algorithm" << std::endl;


	CMesh mesh_AreaEqualization_3D;
	//mesh_AreaEqualization_3D.read_m("E:\\M.Q.Y\\课题\\test1\\colon_3D.m");
	mesh_AreaEqualization_3D.read_m((path + name_origin).c_str());
	CViewerTrait vtraitp(&mesh_AreaEqualization_3D);
	load_uv(mesh_AreaEqualization_3D);
	CMesh mesh_AreaEqualization_2D;
	mesh_AreaEqualization_2D = getMesh(mesh_AreaEqualization_2D, mesh_AreaEqualization_3D);

	std::list<double> u_feature_area;
	std::list<double> v_feature_area;

	std::vector<double> u_GridLine;
	std::vector<double> v_GridLine;
	std::copy(horizontal_feature.begin(), horizontal_feature.end(), std::back_inserter(u_GridLine));
	std::copy(vertical_feature.begin(), vertical_feature.end(), std::back_inserter(v_GridLine));
	double u_bound[2] = { 0 }, v_bound[2] = { 0 };


	double para_boundary_u[2] = { *u_feature.begin(), *u_feature.rbegin() };
	double para_boundary_v[2] = { *v_feature.begin(), *v_feature.rbegin() };

	//2维面积离散阵
	std::vector<double>::iterator it_v = v_GridLine.begin();
	std::vector<double>::iterator it_u = u_GridLine.begin();

	std::list<CPoint> area_array;
	u_bound[0] = *it_u;
	it_u++;
	for (; it_u != u_GridLine.end(); it_u++)
	{
		u_bound[1] = *it_u;
		v_bound[0] = *it_v;
		it_v++;
		for (; it_v != v_GridLine.end(); it_v++)
		{
			v_bound[1] = *it_v;
			area_array.push_back(CPoint((u_bound[0]+u_bound[1])/2, (v_bound[0]+v_bound[1])/2, areaCalculation(u_bound, v_bound, mesh_2D, mesh_3D)));
			v_bound[0] = v_bound[1];
		}
		u_bound[0] = u_bound[1];
		it_v = v_GridLine.begin();
	}


	std::ofstream Area_matrix;
	Area_matrix.open("E:\\M.Q.Y\\课题\\实验六：Colon\\Area_matrix.txt");
	for (std::list<CPoint>::iterator it = area_array.begin(); it != area_array.end(); it++)
	{	
		Area_matrix << (*it)[0] << " " << (*it)[1] << " " << (*it)[2] << std::endl;
	}

	// Part 4.1: Deal with vertical grid line
	std::list<double> area_u;
	std::list<double> inter_cor_v;


	it_v = v_GridLine.begin();
	v_bound[0] = *it_v;
	it_v++;
	for (; it_v != v_GridLine.end(); it_v++)
	{
		v_bound[1] = *it_v;
		//std::cout << "bound" << std::endl;
		//std::cout << u_bound[0] << " " << u_bound[1] << " " << v_bound[0] << " " << v_bound[1] << std::endl;
		area_u.push_back(areaCalculation(para_boundary_u, v_bound, mesh_2D, mesh_3D));
		inter_cor_v.push_back((v_bound[0] + v_bound[1]) / 2);
		v_bound[0] = v_bound[1];
	}

	Normalize(area_u);
	area_u.push_front(0);
	area_u.push_back(1);
	inter_cor_v.push_front(para_boundary_v[0]);
	inter_cor_v.push_back(para_boundary_v[1]);


	std::cout << "**********************************************" << std::endl;
	std::cout << "Area" << std::endl;
	std::copy(area_u.begin(), area_u.end(), std::ostream_iterator<double>(std::cout, " "));
	std::cout << std::endl;
	std::cout << "Inter_cor" << std::endl;
	std::copy(inter_cor_v.begin(), inter_cor_v.end(), std::ostream_iterator<double>(std::cout, " "));
	std::cout << std::endl;

	std::cout << "v_feature_area" << std::endl;
	spline horizontal_density(inter_cor_v, area_u);
	horizontal_density.computeCubicSpline();
	for (std::list<double>::iterator it = v_feature.begin(); it != v_feature.end(); it++)
	{
		double k = horizontal_density.evaluateSpline(*it);
		v_feature_area.push_back(k);
		std::cout << k << " ";
	}
	std::cout << std::endl;

	std::cout << "v_new_gridlines" << std::endl;
	std::list<double> v_new_gridlines_area = PointSelection(v_feature_area);
	for (std::list<double>::iterator it = v_new_gridlines_area.begin(); it != v_new_gridlines_area.end(); ++it)
	{
		double k = horizontal_density.find_x_for_y(*it);
		draw_featureline_3D(mesh_AreaEqualization_2D, mesh_AreaEqualization_3D, "vertical", k);
	}

	// Part 4.2: Deal with horizontal grid line
	std::list<double> area_v;
	std::list<double> inter_cor_u;


	it_u = u_GridLine.begin();
	u_bound[0] = *it_u;
	it_u++;
	for (; it_u != u_GridLine.end(); it_u++)
	{
		u_bound[1] = *it_u;
		//std::cout << "bound" << std::endl;
		//std::cout << u_bound[0] << " " << u_bound[1] << " " << v_bound[0] << " " << v_bound[1] << std::endl;
		area_v.push_back(areaCalculation(u_bound, para_boundary_v, mesh_2D, mesh_3D));
		inter_cor_u.push_back((u_bound[0] + u_bound[1]) / 2);
		u_bound[0] = u_bound[1];
	}


	Normalize(area_v);
	area_v.push_front(0);
	area_v.push_back(1);
	inter_cor_u.push_front(para_boundary_u[0]);
	inter_cor_u.push_back(para_boundary_u[1]);


	std::cout << "**********************************************" << std::endl;
	std::cout << "Area" << std::endl;
	std::copy(area_v.begin(), area_v.end(), std::ostream_iterator<double>(std::cout, " "));
	std::cout << std::endl;
	std::cout << "Inter_cor" << std::endl;
	std::copy(inter_cor_v.begin(), inter_cor_v.end(), std::ostream_iterator<double>(std::cout, " "));
	std::cout << std::endl;

	std::cout << "u_feature_area" << std::endl;
	spline vertical_density(inter_cor_u, area_v);
	vertical_density.computeCubicSpline();
	for (std::list<double>::iterator it = u_feature.begin(); it != u_feature.end(); it++)
	{
		double k = vertical_density.evaluateSpline(*it);
		u_feature_area.push_back(k);
		std::cout << k << " ";
	}
	std::cout << std::endl;

	std::cout << "u_new_gridlines" << std::endl;
	std::list<double> u_new_gridlines_area = PointSelection(u_feature_area);
	for (std::list<double>::iterator it = u_new_gridlines_area.begin(); it != u_new_gridlines_area.end(); ++it)
	{
		double k = vertical_density.find_x_for_y(*it);
		draw_featureline_3D(mesh_AreaEqualization_2D, mesh_AreaEqualization_3D, "horizontal", k);
	}


	mesh_AreaEqualization_2D.write_m((path + name_opt_2D).c_str());
	mesh_AreaEqualization_3D.write_m((path + name_opt_3D).c_str());

	/*
	 CViewerTrait vtrait( & mesh );
	normalize_mesh( &mesh );
	compute_normal( &mesh );
	load_uv();
	set_check_image( 16 );
	// glut stuff
	  glutInit(&argc, argv);                // Initialize GLUT
	  glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH );
	  glutInitWindowSize(800, 800);
	  glutCreateWindow("Mesh Viewer");        // Create window with given title
	  glViewport(0,0,800,800 );

	  glutDisplayFunc(display);             // Set-up callback functions
	  glutReshapeFunc(reshape);
	  glutMouseFunc(mouseClick);
	  glutMotionFunc(mouseMove);
	  glutKeyboardFunc(keyBoard);
	  setupGLstate();
	initialize_texture();

	glutMainLoop();                       // Start GLUT event-processing loop
	*/

	return 0;
}