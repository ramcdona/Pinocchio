#include "../Pinocchio/skeleton.h"
#include "../Pinocchio/utils.h"
#include "../Pinocchio/debugging.h"
#include "../Pinocchio/attachment.h"
#include "../Pinocchio/mesh.h"
#include "../Pinocchio/pinocchioApi.h"

#include "cube_model.h"

#include <stdint.h>
#include <limits.h>
#include <string>
#include <fstream>

#include <GL/gl.h>

struct LineSegment
{
  LineSegment() : thickness(1.) {}
  LineSegment(const Vector3 &inP1, const Vector3 &inP2,
    const Vector3 &inColor = Vector3(1, 1, 1), double inThickness = 1.)
    : p1(inP1), p2(inP2), color(inColor), thickness(inThickness) {}

  Vector3 p1, p2, color;
  double thickness;
};

class DisplayMesh
{
  public:
    virtual ~DisplayMesh() {}

    virtual const Mesh &getMesh(int &framenum) = 0;
    virtual std::vector<Vector3> getSkel() const { return std::vector<Vector3>(); }
};

class StaticDisplayMesh : public DisplayMesh
{
  public:
    StaticDisplayMesh(const Mesh &inM) : m(inM) {}

    virtual const Mesh &getMesh(int &framenum) { return m; }
  private:

    Mesh m;
};

void load_model(const char * obj_file_name) {
	// Calculate Skeleton and Attachment Values with Pinocchio
	Mesh m(obj_file_name, Mesh::DQS);
	Quaternion<> meshTransform;
	double skelScale = 1.;
	double stiffness = 1.;
	bool fitSkeleton = true;
	assert (m.vertices.size() != 0);
	for (int i = 0; i < (int)m.vertices.size(); ++i) {
		m.vertices[i].pos = meshTransform * m.vertices[i].pos;
	}
	m.normalizeBoundingBox();
	m.computeVertexNormals();

	std::string skeletonName("human");
	Skeleton skeleton;
	if (skeletonName == std::string("human")) {
		skeleton = HumanSkeleton();
	} else if (skeletonName == std::string("horse")) {
		skeleton = HorseSkeleton();
	} else if (skeletonName == std::string("quad")) {
		skeleton = QuadSkeleton();
	} else if (skeletonName == std::string("centaur")) {
		skeleton = CentaurSkeleton();
	} else {
		skeleton = FileSkeleton(skeletonName);
	}

	Skeleton given = skeleton;
	given.scale(skelScale * 0.7);
	PinocchioOutput o;
	if (fitSkeleton) {
		o = autorig(given, m);
	} else { // skip the fitting step--assume the skeleton is already correct for the mesh
		TreeType *distanceField = constructDistanceField(m);
		VisTester<TreeType> *tester = new VisTester<TreeType>(distanceField);
		o.embedding = skeleton.fGraph().verts;
		for (int i = 0; i < (int)o.embedding.size(); ++i) {
			o.embedding[i] = m.toAdd + o.embedding[i] * m.scale;
		}
		o.attachment = new Attachment(m, skeleton, o.embedding, tester, stiffness);
		delete tester;
		delete distanceField;
	}
	assert (o.embedding.size() != 0);

	// output skeleton embedding
	std::string skelOutName("skeleton.out");
	for (int i = 0; i < (int)o.embedding.size(); ++i) {
		o.embedding[i] = (o.embedding[i] - m.toAdd) / m.scale;
	}
	std::ofstream os(skelOutName.c_str());
	for (int i = 0; i < (int)o.embedding.size(); ++i) {
		os << i << " " << o.embedding[i][0] << " " << o.embedding[i][1] <<
		" " << o.embedding[i][2] << " " << skeleton.fPrev()[i] << std::endl;
	}

	// output attachment
	std::string weightOutName("attachment.out");
	std::ofstream astrm(weightOutName.c_str());
	for (int i = 0; i < (int)m.vertices.size(); ++i) {
		Vector<double, -1> v = o.attachment->getWeights(i);
		for (int j = 0; j < v.size(); ++j) {
			double d = floor(0.5 + v[j] * 10000.) / 10000.;
			astrm << d << " ";
		}
		astrm << std::endl;
	}
	delete o.attachment;

	//printf("Mesh vertices: ");
	//for (std::vector<MeshVertex>::iterator it = m.vertices.begin() ; it != m.vertices.end(); ++it) {
	//	printf("(%f, %f, %f) ", it->pos[0], it->pos[1], it->pos[2]);
	//}
	//printf("\n");

	//printf("Mesh normals: ");
	//for (std::vector<MeshVertex>::iterator it = m.vertices.begin() ; it != m.vertices.end(); ++it) {
	//	printf("(%f, %f, %f) ", it->normal[0], it->normal[1], it->normal[2]);
	//}
	//printf("\n");
}


void initGL() {
  static GLfloat pos[4] = { 5.0, 5.0, 10.0, 1.0 };

  glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
  glLightfv( GL_LIGHT0, GL_POSITION, pos );
  glEnable(GL_LIGHTING);
  glEnable(GL_LIGHT0);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_NORMALIZE);
  glDisable(GL_ALPHA_TEST);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
  glShadeModel(GL_FLAT);
  glClearColor(0.f, 0.f, 0.f, 0.f);
}

void init_draw(double w, double h) {
  // Init viewport and projection
    initGL();

    glViewport(0, 0, (int)w, (int)h);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();

    double left = -.1;
    double bottom = -.1;
    double right = 1.1;
    double top = 1.1;

    if (w > 1 && h > 1) {
      if (w > h) {
        right = -0.1 + 1.2 * w / h;
      }
      if (h > w) {
        bottom = 1.1 - 1.2 * h / w;
      }
    }

    double scale = 1. / 1000.;
    left = -w * scale;
    right = w * scale;
    bottom = -h * scale;
    top = h * scale;
    glFrustum(left, right, bottom, top, 5., 30.);

    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
}

void draw_mesh(const Mesh &m, bool flatShading, Vector3 trans = Vector3()) {
  int i;
  Vector3 normal;

  glBegin(GL_TRIANGLES);
  for (i = 0; i < (int)m.edges.size(); ++i)
  {
    int v = m.edges[i].vertex;
    const Vector3 &p = m.vertices[v].pos;

    if (!flatShading) {
      normal = m.vertices[v].normal;
      glNormal3d(normal[0], normal[1], normal[2]);
    } else if (i % 3 == 0) {
      const Vector3 &p2 = m.vertices[m.edges[i + 1].vertex].pos;
      const Vector3 &p3 = m.vertices[m.edges[i + 2].vertex].pos;

      normal = ((p2 - p) % (p3 - p)).normalize();
      glNormal3d(normal[0], normal[1], normal[2]);
    }

    glVertex3d(p[0] + trans[0], p[1] + trans[1], p[2] + trans[2]);
  }
  glEnd();
}

void draw_floor(bool flatShading) {
  int i;
  Mesh floor;
  floor.vertices.resize(4);
  for (i = 0; i < 4; ++i) {
    floor.vertices[i].normal = Vector3(0, 1, 0);
    floor.vertices[i].pos = 10. * Vector3(((i + 0) % 4) / 2, 0, ((i + 1) % 4) / 2) - Vector3(4.5, 0, 4.5);
  }
  floor.edges.resize(6);
  for (i = 0; i < 6; ++i) {
    floor.edges[i].vertex = (i % 3) + ((i > 3) ? 1 : 0);
  }

  static GLfloat colrb[4] = {0.5f, .9f, .75f, 1.0f };
  static GLfloat colr[4] = {0.5f, .6f, .9f, 1.0f };
  glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, colr);
  glMaterialfv( GL_BACK, GL_AMBIENT_AND_DIFFUSE, colrb);

  glShadeModel(GL_SMOOTH);
  draw_mesh(floor, false);
  glShadeModel( flatShading ? GL_FLAT : GL_SMOOTH);

  glColor4d(.5, .6, .9, .3);
  glLineWidth(1.);
  int gridSize = 20;
  double y = floor.vertices[0].pos[1];
  double minX = floor.vertices[1].pos[0];
  double maxX = floor.vertices[2].pos[0];
  double minZ = floor.vertices[1].pos[2];
  double maxZ = floor.vertices[3].pos[2];
  double stepX = (maxX - minX) / double(gridSize);
  double stepZ = (maxZ - minZ) / double(gridSize);
  glEnable(GL_BLEND);
  glDisable(GL_LIGHTING);
  glDisable(GL_DEPTH_TEST);
  glBegin(GL_LINES);
  for (i = 0; i <= gridSize; ++i) {
    glVertex3d(minX + i * stepX, y, minZ);
    glVertex3d(minX + i * stepX, y, maxZ);
    glVertex3d(minX, y, minZ + i * stepZ);
    glVertex3d(maxX, y, minZ + i * stepZ);
  }
  glEnd();
  glEnable(GL_LIGHTING);
  glEnable(GL_DEPTH_TEST);
  glDisable(GL_BLEND);
}

void draw_model(Transform<> transform, std::vector<DisplayMesh *> meshes, std::vector<LineSegment> lines, HumanSkeleton human, bool floor, bool skeleton, bool flatShading, bool paused) {
  int i;
  static int framenum;

  glEnable(GL_DEPTH_TEST);
  glEnable(GL_LIGHTING);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glLoadIdentity();

  // Transform
  Vector3 trans = transform.getTrans();
  glTranslated(trans[0], trans[1], -10 + trans[2]);

  double scale = transform.getScale();
  glScaled(scale, scale, scale);

  Quaternion<> r = transform.getRot();
  double ang = r.getAngle();
  if (fabs(ang) > 1e-6) {
    Vector3 ax = r.getAxis();
    glRotated(ang * 180. / M_PI, ax[0], ax[1], ax[2]);
  }

  if (floor) {
    draw_floor(flatShading);
  }

  // Get mesh to draw, but only if not paused.
  static std::vector<const Mesh *> ms(meshes.size());
  if (!paused) {
    for (i = 0; i < (int)meshes.size(); ++i) {
      ms[i] = &(meshes[i]->getMesh(framenum));
    }
  }

  // display frame number
  std::stringstream strs;
  strs << framenum;
  std::string temp = strs.str();
  //const char* strFramenum = temp.c_str();

  // Shadows
  if (floor) {
    Vector3 lightRay = transform.getRot().inverse() * Vector3(1, 2, 2);
    if (lightRay[1] == 0)
      lightRay[1] = 1e-5;
    lightRay = -lightRay / lightRay[1];

    glDisable(GL_LIGHTING);
    glColor3f(0.1f, 0.1f, 0.1f);
    glPushMatrix();
    float matr[16] = {1,0,0,0, (float)lightRay[0],0,(float)lightRay[2],0, 0,0,1,0, 0,0.01f,0,1};
    glMultMatrixf(matr);
    glDepthMask(0);
    for (i = 0; i < (int)ms.size(); ++i)
      draw_mesh(*(ms[i]), flatShading);
    glDepthMask(1);
    glEnable(GL_LIGHTING);
    glPopMatrix();
  }

  static GLfloat colr[4] = {1.f, .9f, .75f, 1.0f };
  static GLfloat colrb[4] = {1.f, .9f, .75f, 1.0f };
  glMaterialfv( GL_FRONT, GL_AMBIENT_AND_DIFFUSE, colr);
  glMaterialfv( GL_BACK, GL_AMBIENT_AND_DIFFUSE, colrb);

  // Draw meshes
  for (i = 0; i < (int)meshes.size(); ++i) {
    draw_mesh(*(ms[i]), flatShading);
  }

  // Draw lines
  glDisable(GL_DEPTH_TEST);
  glDisable(GL_LIGHTING);
  for (i = 0; i < (int)lines.size(); ++i) {
    glColor3d(lines[i].color[0], lines[i].color[1], lines[i].color[2]);
    glLineWidth((float)lines[i].thickness);
    glBegin(GL_LINES);
    glVertex3d(lines[i].p1[0], lines[i].p1[1], lines[i].p1[2]);
    glVertex3d(lines[i].p2[0], lines[i].p2[1], lines[i].p2[2]);
    glEnd();
  }

  if (skeleton) {
    glLineWidth(5);
    for (i = 0; i < (int)meshes.size(); ++i) {
      std::vector<Vector3> v = meshes[i]->getSkel();
      if (v.size() == 0) {
        continue;
      }
      glColor3d(.5, 0, 0);

      const std::vector<int> &prev = human.fPrev();
      glBegin(GL_LINES);
      for (int j = 1; j < (int)prev.size(); ++j) {
        int k = prev[j];
        glVertex3d(v[j][0], v[j][1], v[j][2]);
        glVertex3d(v[k][0], v[k][1], v[k][2]);
      }
      glEnd();
    }
  }
}
