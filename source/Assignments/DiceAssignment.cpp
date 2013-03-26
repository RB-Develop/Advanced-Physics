#include <gl/glut.h>
#include <cyclone/cyclone.h>
#include "../app.h"
#include "../timing.h"

#include <stdio.h>

#define MAX_DICES 2

using namespace cyclone;

/*
* A normal 6 sided dice.
*/
class NormalDice : public CollisionBox
{
	CollisionSphere* sphere;

public:
	NormalDice(real rad)
	{	
		sphere = new CollisionSphere();

		halfSize = Vector3(rad, rad, rad);
		sphere->radius = rad*1.48f;

		body = new RigidBody();

		body->setMass(3.0f);
		body->setAcceleration(0.0f, -20.0f, 0.0f);
		body->setDamping(0.7f, 0.7f);

		body->setAwake(true);

		cyclone::Matrix3 tensor;
		cyclone::real coeff = 0.4f*body->getMass()*rad*rad;
		tensor.setInertiaTensorCoeffs(coeff,coeff,coeff);
		body->setInertiaTensor(tensor);

		// Set the data common to all particle types
		body->setPosition(0, 5, 0);
		body->setRotation(Vector3(5, 5, -5));

		body->calculateDerivedData();

		sphere->body = body;

		sphere->body->setMass(0.2f);
		sphere->body->setDamping(0.8f, 0.8f);

		sphere->body->calculateDerivedData();
	}

	~NormalDice()
	{
		delete body;
		delete sphere;
	}

	void setState(const cyclone::Vector3 &position,
		const cyclone::Quaternion &orientation,
		const cyclone::Vector3 &extents,
		const cyclone::Vector3 &velocity)
	{
		body->setPosition(position);
		body->setOrientation(orientation);
		body->setRotation(cyclone::Vector3(5,0,0));

		body->setAwake();

		body->calculateDerivedData();
		sphere->body->calculateDerivedData();
	}

	void render()
	{
		GLfloat mat[16];
		body->getGLTransform( mat );

		glColor3f(0.4f,0.7f,0.0f);

		glPushMatrix();
		glMultMatrixf(mat);
		glScalef(halfSize.x*2, halfSize.y*2, halfSize.z*2);
		glutSolidCube(1.0f);
		glPopMatrix();
	}

	void update(real duration)
	{	
		sphere->body->integrate(duration);
		sphere->calculateInternals();

		body->integrate( duration );
		calculateInternals();
	}

	CollisionSphere getBoundingSphere()
	{
		return *sphere;
	}
};

class EightSidedDice : public CollisionBox
{
public:
	EightSidedDice(real halfsize)
	{	
		halfSize = Vector3(halfsize, halfsize, halfsize);

		body = new RigidBody();

		body->setMass(3.0f);
		body->setAcceleration(0.0f, -20.0f, 0.0f);
		body->setDamping(0.7f, 0.7f);

		body->setAwake(true);

		cyclone::Matrix3 tensor;
		cyclone::real coeff = 0.4f*body->getMass()*halfsize*halfsize;
		tensor.setInertiaTensorCoeffs(coeff,coeff,coeff);
		body->setInertiaTensor(tensor);

		// Set the data common to all particle types
		body->setPosition(-4, 5, 5);
		body->setRotation(Vector3(5, 15, -5));

		body->calculateDerivedData();
	}

	~EightSidedDice()
	{
		delete body;
	}

	void setState(const cyclone::Vector3 &position,
		const cyclone::Quaternion &orientation,
		const cyclone::Vector3 &extents,
		const cyclone::Vector3 &velocity)
	{
		body->setPosition(position);
		body->setOrientation(orientation);
		body->setRotation(cyclone::Vector3(5,3,0));

		body->setAwake();

		body->calculateDerivedData();
	}

	void render()
	{
		GLfloat mat[16];
		body->getGLTransform( mat );

		glColor3f(0.0f,0.0f,1.0f);

		glPushMatrix();
		glMultMatrixf(mat);
		glScalef(halfSize.x, halfSize.y, halfSize.z);
		glutSolidOctahedron();
		glPopMatrix();
	}

	void update(real duration)
	{	
		body->integrate( duration );
		calculateInternals();
	}
};

/*
* The main demo class definition.
*/
class DiceAssignment : public RigidBodyApplication
{
	NormalDice* dices[MAX_DICES];
	EightSidedDice* octahedron;

	void reset()
	{
		dices[0]->setState(cyclone::Vector3(0,5,0),
			cyclone::Quaternion(),
			cyclone::Vector3(2,2,2),
			cyclone::Vector3(0,1,0));
		octahedron->setState(cyclone::Vector3(0,5,0),
			cyclone::Quaternion(),
			cyclone::Vector3(2,2,2),
			cyclone::Vector3(0,1,0));
	}

	void generateContacts()
	{
		CollisionPlane plane;

		plane.direction = cyclone::Vector3(0,1,0);
		plane.offset = 0;

		cData.reset(maxContacts);
		cData.friction = (cyclone::real)0.9;
		cData.restitution = (cyclone::real)0.1;
		cData.tolerance = (cyclone::real)0.1;

		cyclone::CollisionDetector::eightDiceAndHalfSpace(*octahedron, plane, &cData);

		cyclone::real projectedRadius = dices[0]->halfSize.x * real_abs(plane.direction * dices[0]->getAxis(0)) +
			dices[0]->halfSize.y * real_abs(plane.direction * dices[0]->getAxis(1)) +
			dices[0]->halfSize.z * real_abs(plane.direction * dices[0]->getAxis(2));

		cyclone::real boxDistance = plane.direction * dices[0]->getAxis(3) - projectedRadius;

		if(!(boxDistance <= plane.offset))
			return;

		cyclone::real sphereDistance = plane.direction * dices[0]->getBoundingSphere().getAxis(3) - dices[0]->getBoundingSphere().radius;

		if(!(sphereDistance <= plane.offset))
			return;

		if(sphereDistance <= boxDistance) 
			cyclone::CollisionDetector::boxAndHalfSpace(*dices[0], plane, &cData);
		else
			cyclone::CollisionDetector::sphereAndHalfSpace(dices[0]->getBoundingSphere(), plane, &cData);	
	}

	void updateObjects(cyclone::real duration)
	{
		dices[0]->update(duration);
		octahedron->update(duration);
	}

public:
	DiceAssignment()
	{
		pauseSimulation = false;
		dices[0] = new NormalDice(2);
		octahedron = new EightSidedDice(2);

		reset();
	}

	const char* getTitle()
	{
		return "DiceAssignment";
	}

	void initGraphics()
	{
		GLfloat lightAmbient[] = {0.8f,0.8f,0.8f,1.0f};
		GLfloat lightDiffuse[] = {0.9f,0.95f,1.0f,1.0f};
		GLfloat lightSpecular[] = {0.9f,0.95f,1.0f,1.0f};

		glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);
		glLightfv(GL_LIGHT0, GL_SPECULAR, lightSpecular);

		glEnable(GL_LIGHT0);

		Application::initGraphics();
	}

	void display()
	{
		const static GLfloat lightPosition[] = {-1, 1, 0, 0};

		RigidBodyApplication::display();

		// Draw some scale circles
		glColor3f( 0.75, 0.75, 0.75 );
		for( unsigned i = 1 ; i < 20 ; ++i )
		{
			glBegin( GL_LINE_LOOP );
			for( unsigned j = 0 ; j < 32 ; ++j )
			{
				float theta = 3.1415926 * j / 16.0;
				glVertex3f( i * cosf( theta ), 0.0, i * sinf( theta ) );
			}
			glEnd();
		}

		glBegin( GL_LINES);
		glVertex3f( -20, 0 ,0 );
		glVertex3f( 20, 0, 0 );
		glVertex3f( 0, 0, -20 );
		glVertex3f( 0, 0, 20 );
		glEnd();

		glEnable( GL_DEPTH_TEST );
		glEnable( GL_LIGHTING );
		glLightfv( GL_LIGHT0, GL_POSITION, lightPosition );
		glColorMaterial( GL_FRONT_AND_BACK, GL_SPECULAR );
		glEnable( GL_COLOR_MATERIAL );

		glPolygonMode( GL_FRONT_AND_BACK, GL_DIFFUSE );

		dices[0]->render();
		octahedron->render();

		glDisable( GL_COLOR_MATERIAL );
		glDisable( GL_LIGHTING );
		glDisable( GL_DEPTH_TEST );
	}

	void Select( int x, int y )
	{
		GLuint b[100] = { 0 };
		GLint h, v[4];
		int id;

		glSelectBuffer( 3, b );

		glGetIntegerv( GL_VIEWPORT, v );

		glRenderMode( GL_SELECT );

		glInitNames();
		glPushName( 0 );

		glMatrixMode( GL_PROJECTION );
		glPushMatrix();
		glLoadIdentity();

		gluPickMatrix( x, v[3] - y, 13, 13, v );
		gluPerspective( 60.0, (double) 640 / (double) 320, 1.0, 500.0 );

		glMatrixMode( GL_MODELVIEW );

		glutSwapBuffers();

		glMatrixMode( GL_PROJECTION );
		glPopMatrix();

		h = glRenderMode( GL_RENDER );

		printf( "%i hits\n", h );

		glMatrixMode( GL_MODELVIEW );
	}

	void mouseDrag(int x, int y)
	{
		RigidBodyApplication::mouseDrag(x, y);

		// Remember the position
		last_x = x;
		last_y = y;
	}
	
	void mouse(int button, int state, int x, int y)
	{
		Select(x, y);
		
		// Remember the position
		last_x = x;
		last_y = y;
	}

	void key(unsigned char key)
	{
		RigidBodyApplication::key(key);
	}
};

Application* getApplication()
{
	return new DiceAssignment();
}