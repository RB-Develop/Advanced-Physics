#include <gl/glut.h>
#include <cyclone/cyclone.h>
#include "../app.h"
#include "../timing.h"

#include <stdio.h>
#include <iostream>

#define MAX_DICES 2
#define EXTRA 1;
// 1 2



// Euhm, op hoop van zegen
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
		sphere->radius = rad*1.45f;

		body = new RigidBody();

		body->setMass(5.0f);
		body->setAcceleration(0.0f, -30.0f, 0.0f);
		body->setDamping(0.2f, 0.2f);

		body->setAwake(true);
		body->setCanSleep(false);

		cyclone::Matrix3 tensor;
		cyclone::real coeff = 0.4f*body->getMass()*rad*rad;
		tensor.setInertiaTensorCoeffs(coeff,coeff,coeff);
		body->setInertiaTensor(tensor);

		// Set the data common to all particle types
		body->setPosition(0, 5, 0);
		body->setRotation(Vector3(5, 5, -5));

		body->calculateDerivedData();

		sphere->body = body;

		sphere->body->setMass(1.0f);
		sphere->body->setDamping(1.01f, 1.01f);

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

		body->setMass(6.0f);
		body->setAcceleration(0.0f, -30.0f, 0.0f);
		body->setDamping(0.7f, 0.7f);

		body->setAwake(true);
		body->setCanSleep(false);

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
	NormalDice* dice;
	EightSidedDice* octahedron;

	RigidBody *dragPoint;
	Joint *dragJoint;

	GLfloat zWin;

	Vector3 previousScreenVector, screenVector;

	bool dragging_Dice;

	void reset()
	{
		dice->setState(cyclone::Vector3(0,5,0),
			cyclone::Quaternion(),
			cyclone::Vector3(2,2,2),
			cyclone::Vector3(0,1,0));
		octahedron->setState(cyclone::Vector3(0,5,0),
			cyclone::Quaternion(),
			cyclone::Vector3(2,2,2),
			cyclone::Vector3(0,1,0));

		dragging_Dice = false;
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

		if(dragging_Dice)
			cData.addContacts(dragJoint->addContact(cData.contacts, cData.contactsLeft));

		CollisionDetector::eightDiceAndHalfSpace(*octahedron, plane, &cData);
		CollisionDetector::boxAndBox(*octahedron, *dice, &cData);

		real projectedRadius = dice->halfSize.x * real_abs(plane.direction * dice->getAxis(0)) +
			dice->halfSize.y * real_abs(plane.direction * dice->getAxis(1)) +
			dice->halfSize.z * real_abs(plane.direction * dice->getAxis(2));

		real boxDistance = plane.direction * dice->getAxis(3) - projectedRadius;

		if(!(boxDistance <= plane.offset))
			return;

		real sphereDistance = plane.direction * dice->getBoundingSphere().getAxis(3) - dice->getBoundingSphere().radius;

		if(!(sphereDistance <= plane.offset))
			return;

		if(sphereDistance <= boxDistance || dice->body->getRotation() < Vector3(1.0f, 1.0f, 1.0f)) 
			CollisionDetector::boxAndHalfSpace(*dice, plane, &cData);
		else
			CollisionDetector::sphereAndHalfSpace(dice->getBoundingSphere(), plane, &cData);
	}

	void updateObjects(cyclone::real duration)
	{
		dice->update(duration);
		octahedron->update(duration);

		//dragPoint->integrate(duration);
	}

public:
	DiceAssignment()
	{
		pauseSimulation = false;
		dice = new NormalDice(2);
		octahedron = new EightSidedDice(2);
		dragging_Dice = false;

		dragJoint = new Joint();

		dragPoint = new RigidBody();
	
		dragPoint->setMass(100.0f);
		dragPoint->setDamping(0.0f, 0.0f);

		dragPoint->setAwake(true);
		dragPoint->setCanSleep(false);

		dragPoint->calculateDerivedData();

		reset();
	}

	~DiceAssignment()
	{
		delete octahedron;
		delete dice;
	}

	const char* getTitle()
	{
		return "DiceAssignment";
	}

	void initGraphics()
	{
		GLfloat lightAmbient[] = {0.8f,0.8f,0.8f,1.0f};
		GLfloat lightDiffuse[] = {0.9f,0.95f,1.0f,1.0f};

		glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);
		glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);

		glEnable(GL_LIGHT0);

		Application::initGraphics();
	}

	void display()
	{
		const static GLfloat lightPosition[] = {-0.5f, 0.5f, 0, 0};

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
		glColorMaterial( GL_FRONT_AND_BACK, GL_DIFFUSE );
		glEnable( GL_COLOR_MATERIAL );

		glPolygonMode( GL_FRONT_AND_BACK, GL_DIFFUSE );

		dice->render();
		octahedron->render();

		glDisable( GL_COLOR_MATERIAL );
		glDisable( GL_LIGHTING );
		glDisable( GL_DEPTH_TEST );
	}

	void debugRenderDragPoint()
	{
		GLfloat mat[16];
		dragPoint->getGLTransform( mat );

		glColor3f(0.0f,0.0f,1.0f);

		glPushMatrix();
		glMultMatrixf(mat);
		glutWireCube(1.0f);
		glPopMatrix();
	}

	Vector3 GetOGLPos(int x, int y)
	{
		GLint viewport[4];
		GLdouble modelview[16];
		GLdouble projection[16];
		GLfloat winX, winY, winZ;
		GLdouble posX, posY, posZ;

		glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
		glGetDoublev( GL_PROJECTION_MATRIX, projection );
		glGetIntegerv( GL_VIEWPORT, viewport );
 
		winX = (float)x;
		winY = (float)viewport[3] - (float)y;
		glReadPixels( x, int(winY), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT, &winZ );

		zWin = winZ;

		gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);

		return Vector3(posX, posY, posZ);
	}

	Vector3 GetOGLPos2(int x, int y, float winZ)
	{
		GLint viewport[4];
		GLdouble modelview[16];
		GLdouble projection[16];
		GLfloat winX, winY;
		GLdouble posX, posY, posZ;

		glGetDoublev( GL_MODELVIEW_MATRIX, modelview );
		glGetDoublev( GL_PROJECTION_MATRIX, projection );
		glGetIntegerv( GL_VIEWPORT, viewport );
 
		winX = (float)x;
		winY = (float)viewport[3] - (float)y;
 
		gluUnProject( winX, winY, winZ, modelview, projection, viewport, &posX, &posY, &posZ);

		return Vector3(posX, posY, posZ);
	}

	void mouseDrag(int x, int y)
	{
		screenVector = GetOGLPos2(x, y, zWin);

		if(dragging_Dice){
			dragPoint->setPosition(screenVector);
			dragPoint->calculateDerivedData();
		}

		// Remember the position
		last_x = x;
		last_y = y;

		previousScreenVector = screenVector;
	}
	
	void mouse(int button, int state, int x, int y)
	{
		if(state==GLUT_DOWN){
			dragPoint->setPosition(GetOGLPos(x,y));
			dragPoint->calculateDerivedData();

			if(CollisionDetector::boxAndPoint(*dice, GetOGLPos(x,y), &cData)) {
				dragJoint->set(dice->body, dice->getBoundingSphere().body->getPointInLocalSpace(GetOGLPos(x,y)), dragPoint, dragPoint->getPointInLocalSpace(GetOGLPos(x,y)), (real)0);
				dragging_Dice = true;
			}
			if(CollisionDetector::boxAndPoint(*octahedron, GetOGLPos(x,y), &cData)) {
				dragJoint->set(octahedron->body, octahedron->body->getPointInLocalSpace(GetOGLPos(x,y)), dragPoint, dragPoint->getPointInLocalSpace(GetOGLPos(x,y)), (real)0);
				dragging_Dice = true;
			}
		}
		if(state==GLUT_UP){
			dragJoint->set(NULL, Vector3(), NULL, Vector3(), (real)0);
			dragging_Dice = false;
		}

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


// WHEEEEEEEEEEEEEEEE
// YAAAAAAAYYYY

// Meer changes in de branch.