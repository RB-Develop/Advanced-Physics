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
	NormalDice(real x, real y, real z, real rad);

	~NormalDice();

	virtual void render();

	virtual void update(real duration);

	virtual CollisionSphere getBoundingSphere();
};

NormalDice::NormalDice(real x, real y, real z, real rad)
{	
	sphere = new CollisionSphere();

	halfSize = Vector3(rad, rad, rad);
	sphere->radius = rad*1.5f;

	body = new RigidBody();

	body->setMass(1.0f);
	body->setAcceleration(0.0f, -25.0f, 0.0f);
	body->setDamping(0.7f, 0.7f);

	body->setAwake(true);

	cyclone::Matrix3 tensor;
	cyclone::real coeff = 0.4f*body->getMass()*rad*rad;
	tensor.setInertiaTensorCoeffs(coeff,coeff,coeff);
	body->setInertiaTensor(tensor);

	// Set the data common to all particle types
	body->setPosition(x, y, z);
	body->setRotation(Vector3(5, 5, 0));

	body->calculateDerivedData();

	sphere->body = body;

	sphere->body->setMass(0.1f);
	sphere->body->setDamping(0.8f, 0.8f);

	sphere->body->calculateDerivedData();
}

NormalDice::~NormalDice()
{
	delete body;

	delete sphere;
}

void NormalDice::render()
{
	GLfloat mat[16];
	body->getGLTransform( mat );

	glPushMatrix();
	glMultMatrixf( mat );
	glPushMatrix();

	glutWireCube(halfSize.x*2);
	//glutSolidCube(halfSize.x*2);

	glPopMatrix();
	
	sphere->body->getGLTransform( mat );

	glPushMatrix();
	glMultMatrixf( mat );
	glPushMatrix();

	//glutWireSphere(NormalDice::getBoundingSphere().radius, 10, 10 );

	glPopMatrix();
}

void NormalDice::update(real duration)
{	
	sphere->body->integrate(duration);
	sphere->calculateInternals();

	body->integrate( duration );
	calculateInternals();
}

CollisionSphere NormalDice::getBoundingSphere()
{
	return *sphere;
}

/*
* The main demo class definition.
*/
class DiceAssignment : public RigidBodyApplication
{
	NormalDice* dices[MAX_DICES];
	unsigned first;

	virtual void reset();

	virtual void generateContacts();

	virtual void updateObjects(cyclone::real duration);
public:
	DiceAssignment();

	virtual const char* getTitle();

	virtual void initGraphics();

	virtual void display();

	virtual void mouse(int button, int state, int x, int y);

	virtual void key(unsigned char key);
};

DiceAssignment::DiceAssignment()
	: 
	RigidBodyApplication()
{
	pauseSimulation = false;

	reset();
}


void DiceAssignment::initGraphics()
{
	Application::initGraphics();
}

void DiceAssignment::reset()
{
	dices[0] = new NormalDice(0, 10, 0, 2);
}

const char* DiceAssignment::getTitle()
{
	return "DiceAssignment";
}

void DiceAssignment::updateObjects(cyclone::real duration)
{
	dices[0]->update(duration);
}

void DiceAssignment::display()
{
	const static GLfloat lightPosition[] = {-1, 1, 0, 0};

	glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );
	glLoadIdentity();
	gluLookAt( -25.0, 8.0, 5.0, 0.0, 5.0, 0.0, 0.0, 1.0, 0.0 );

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
	glColor3f( 1.0, 1.0, 1.0 );

	glPolygonMode( GL_FRONT_AND_BACK, GL_LINE );

	dices[0]->render();

	glDisable( GL_COLOR_MATERIAL );
	glDisable( GL_LIGHTING );
	glDisable( GL_DEPTH_TEST );
}


void DiceAssignment::generateContacts()
{
	// Create the ground plane data
	CollisionPlane plane;

	plane.direction = cyclone::Vector3(0,1,0);
	plane.offset = 0;

	// Set up the collision data structure
	cData.reset(maxContacts);
	cData.friction = (cyclone::real)0.9;
	cData.restitution = (cyclone::real)0.1;
	cData.tolerance = (cyclone::real)0.1;

	//CollisionDetector::boxAndHalfSpace(*dices[0], plane, &cData);
	
	// Work out the projected radius of the box onto the plane direction.
	cyclone::real projectedRadius = dices[0]->halfSize.x * real_abs(plane.direction * dices[0]->getAxis(0)) +
		dices[0]->halfSize.y * real_abs(plane.direction * dices[0]->getAxis(1)) +
		dices[0]->halfSize.z * real_abs(plane.direction * dices[0]->getAxis(2));

	// Work out how far the box is from the origin.
	cyclone::real boxDistance = plane.direction * dices[0]->getAxis(3) - projectedRadius;

	// Check for the box intersection. If no intersection, then stop further calculations.
	if(!(boxDistance <= plane.offset))
		return;

	// Find the distance from the origin.
	cyclone::real sphereDistance = plane.direction * dices[0]->getBoundingSphere().getAxis(3) - dices[0]->getBoundingSphere().radius;

	// Check for the sphere intersection. If no intersection, then stop further calculations.
	if(!(sphereDistance <= plane.offset))
		return;

	// If the distance between the sphere and plane is equal or smaller than the distance between the box and plane,
	// then collide with the box. Else collide with the sphere.
	if(sphereDistance <= boxDistance) 
		cyclone::CollisionDetector::boxAndHalfSpace(*dices[0], plane, &cData);
	else
		cyclone::CollisionDetector::sphereAndHalfSpace(dices[0]->getBoundingSphere(), plane, &cData);	
		
}

void DiceAssignment::mouse(int button, int state, int x, int y)
{
	printf("Mouse x: %i, Mouse y: %i\n", x, y);
}

void DiceAssignment::key(unsigned char key)
{

}

Application* getApplication()
{
	return new DiceAssignment();
}