/*
 * Part of the Cyclone physics system.
 *
 * Copyright (c) Icosagon 2003. All Rights Reserved.
 *
 * This software is distributed under licence. Use of this software
 * implies agreement with all terms and conditions of the accompanying
 * software licence.
 */

#include <gl/glut.h>
#include <cyclone/cyclone.h>
#include "../app.h"
#include "../timing.h"

#include <stdio.h>
#include <cassert>

#define ROD_COUNT 17
#define PARTICLES 15

#define BASE_MASS 1

using namespace cyclone;

/**
 * The main class definition.
 */
class PuppetAssignment : public MassAggregateApplication
{
	ParticleBungee *support1, *support2, *support3;
	ParticleRod* rods;

	int index;

public:
	PuppetAssignment();
	virtual ~PuppetAssignment();

	virtual void addRod(Particle* first, Particle* second, real length=2);

	virtual const char* getTitle();

	virtual void display();

	virtual void displayAnchor(Vector3 particlePosition, Vector3 anchorPosition);

	virtual void update();

	virtual void key(unsigned char key);
};

PuppetAssignment::PuppetAssignment() : MassAggregateApplication(PARTICLES), rods(0) {
	/*
	* Top 3 anchors 
	* Place 13, 14, 15. 
	* So index 12, 13, 14.
	*/
	particleArray[12].setPosition(-1, 8, 0);
	particleArray[13].setPosition(0, 8, 0);
	particleArray[14].setPosition(1, 8, 0);

	/** Set-up all the masses. */
	for(unsigned i = 0; i < PARTICLES; i++){
		if(i < 12) {
			particleArray[i].setPosition(0, i == 5 ? 8 : 7, 0);
			particleArray[i].setAcceleration(Vector3::GRAVITY);
		}
		particleArray[i].setVelocity(0, 0, 0);
		particleArray[i].setDamping(0.9f);
		particleArray[i].clearAccumulator();
		particleArray[i].setMass(BASE_MASS);
	}

	/** Add all the bungee supports and add them to the force registry.	*/
	support1 = new ParticleBungee(particleArray+12, -20.0f, 0.3f);
	support2 = new ParticleBungee(particleArray+13, -20.0f, 0.3f);
	support3 = new ParticleBungee(particleArray+14, -20.0f, 0.3f);

	world.getForceRegistry().add(particleArray, support1);
	world.getForceRegistry().add(particleArray+1, support2);
	world.getForceRegistry().add(particleArray+2, support3);

	/** Adding all the rods. */
	index = 0;
	rods = new cyclone::ParticleRod[ROD_COUNT];
	// Left arm
	addRod(particleArray, particleArray+3, 2);
	addRod(particleArray, particleArray+4, 0.6f);
	addRod(particleArray+3, particleArray+4, 1.8f);
	// Right arm
	addRod(particleArray+2, particleArray+7, 2);
	addRod(particleArray+2, particleArray+6, 0.6f);
	addRod(particleArray+6, particleArray+7, 1.8f);
	// Neck and head
	addRod(particleArray+1, particleArray+5, 0.6f);
	addRod(particleArray+4, particleArray+5, 0.9f);
	addRod(particleArray+5, particleArray+6, 0.9f);
	addRod(particleArray+4, particleArray+6, 1.7f);
	// Body
	addRod(particleArray+4, particleArray+8, 2);
	addRod(particleArray+6, particleArray+9, 2);
	addRod(particleArray+8, particleArray+9, 1.7f);
	// Legs
	addRod(particleArray+8, particleArray+10, 1.5f);
	addRod(particleArray+9, particleArray+11, 1.5f);
	// Top anchors
	addRod(particleArray+12, particleArray+13, 1);
	addRod(particleArray+13, particleArray+14, 1);
}

/*
* Simple destructor.
*/
PuppetAssignment::~PuppetAssignment() {
	if(support1) delete support1;
	if(support2) delete support2;
	if(support3) delete support3;
	if(rods) delete[] rods;
}

/*
* Simple utility function for easely adding rods.
*/
void PuppetAssignment::addRod(Particle* first, Particle* second, real length){
	rods[index].particle[0] = first;
	rods[index].particle[1] = second;
	rods[index].length = length;
	world.getContactGenerators().push_back(&rods[index]);
	index++;
}

/*
* Function to display all the connections.
*/
void PuppetAssignment::display() {
	MassAggregateApplication::display();
	
	glBegin(GL_LINES);
	glColor3f(0,0,1); // Blue

	/** Drawing all the rods. */
    for (unsigned i = 0; i < ROD_COUNT; i++)
    {
		cyclone::Particle **particles = rods[i].particle;
        const cyclone::Vector3 &p0 = particles[0]->getPosition();
        const cyclone::Vector3 &p1 = particles[1]->getPosition();
        glVertex3f(p0.x, p0.y, p0.z);
        glVertex3f(p1.x, p1.y, p1.z);
    }

    glColor3f(1, 0, 0); // Red
	displayAnchor(particleArray[0].getPosition(), particleArray[12].getPosition());
	displayAnchor(particleArray[1].getPosition(), particleArray[13].getPosition());
	displayAnchor(particleArray[2].getPosition(), particleArray[14].getPosition());	
	glEnd();

	renderText(10.0f, 34.0f, "Press 'A' to move to the left.\nPress 'D' to move to the right\nHold 'S' to slow down.");
}

/*
* Simple utlity tool for displaying the bungee connection to the anchors.
*/
void PuppetAssignment::displayAnchor(Vector3 particlePosition, Vector3 anchorPosition) {
	const cyclone::Vector3 &p0 = particlePosition;
	const cyclone::Vector3 &p1 = anchorPosition;
		
    glVertex3f(p0.x, p0.y, p0.z);
    glVertex3f(p1.x, p1.y, p1.z);
}

/*
* Update function. Just calling the base class.
*/
void PuppetAssignment::update() {
	MassAggregateApplication::update();
}

/*
* Configure the title to display at the top of the application window.
*/ 
const char* PuppetAssignment::getTitle() {
	return "Puppet Assignment";
}

/*
* Configure behaviour for key presses.
*/
void PuppetAssignment::key(unsigned char key) {
	
	switch(key)
    {
    case 'a': case 'A':
		particleArray[12].setVelocity(-2.0f, 0, 0);
        break;
    case 'd': case 'D':
		particleArray[12].setVelocity(2.0f, 0, 0);
        break;
	case 's': case 'S':
		particleArray[12].setVelocity(0, 0, 0);
		break;
    default:
        MassAggregateApplication::key(key);
    }
}

/*
* Black box magic for now.
*/ 
Application* getApplication()
{
	return new PuppetAssignment();
}