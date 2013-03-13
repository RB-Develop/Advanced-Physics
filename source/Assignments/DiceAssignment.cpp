#include <gl/glut.h>
#include <cyclone/cyclone.h>
#include "../app.h"
#include "../timing.h"

#include <stdio.h>


/**
 * The main demo class definition.
 */
class DiceAssignment : public RigidBodyApplication
{
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
    reset();
}


void DiceAssignment::initGraphics()
{
	//Application::initGraphics();
}

void DiceAssignment::reset()
{

}

const char* DiceAssignment::getTitle()
{
    return "DiceAssignment";
}

void DiceAssignment::updateObjects(cyclone::real duration)
{

}

void DiceAssignment::display()
{

}

void DiceAssignment::generateContacts()
{

}

void DiceAssignment::mouse(int button, int state, int x, int y)
{

}

void DiceAssignment::key(unsigned char key)
{

}

Application* getApplication()
{
    return new DiceAssignment();
}