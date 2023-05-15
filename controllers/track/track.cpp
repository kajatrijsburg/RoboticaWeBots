// File:          track.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add webots include files such as
// <webots/DistanceSensor.hpp>, <webots/Motor.hpp>, etc.
// and/or to add some other includes
#include <webots/Robot.hpp>
#include <webots/Motor.hpp>
#include <webots/Keyboard.hpp>

// All the webots classes are defined in the "webots" namespace
using namespace webots;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char** argv)
{
    // create Keyboard
    Keyboard keyboard;
    
    // create the Robot instance.
    Robot* robot = new Robot();

    // get the time step of the current world.
    int timeStep = (int)robot->getBasicTimeStep();

    // get the motor devices
    Motor* left_motor = robot->getMotor("left motor");
    Motor* right_motor = robot->getMotor("right motor");

    // You should insert a getDevice-like function in order to get the
    // instance of a device of the robot. Something like:
    //  Motor *motor = robot->getMotor("motorname");
    //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
    //  ds->enable(timeStep);

    // enable the Keyboard
    keyboard.enable(timeStep);

    // set the target position of the motors
    left_motor->setPosition(INFINITY);
    right_motor->setPosition(INFINITY);
    
    // Main loop:
    // - perform simulation steps until Webots is stopping the controller
    while (robot->step(timeStep) != -1)
    {
        // Read the sensors:
        // Enter here functions to read sensor data, like:
        //  double val = ds->getValue();

        const int key = keyboard.getKey();
        if (key == 87) //w
        {
            left_motor->setVelocity(0.1);
            right_motor->setVelocity(0.1);
        }
        else if (key == 65) //a
        {
            left_motor->setVelocity(0);
            right_motor->setVelocity(0.1);
        }
        else if (key == 68) //d
        {
            left_motor->setVelocity(0.1);
            right_motor->setVelocity(0);
        }
        else if (key == 83) //s
        {
            left_motor->setVelocity(-0.1);
            right_motor->setVelocity(-0.1);
        }
        else if (key == 70) //f
        {
            left_motor->setVelocity(0.1);
            right_motor->setVelocity(-0.1);
        }
        else //stop the robot
        {
            left_motor->setVelocity(0);
            right_motor->setVelocity(0);
        }
    };

    // Enter here exit cleanup code.
    delete robot;
    delete left_motor;
    delete right_motor;
    return 0;
}
