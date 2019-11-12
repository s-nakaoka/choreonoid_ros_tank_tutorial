#include <cnoid/SimpleController>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>

using namespace std;
using namespace cnoid;

namespace {
const double frequency = 10.0;
}

class JointStateOutputController : public SimpleController
{
    BodyPtr ioBody;
    ros::NodeHandle node;
    ros::Publisher jointStatePublisher;
    sensor_msgs::JointState jointState;
    double time;
    double timeStep;
    double cycleTime;
    double timeCounter;

public:
    JointStateOutputController()
        : node("tank")
    {
        jointStatePublisher = node.advertise<sensor_msgs::JointState>("joint_state", 1000);
    }

    virtual bool initialize(SimpleControllerIO* io) override
    {
        ioBody = io->body();

        const int n = ioBody->numJoints();

        jointState.name.resize(n);
        jointState.position.resize(n);
        jointState.velocity.resize(n);
        jointState.effort.resize(n);

        for(int i=0; i < n; ++i){
            auto joint = ioBody->joint(i);
            io->enableInput(joint, JOINT_DISPLACEMENT | JOINT_VELOCITY | JOINT_EFFORT);
            jointState.name[i] = joint->name();
        }

        time = 0.0;
        timeStep = io->timeStep();
        cycleTime = 1.0 / frequency;
        timeCounter = 0.0;
    }

    virtual bool control() override
    {
        time += timeStep;
        timeCounter += timeStep;

        if(timeCounter >= cycleTime){
            
            jointState.header.stamp.fromSec(time);

            for(int i=0; i < ioBody->numJoints(); ++i){
                auto joint = ioBody->joint(i);
                jointState.position[i] = joint->q();
                jointState.velocity[i] = joint->dq();
                jointState.effort[i] = joint->u();
            }
            
            jointStatePublisher.publish(jointState);

            timeCounter -= cycleTime;
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(JointStateOutputController)
