#include <cnoid/SimpleController>
#include <ros/node_handle.h>
#include <sensor_msgs/JointState.h>
#include <mutex>

using namespace std;
using namespace cnoid;

class JointStateInputController : public SimpleController
{
    ros::NodeHandle node;
    ros::Subscriber jointStateSubscriber;
    sensor_msgs::JointState jointState;
    std::mutex jointStateMutex;
    
    Link* turretJoint[2];
    double qref[2];
    double qref_next[2];
    double qprev[2];
    double dt;

public:
    virtual bool initialize(SimpleControllerIO* io) override
    {
        ostream& os = io->os();
        Body* body = io->body();
        dt = io->timeStep();

        turretJoint[0] = body->link("TURRET_Y");
        turretJoint[1] = body->link("TURRET_P");
        for(int i=0; i < 2; ++i){
            Link* joint = turretJoint[i];
            qref[i] = qref_next[i] = qprev[i] = joint->q();
            joint->setActuationMode(Link::ActuationMode::JOINT_TORQUE);
            io->enableIO(joint);
        }

        jointStateSubscriber = node.subscribe(
            "tank/joint_state", 1, &JointStateInputController::jointStateCallback, this);

        return true;
    }

    void jointStateCallback(const sensor_msgs::JointState& msg)
    {
        std::lock_guard<std::mutex> lock(jointStateMutex);
        jointState = msg;
    }

    virtual bool control() override
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            int n = std::min(2, (int)jointState.position.size());
            for(int i=0; i < n; ++i){
                qref_next[i] = jointState.position[i];
            }
        }
            
        static const double P = 200.0;
        static const double D = 50.0;

        for(int i=0; i < 2; ++i){
            Link* joint = turretJoint[i];
            double q = joint->q();
            double dq = (q - qprev[i]) / dt;
            double dqref = (qref_next[i] - qref[i]) / dt;
            qref[i] = qref_next[i];
            joint->u() = P * (qref[i] - q) + D * (dqref - dq);
            qprev[i] = q;
        }

        return true;
    }

    virtual void stop() override
    {
        jointStateSubscriber.shutdown();
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(JointStateInputController)
