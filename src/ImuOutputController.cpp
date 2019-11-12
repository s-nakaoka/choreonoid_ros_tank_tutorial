#include <cnoid/SimpleController>
#include <cnoid/AccelerationSensor>
#include <cnoid/RateGyroSensor>
#include <ros/node_handle.h>
#include <sensor_msgs/Imu.h>

using namespace std;
using namespace cnoid;

namespace {
const double frequency = 20.0;
}

class ImuOutputController : public SimpleController
{
    AccelerationSensorPtr accelSensor;
    RateGyroSensorPtr gyro;
    ros::NodeHandle node;
    ros::Publisher imuPublisher;
    sensor_msgs::Imu imu;
    double time;
    double timeStep;
    double cycleTime;
    double timeCounter;

public:
    ImuOutputController()
        : node("tank")
    {
        imuPublisher = node.advertise<sensor_msgs::Imu>("imu", 1000);
    }

    virtual bool initialize(SimpleControllerIO* io) override
    {
        accelSensor = io->body()->findDevice<AccelerationSensor>("ACCEL_SENSOR");
        gyro = io->body()->findDevice<RateGyroSensor>("GYRO");
        
        io->enableInput(accelSensor);
        io->enableInput(gyro);

        accelSensor->sigStateChanged().connect(
            [&](){
                auto dv = accelSensor->dv();
                imu.linear_acceleration.x = dv.x();
                imu.linear_acceleration.y = dv.y();
                imu.linear_acceleration.z = dv.z();
            });

        gyro->sigStateChanged().connect(
            [&](){
                auto w = gyro->w();
                imu.angular_velocity.x = w.x();
                imu.angular_velocity.y = w.y();
                imu.angular_velocity.z = w.z();
            });

        for(int i=0; i < 9; ++i){
            imu.orientation_covariance[i] = 0.0;
            imu.angular_velocity_covariance[i] = 0.0;
            imu.linear_acceleration_covariance[i] = 0.0;
        }
        imu.orientation_covariance[0] = -1.0;
        imu.orientation.x = 0.0;
        imu.orientation.y = 0.0;
        imu.orientation.z = 0.0;
        imu.orientation.w = 0.0;

        timeStep = io->timeStep();
        cycleTime = 1.0 / frequency;
        timeCounter = 0.0;
    }

    virtual bool control() override
    {
        time += timeStep;
        timeCounter += timeStep;

        if(timeCounter >= cycleTime){
            imu.header.stamp.fromSec(time);
            imuPublisher.publish(imu);
            timeCounter -= cycleTime;
        }

        return true;
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(ImuOutputController)
