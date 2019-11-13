#include <cnoid/SimpleController>
#include <cnoid/Camera>
#include <ros/node_handle.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>

using namespace std;
using namespace cnoid;

class CameraImageOutputController : public SimpleController
{
    CameraPtr camera;
    ScopedConnection cameraConnection;
    ros::NodeHandle node;
    image_transport::Publisher cameraImagePublisher;
    double time;
    double timeStep;

public:
    CameraImageOutputController()
        : node("tank")
    {
        image_transport::ImageTransport imageTransport(node);
        cameraImagePublisher = imageTransport.advertise("camera/image", 1000);
    }

    virtual bool initialize(SimpleControllerIO* io) override
    {
        camera = io->body()->findDevice<Camera>("Kinect");
        
        io->enableInput(camera);

        cameraConnection =
            camera->sigStateChanged().connect(
                [&](){ publishCameraImage(); });

        time = 0.0;
        timeStep = io->timeStep();

        return true;
    }

    virtual bool control() override
    {
        time += timeStep;
        return true;
    }

    void publishCameraImage()
    {
        sensor_msgs::Image rosImage;
        rosImage.header.stamp.fromSec(time);
        rosImage.header.frame_id = camera->name();
        auto& srcImage = camera->image();
        rosImage.height = srcImage.height();
        rosImage.width = srcImage.width();
        if(srcImage.numComponents() == 3){
            rosImage.encoding = sensor_msgs::image_encodings::RGB8;
        } else if (srcImage.numComponents() == 1){
            rosImage.encoding = sensor_msgs::image_encodings::MONO8;
        } else {
            ROS_WARN("unsupported image component number: %i", srcImage.numComponents());
        }
        rosImage.is_bigendian = 0;
        rosImage.step = srcImage.width() * srcImage.numComponents();
        rosImage.data.resize(rosImage.step * rosImage.height);
        std::memcpy(&(rosImage.data[0]), &(srcImage.pixels()[0]), rosImage.step * rosImage.height);
        cameraImagePublisher.publish(rosImage);
    }
};

CNOID_IMPLEMENT_SIMPLE_CONTROLLER_FACTORY(CameraImageOutputController)
