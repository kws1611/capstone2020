#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "getImage");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher image_pub = it.advertise("/camera/image_raw",1);
    
    // Open Camera
    cv::VideoCapture capture(0);

    if(!capture.isOpened())
        std::cout<<"Camera can't open"<<std::endl;

    // Load camera data and Publish
    cv::Mat frame;
    sensor_msgs::Image::ConstPtr msg;

    ros::Rate rate(30);

    while(ros::ok())
    {
        capture.read(frame);
        
        if(!frame.empty())
        {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            image_pub.publish(msg);
        }

        else
            std::cout<<"No image data"<<std::endl;
              
        rate.sleep();
    }

    return 0;
}