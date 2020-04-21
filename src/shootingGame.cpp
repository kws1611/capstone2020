#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
    // Get image
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    cv::Mat img = cv_ptr->image;

    // Draw crossline
    int crossLine_size = img.rows/20;
    int crossLine_thickness = 1;

    cv::line(img, cv::Point(img.cols/2, (img.rows-crossLine_size)/2), cv::Point(img.cols/2, (img.rows+crossLine_size)/2), cv::Scalar(0,0,255), crossLine_thickness);
    cv::line(img, cv::Point((img.cols-crossLine_size)/2, img.rows/2), cv::Point((img.cols+crossLine_size)/2, img.rows/2), cv::Scalar(0,0,255), crossLine_thickness);
    
    cv::imshow("test", img);

    int k = cv::waitKey(1);
    if(k == 27)
    {
        ros::shutdown();
        cv::destroyAllWindows();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "shootingGame");

    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/camera/image_raw", 1, imageCallback);
    
    ros::spin();

    return 0;
}