#include "ros/ros.h"

#include "OpenNI.h"

#include <vector>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/distortion_models.h>

using namespace std;
using namespace openni;

sensor_msgs::CameraInfoPtr getDefaultCameraInfo(int width, int height, double f) 
{
  sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();

  info->width  = width;
  info->height = height;


  info->D.resize(5, 0.0);
  info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  info->K.assign(0.0);
  info->K[0] = info->K[4] = f;
  info->K[2] = (width / 2) - 0.5;
  info->K[5] = (width * (3./8.)) - 0.5;
  info->K[8] = 1.0;
  info->R.assign(0.0);
  info->R[0] = info->R[4] = info->R[8] = 1.0;
  info->P.assign(0.0);
  info->P[0]  = info->P[5] = f; 
  info->P[2]  = info->K[2];  
  info->P[6]  = info->K[5];   
  info->P[10] = 1.0;

  return info;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "openni2_xtion");
    ros::NodeHandle nh;

    ROS_INFO("1. Creating image_transport");
    image_transport::ImageTransport it(nh);

    image_transport::Publisher image_pub_depth = it.advertise("depth/image_raw", 1);
    ros::Publisher pub_depth_camera_info = nh.advertise<sensor_msgs::CameraInfo>("depth/camera_info", 1);

    string depth_info_url;
    nh.param("depth_camera_info_url", depth_info_url, string());

    boost::shared_ptr<camera_info_manager::CameraInfoManager> depth_info_manager_  = \
        boost::make_shared<camera_info_manager::CameraInfoManager>(nh,  "depth",  depth_info_url);

    try 
    {
        Status initStatus = OpenNI::initialize();
        if (initStatus != STATUS_OK)
        {
            ROS_ERROR("OpenNI initialize error : %s", OpenNI::getExtendedError());
            return -1;
        }

        openni::Device device;
        Status openStatus = device.open(ANY_DEVICE);
        while ( openStatus != STATUS_OK ) 
        {
            ROS_ERROR("Device open error %s", OpenNI::getExtendedError());
            return -1;
        }

        openni::VideoStream depthStream;
        depthStream.create(device, SENSOR_DEPTH);
        depthStream.start();

        openni::VideoStream** streams = new openni::VideoStream*[2];
        streams[0] = &depthStream;

        cv::Mat depthImage;

        cv_bridge::CvImagePtr cv_ptr_depth(new cv_bridge::CvImage);
	
        ROS_INFO("2. Start VideoStrem");

        while (ros::ok()) 
        {
            int changedIndex;

            ros::Time time = ros::Time::now();
				
            openni::VideoFrameRef depthFrame;
            depthStream.readFrame( &depthFrame);

            if ( depthFrame.isValid() ) 
            {
                depthImage = cv::Mat(depthStream.getVideoMode().getResolutionY(),
                                     depthStream.getVideoMode().getResolutionX(),
                                     CV_16U, (char*)depthFrame.getData());

                cv_ptr_depth->image = depthImage;
                cv_ptr_depth->encoding = "16UC1";
                cv_ptr_depth->header.frame_id = "/openni2_depth_frame";
                cv_ptr_depth->header.stamp = time;
                image_pub_depth.publish(cv_ptr_depth->toImageMsg());

                sensor_msgs::CameraInfoPtr info = getDefaultCameraInfo(640, 480, 570.3422241210938);
                info->K[2] -= 5; // cx
                info->K[5] -= 4; // cy
                info->P[2] -= 5; // cx
                info->P[6] -= 4; // cy

                info->header.stamp    = time;
                info->header.frame_id = "/openni2_depth_frame";

                pub_depth_camera_info.publish(info); 
			} 
            ros::spinOnce();
		}
	}
    catch ( std::exception& ) 
    {
        ROS_ERROR("exception error");
    }
    return 0;
}
