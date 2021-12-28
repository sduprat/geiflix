#include "../include/boule_de_cristal/image_node.hpp"
#include <ros/master.h>
#include <ros/network.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <string>

namespace boule_de_cristal {

ImageNode::ImageNode() : it(nh) {
    camera_enabled = false ;
    image_sub_ = it.subscribe("/usb_cam/image_raw", 1, &ImageNode::imageCb, this);
}

ImageNode::~ImageNode() {
  cv::destroyWindow("Camera");
}

void ImageNode::showCamera() {
  cv::namedWindow("Camera");
  camera_enabled = true ;
  ros::Rate r(10) ;
  while (camera_enabled)
  {
    ros::spinOnce() ;
    r.sleep() ;
  }
}

void ImageNode::hideCamera() {
  camera_enabled = false ;
  cv::destroyWindow("Camera");
}


void ImageNode::imageCb(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Update GUI Window
  cv::imshow("Camera", cv_ptr->image);
  cv::waitKey(30);
}

};  // namespace boule_de_cristal

