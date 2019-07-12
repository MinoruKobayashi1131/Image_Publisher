/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date February 2011
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include <ros/ros.h>
#include <ros/network.h>
#include <string>
#include <std_msgs/String.h>
#include <sstream>
#include "../include/pub_img/qnode.hpp"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>
#include <time.h>
#include <ros/network.h>
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/cuda.hpp>
#include <opencv2/core/utility.hpp>
//#include <opencv2/bgsegm.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/dnn.hpp>


/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace pub_img {

/*****************************************************************************
** Implementation
*****************************************************************************/

QNode::QNode(int argc, char** argv ) :
	init_argc(argc),
	init_argv(argv)
	{}

QNode::~QNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
	wait();
}

bool QNode::init() {
	ros::init(init_argc,init_argv,"pub_img");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  // Add your ros communications here.
  //chatter_publisher = n.advertise<std_msgs::String>("chatter", 1000);
  image_pub = it.advertise("image_data",10);

  start();
	return true;
}

bool QNode::init(const std::string &master_url, const std::string &host_url) {
	std::map<std::string,std::string> remappings;
	remappings["__master"] = master_url;
	remappings["__hostname"] = host_url;
	ros::init(remappings,"pub_img");
	if ( ! ros::master::check() ) {
		return false;
	}
	ros::start(); // explicitly needed since our nodehandle is going out of scope.
	ros::NodeHandle n;
	// Add your ros communications here.
	start();
	return true;
}

void QNode::run() {
  ros::Rate loop_rate(30);
	int count = 0;
  movie_name = "";
  cv::Mat image;
  bool opened = false;
  cv::VideoCapture cap;

  while ( ros::ok() )
  {
    if((movie_name != "") && (opened == false))
    {
      cv::String fname = movie_name.toStdString();
      cap.open(fname);
      if(cap.isOpened())
        opened = true;
    }

    if(opened == true)
    {
      bool ret = cap.read(image);
      if(ret == true)
      {
        sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
        image_pub.publish(msg);
        video_buf = image.clone();
        cv::resize(video_buf,video_buf,cv::Size(640,480));
        Q_EMIT videoUpdated();

      }
      else
      {
        cap.set(cv::CAP_PROP_POS_FRAMES,0);
      }
    }

		ros::spinOnce();
		loop_rate.sleep();
		++count;
	}
	std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
	Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}



}  // namespace pub_img
