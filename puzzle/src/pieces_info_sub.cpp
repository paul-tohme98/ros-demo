/*
**********************************
*	Author : PAUL TOHMEH
*	Date : April 25 2023
**********************************
*/


#include "ros/ros.h"
#include <sensor_msgs/Image.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>

#include "puzzle/nb_pieces.h"
#include "puzzle/pieces_info.h"

// Fonction qui nous permet de verifier que le contenu du premier message est bien transmit
void msgNbPieces(const puzzle::nb_pieces::ConstPtr& msg){
    ROS_INFO("Recieved nb = %d", msg->nb);    
}

// Fonction qui nous permet de verifier que le contenu du deuxieme message est bien transmit
void msgPiecesInfo(const puzzle::pieces_info::ConstPtr& msg) {
  // Print the received message
  ROS_INFO("Received message: rang=%d, x1=%.2f, y1=%.2f, x2=%.2f, y2=%.2f, x_centre=%.2f, y_centre=%.2f, jouee=%d",
           msg->rang, msg->x1, msg->y1, msg->x2, msg->y2, msg->x_centre, msg->y_centre, msg->jouee);

  // Convert sensor_msgs/Image to OpenCV image
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg->image, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Display the OpenCV image
  cv::imshow("Image", cv_ptr->image);
  cv::waitKey(1);
}

// Ce subscriber est utilise pour la demonstration de la communication ROS entre publisher et subscriber 
int main(int argc, char **argv){
    ros::init(argc, argv, "puzzle_node_subscriber");// Node main function
    ros::NodeHandle nh;// Declare a node handle to communicate with the ROS system
    ros::Subscriber subscriber_nb_pieces = nh.subscribe("msg_nb_pieces", 100, msgNbPieces);
    ros::Subscriber subscriber_pieces_info = nh.subscribe("msg_pieces_info", 100, msgPiecesInfo);
    
    ros::spin();
    return 0;
}

