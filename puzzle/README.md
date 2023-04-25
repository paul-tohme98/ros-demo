#####################################
#	Author : PAUL TOHMEH
#	Date : April 25 2023
#####################################

This is a demo of a ROS communcation between two nodes (publisher and subscriber) where the publisher takes an image containing
puzzle pieces and treats it using opencv library in order to detect the number of pieces present. It then publishes this information alongside another containing the coordinates of each piece detected relatively to the image alongside the piece's image cropped from the original image and sent with the coordinates.

This is one task taken from my university project that was programming a niryo robot to solve a puzzle.
It is done with C++, opencv library and ROS communication to send the data collected as messages.

The messages are customized, their format is present in the folder : msg
The source code is in the folder : src
The image is in the folder : image

To run the demo you should build the project with the command :
$ catkin puzzle

Then open 3 terminals, in the first one run the following command in order to start the ROS master :
$ roscore

In the second terminal run the following command to run the publisher node :
$ rosrun puzzle pieces_info_pub

In the third terminal run the following command to run the subscriber node :
$ rosrun puzzle pieces_info_sub

You should be able to see the data transfered between the two nodes, also you will be able to visualize the pieces in the subscriber node.

