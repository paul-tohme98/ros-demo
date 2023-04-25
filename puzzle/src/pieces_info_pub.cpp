/*
**********************************
*	Author : PAUL TOHMEH
*	Date : April 25 2023
**********************************
*/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/imgcodecs.hpp"
#include <iostream>
#include <fcntl.h>
#include <linux/videodev2.h>

#include "puzzle/nb_pieces.h"
#include "puzzle/pieces_info.h"

using namespace cv;
using namespace std;

// Définition de la structure Piece
struct Piece {
    int rang;                // numéro de la pièce
    float x1;         // coordonnée x du coin supérieur gauche
    float y1;         // coordonnée y du coin supérieur gauche
    float x2;         // coordonnée x du coin inférieur droit
    float y2;         // coordonnée y du coin inférieur droit
    float x_centre;   // coordonnée du centre de la piece
    float y_centre;   // coordonnée du centre de la piece
    bool jouee = false;    // indique si la pièce a été jouée
    Mat piece_img; // Contient l image de la piece
};

// Fonction pour détecter les pièces de puzzle dans une image : publisher
void detecter_pieces(Mat image){
    // Creer le ROS handle necessaire pour transmettre les informations dans un topic
    ros::NodeHandle nh;
    // Creer les publishers qui seront responsable de la publication des messages
    ros::Publisher pub_nb = nh.advertise<puzzle::nb_pieces>("msg_nb_pieces", 10);
    ros::Publisher pub_pieces = nh.advertise<puzzle::pieces_info>("msg_pieces_info", 10);
    // Creer deux messages ROS , le premier est pour publier le nombre de pieces trouve dans l image 
    // Le deuxieme est pour publier les informations relatives a une piece avec son image
    puzzle::nb_pieces msg_nb_pieces;
    puzzle::pieces_info msg_pieces_info;

    while (ros::ok()){
        /*
         * La premiere partie de cette fonction consiste a traiter l image prise en argument 
         * afin d extraire les pieces presente dedans
        */
        
        // Convertir l'image en niveaux de gris
        Mat gray;
        cvtColor(image, gray, COLOR_BGR2GRAY);

        // Appliquer un flou gaussien pour réduire le bruit
        Mat blur;
        GaussianBlur(gray, blur, Size(5, 5), 0);

        // Appliquer une binarisation adaptative pour convertir l'image en noir et blanc
        Mat thresh;
        adaptiveThreshold(blur, thresh, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY_INV, 11, 2);

        // Trouver les contours dans l'image
        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;
        findContours(thresh, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Création d'une liste de pièces détectées
        vector<Piece> pieces;
        //cv_bridge::CvImage img_bridge;
        // Loop over contours to detect puzzle pieces
        vector<Mat> piece_images;
    	for (int i = 0; i < contours.size(); i++){
        	double area = contourArea(contours[i]);
        	if (area > 2900){
        	Piece piece;
            	// Detect piece corners
            	Rect rect = boundingRect(contours[i]);
            	//piece.rang = i + 1;
            	piece.x1 = rect.x;
            	piece.y1 = rect.y;
            	piece.x2= piece.x1 + rect.width;
            	piece.y2 = piece.y1 + rect.height;

            	// Calculate piece center
            	piece.x_centre = piece.x1 + rect.width / 2;
            	piece.y_centre = piece.y1 + rect.height / 2;
		
            	// Extract piece from original image and save it in its proper field
            	piece.piece_img = image(rect);
            	
            	// Add the piece to the vector of pieces
		pieces.push_back(piece);
        	}
    	}
    	
        /*
         * La deuxieme partie consiste a prendre les pieces stockes dans le vecteur de pieces rempli ci-dessus,
         * et les publier 
        */
        
        // Remplir le premier message : msg_nb_pieces
        msg_nb_pieces.nb = pieces.size();
        ROS_INFO("Nombre de pieces : nb = %d", msg_nb_pieces.nb);
        pub_nb.publish(msg_nb_pieces);
        
        for(int p = 0; p < pieces.size(); p++){
        	pieces[p].rang = p + pieces.size() + 1;
		sensor_msgs::Image img_msg;
		try{
		    // Convert OpenCV image to ROS image
  		    cv_bridge::CvImage img_bridge;
  		    std_msgs::Header header;
  		
  		    header.stamp = ros::Time::now(); // time stamp the header
  		    img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, pieces[p].piece_img);
  		    img_bridge.toImageMsg(img_msg); // from cv_bridge to ROS sensor_msgs/Image
  		    ROS_INFO("Image well saved!");
		}
	        catch (cv_bridge::Exception &e){
		    ROS_ERROR("cv_bridge exception: %s", e.what());
		    return;
		}
		

        	// Remplir le deuxieme message : msg_pieces_info
            	msg_pieces_info.rang = pieces[p].rang;
            	msg_pieces_info.x1 = pieces[p].x1;
            	msg_pieces_info.y1 = pieces[p].y1;
            	msg_pieces_info.x2 = pieces[p].x2;
            	msg_pieces_info.y2 = pieces[p].y2;
            	msg_pieces_info.x_centre = pieces[p].x_centre;
            	msg_pieces_info.y_centre = pieces[p].y_centre;
            	msg_pieces_info.jouee = pieces[p].jouee;
            	msg_pieces_info.image = img_msg;
            	
            	// Verifier que le message est bien transmit
            	ROS_INFO("La %deme piece", msg_pieces_info.rang);
            	ROS_INFO("x1 = %f", msg_pieces_info.x1);
            	ROS_INFO("y1 = %f", msg_pieces_info.y1);
            	ROS_INFO("x2 = %f", msg_pieces_info.x2);
            	ROS_INFO("y2 = %f", msg_pieces_info.y2);
            	ROS_INFO("x_centre = %f", msg_pieces_info.x_centre);
            	ROS_INFO("y_centre = %f", msg_pieces_info.y_centre);
            	ROS_INFO("jouee = %d", msg_pieces_info.jouee);
            	
            	pub_pieces.publish(msg_pieces_info);
            	if(p == pieces.size() - 1){
            		break;
            	}
        }
    }
    ros::spinOnce();
}

int main(int argc, char **argv){
    const char* video_device = "/dev/video"; // Base video device name
    int index = 0; // Camera index
    std::string device_path;
    
    // Try to open the first video device
    /*while (true) {
        device_path = video_device + std::to_string(index);
        int fd = open(device_path.c_str(), O_RDWR);
        if (fd == -1) {
            // Couldn't open this video device, so try the next one
            index++;
            continue;
        }
        // Successfully opened a video device, so use it and exit the loop
        std::cout << "Using video device " << device_path << std::endl;
        close(fd);
        break;
    }*/
	ros::init(argc, argv, "pieces_info_pub");// Node main function
	Mat img = cv::imread("/home/ros/catkin_ws/src/puzzle/images/pieces.jpeg");
	detecter_pieces(img);
	ros::spin();
	/*VideoCapture cap(index);
	if(!cap.isOpened()){
		cout << "Error opening video stream or file" << endl;
		return -1;
	}
	while(1){
	    Mat img;
            cap.read(img);
            if(img.empty()){
            	break;
            	}
	    detecter_pieces(img);
	    
	}*/
	//ros::spin();
	return 0;
}

