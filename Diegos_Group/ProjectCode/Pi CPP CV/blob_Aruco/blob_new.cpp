#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <math.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdlib.h>
#include <stdint.h>
#include <sys/mman.h>
#include <linux/fb.h>
#include <string.h>
#include <errno.h>
#include <signal.h>
#include <ctype.h>
#include <sys/types.h>

// aruco detector
#include <opencv2/aruco.hpp>
#include <iostream>
#include <cstdlib>


#include <time.h>
#include <memory>


extern "C"
{
#include "serial_dev.h"
}

using namespace cv;
using namespace std;
namespace {// namespace for Aruco
const char* about = "Pose estimation of ArUco marker images";
const char* keys  =
        "{d        |16    | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, "
        "DICT_4X4_250=2, DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, "
        "DICT_5X5_250=6, DICT_5X5_1000=7, DICT_6X6_50=8, DICT_6X6_100=9, "
        "DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12, DICT_7X7_100=13, "
        "DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
        "{h        |false | Print help }"
        "{l        |      | Actual marker length in meter }"
        "{v        |<none>| Custom video source, otherwise '0' }"
        "{h        |false | Print help }"
        "{l        |      | Actual marker length in meter }"
        "{v        |<none>| Custom video source, otherwise '0' }"
        ;
}

union float_char
{
    float val[4];  // number of floats to send
    char bytes[16];  // 4 * the number of floats to send
};

#define SHOW_IMAGE 
char SERFILE[] = "/dev/ttyAMA1";
#define SERIALBUFFSIZE 1024
union float_char to_send;
union float_char received;
int charsread = 0;

int frame_count = 0;

// blob detection
int max_index=0;
float x,y;
//float xi[8] = {0,0,0,0, 0,0,0,0}; //saving the past 8 values of x for noise reduction (not used)
float size = 0;
std::vector<cv::KeyPoint> keypoints;

// variables for Aruco
int dictionaryId = 0;
float marker_length_m = 0.032;// side of length of a single marker (meters)

// physical variables
float distance2marker = 0;
int32_t ball_found = 0;
int32_t marker_found = 0;
int32_t wall = 0; // wall = 1 when too close to a wall
float marker_x = 0; //position of the marker center(in pixels)
float marker_y = 0;
float distance2ball = 0;
float cx; //position of the blob center(in pixels)
float cy;

// debugging tool for red board
//float K1 = -85;
//float K2 = -4.7;
//float K3 = -1.2;
//float K4 = -0.12;


/*
 * setup_serial()
 *   sets the serial port up at 115200 baud
 */
void setup_serial()
{
    sd_setup(SERFILE); //starts non-blocking
    sd_ioflush();
}

/////////////// Color Detection //////////////////////
char asterisk[] = "*";
char exclamation[] = "!";

int main()
{
	// to_send.val[0] = 0.1234;
    // to_send.val[1] = 0.4567;
    printf("Initializing serial port driver %s...\n", "/dev/ttyAMA1");
    setup_serial();
    printf("...OK\n");

    //sd_set_blocking();
    printf(".\n");
    sd_ioflush();


    // initiate camera capture
    printf("Starting\n");
    VideoCapture cap(0);
    cap.set(CAP_PROP_FRAME_WIDTH,320);//160
    cap.set(CAP_PROP_FRAME_HEIGHT,240);//120
    Mat img;

    //default value for trackbars (hsv threshold)
    // int hmin = 111, smin = 25, vmin = 35; //values for purple
    // int hmax = 133, smax = 124, vmax = 255;
    int hmin = 33, smin = 60, vmin = 15; //values for green
    int hmax = 89, smax = 249, vmax = 255;
    


    // create trackbar to select values, hue max is 179, saturation and value are 255
#ifdef SHOW_IMAGE
    namedWindow("Trackbars", (640, 200));
    createTrackbar("Hue Min", "Trackbars", &hmin, 179);
    createTrackbar("Hue Max", "Trackbars", &hmax, 179);
    createTrackbar("Sat Min", "Trackbars", &smin, 255);
    createTrackbar("Sat Max", "Trackbars", &smax, 255);
    createTrackbar("Val Min", "Trackbars", &vmin, 255);
    createTrackbar("Val Max", "Trackbars", &vmax, 255);
#endif

    //params for simpleblobdetector (setup for golf ball)
	cv::SimpleBlobDetector::Params params;
	params.minDistBetweenBlobs = 40.0f;
	params.filterByInertia = false;
    params.minInertiaRatio = 0.7;
    params.maxInertiaRatio = 1;
	params.filterByConvexity = false;
	params.filterByColor = false;
	params.filterByCircularity = true;
    params.minCircularity = 0.5;
    params.maxCircularity = 1;
	params.filterByArea = true;
	params.minArea = 64.0f; // 8 pixels in image
	params.maxArea = 100000.0f;
    Mat frame, frame_HSV, frame_threshold, frame_key, frame_single_blob;
    Ptr<SimpleBlobDetector> blob_detector = SimpleBlobDetector::create(params);

    //Aruco
    Mat frame_Aruco, camera_matrix, dist_coeffs;
    ostringstream vector_to_marker;
    Ptr<cv::aruco::Dictionary> dictionary =
        cv::aruco::getPredefinedDictionary( \
        cv::aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));

    cv::FileStorage fs("calibration_params.yml", cv::FileStorage::READ); //calibration file

    fs["camera_matrix"] >> camera_matrix;
    fs["distortion_coefficients"] >> dist_coeffs;

    std::cout << "camera_matrix\n" << camera_matrix << std::endl;
    std::cout << "\ndist coeffs\n" << dist_coeffs << std::endl;

    //Wall
    Mat frame_wall;


    // main while loop
    while (true)
    {
        frame_count++;
		// printf("Framecount = %d\n",frame_count);
        cap >> frame;
        if (frame.empty())
        {
            break;
        }
        // Convert from BGR to HSV colorspace
        cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        // // Detect the object based on HSV Range Values
        Scalar lower(hmin, smin, vmin); //set lower bound
        Scalar upper(hmax, smax, vmax); //set upper bound
        inRange(frame_HSV, lower, upper, frame_threshold);
        blob_detector->detect(frame_threshold, keypoints);
#ifdef SHOW_IMAGE
        drawKeypoints(frame,keypoints,frame_key,Scalar::all(-1),DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
#endif
        //find the keypoint with the largest y (closest to the ground)
        for (int i=0; i<keypoints.size(); i++){
			x = keypoints[i].pt.x; 
			y = keypoints[i].pt.y;
            size = keypoints[i].size;
            if (y > keypoints[max_index].pt.y) {//finding the lowest keypoint
                max_index = i;
            }
            // printf("keypoints %d ,x %.3f, y %.3f, size %f \n",keypoints.size(),x,y,size);
		}

        frame.copyTo(frame_single_blob);
        // cout<<"size:"<<keypoints[max_index].size<<endl;


        // when one or more keypoints are found
        if(keypoints.size()>0) {
            ball_found=1;
            // anti-flickering (calculate the Mean sqaure variance of the past 8 x-coordinate of the keypoint)
            // used when using purple ball, disabled when using green ball
            x = keypoints[max_index].pt.x;
            y = keypoints[max_index].pt.y;
            size = keypoints[max_index].size;
//            cout<<"size: "<<size<<endl;
            // xi[0] = x;//save past x
            // for (int i=7; i>0;i--){
            //     xi[i]=xi[i-1];
            // }
            // float stdv_x = 0;
            // float sum_x = 0;
            // for (int i=0;i<8;i++){
            //     sum_x = sum_x +xi[i];
            // }
            // sum_x = sum_x/8;
            // for (int i=0;i<8;i++){
            //     stdv_x = stdv_x+(xi[i]-sum_x)*(xi[i]-sum_x);
            // }
            // stdv_x = stdv_x/8;
            // // cout<<"stdv_x"<<stdv_x<<endl;

            // if (stdv_x<1000){
            //     ball_found = 1;
            // }else{
            //     ball_found = 0;
            // }

            //UART
            to_send.val[0] = keypoints[max_index].pt.x;
            to_send.val[1] = keypoints[max_index].pt.y;
            to_send.val[2] = ball_found;
            to_send.val[3] = wall;
            to_send.val[4] = marker_x;
            to_send.val[5] = marker_y;
            to_send.val[6] = marker_found;
            to_send.val[7] = distance2marker;

            sd_write(asterisk);
            sd_write(asterisk);
            sd_writen(to_send.bytes,32);  // number is 4 * number of floats
            cout<<"ball_found:"<<ball_found<<endl;
            cout<<"marker_found:"<<marker_found<<endl;
            cout<<"marker_x:"<<marker_x<<endl;
            cout<<"marker_y:"<<marker_y<<endl;
            cout<<"wall: "<<wall<<endl;
            cout<<"wall intensity:"<<cv::countNonZero(frame_wall)<<endl;


            // cout<<"marker: "<<to_send.val[6]<<endl;
            // cout<<"y: "<<to_send.val[1]<<endl;
        }else{//no keypoints found
            ball_found=0;
            //UART
            to_send.val[0] = 160;
            to_send.val[1] = 0;
            to_send.val[2] = ball_found;
            to_send.val[3] = wall;
            to_send.val[4] = marker_x;
            to_send.val[5] = marker_y;
            to_send.val[6] = marker_found;
            to_send.val[7] = distance2marker;

            sd_write(asterisk);
            sd_write(asterisk);
            sd_writen(to_send.bytes,32);  // number is 4 * number of floats
            cout<<"ball_found:"<<ball_found<<endl;
            cout<<"marker_found:"<<marker_found<<endl;
            cout<<"marker_x:"<<marker_x<<endl;
            cout<<"marker_y:"<<marker_y<<endl;
            cout<<"wall: "<<wall<<endl;
            cout<<"wall intensity:"<<cv::countNonZero(frame_wall)<<endl;

        }

        // draw the found golf ball
        if(ball_found==1){        
        Point center(x, y);//Declaring the center point
        int radius = (int)(size/2); //Declaring the radius
        Scalar line_Color(0, 0, 0);//Color of the circle
        int thickness = 2;//thickens of the line
        // frame.copyTo(frame_single_blob);//moved elsewhere
        circle(frame_single_blob, center,radius, line_Color, thickness);
        }

        //Aruco
        frame.copyTo(frame_Aruco);
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f>> corners;
        cv::aruco::detectMarkers(frame, dictionary, corners, ids);
        
        marker_found=0;//reset
        // If at least one marker detected
        if (ids.size() > 0)
        {
            //draw all detected markers
            cv::aruco::drawDetectedMarkers(frame_Aruco, corners, ids);
            //pose estimation
            std::vector<cv::Vec3d> rvecs, tvecs; //rotational and translation vector
            cv::aruco::estimatePoseSingleMarkers(corners, marker_length_m,
                    camera_matrix, dist_coeffs, rvecs, tvecs);     
            // std::cout << "Translation: " << tvecs[0]
            //     << "\tRotation: " << rvecs[0]
            //     << std::endl;
            
            // Find marker with particular id
            for(int i=0; i < ids.size(); i++)
            {
                // cout << "id " << ids[i] << endl;
                if(ids[i]==18){


                    //calculate center of marker
                    marker_found = 1;
                    marker_x = (corners[i].at(0).x + corners[i].at(1).x + corners[i].at(2).x + corners[i].at(3).x)/4;
                    marker_y = (corners[i].at(0).y+ corners[i].at(1).y + corners[i].at(2).y + corners[i].at(3).y)/4;
                    // cout << "marker_x,marker_y " << marker_x <<", " << marker_y << endl;

                    //calculate distance to marker
                    distance2marker =sqrt(tvecs[i][0]*tvecs[i][0] + tvecs[i][1]*tvecs[i][1] + tvecs[i][2]*tvecs[i][2]);
                    // cout << "distance:  " << distance2marker <<", " << distance2ball << endl;

                    //draw the axis, translation and rotation information
                    cv::aruco::drawAxis(frame_Aruco, camera_matrix, dist_coeffs,
                            rvecs[i], tvecs[i], 0.1);

                    vector_to_marker.str(std::string());
                    vector_to_marker << std::setprecision(4)
                                    << "x: " << std::setw(8) << tvecs[0](0);
                    cv::putText(frame_Aruco, vector_to_marker.str(),
                                cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                cv::Scalar(0, 252, 124), 1, CV_AVX);

                    vector_to_marker.str(std::string());
                    vector_to_marker << std::setprecision(4)
                                    << "y: " << std::setw(8) << tvecs[0](1);
                    cv::putText(frame_Aruco, vector_to_marker.str(),
                                cv::Point(10, 50), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                cv::Scalar(0, 252, 124), 1, CV_AVX);

                    vector_to_marker.str(std::string());
                    vector_to_marker << std::setprecision(4)
                                    << "z: " << std::setw(8) << tvecs[0](2);
                    cv::putText(frame_Aruco, vector_to_marker.str(),
                                cv::Point(10, 70), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                cv::Scalar(0, 252, 124), 1, CV_AVX);

                    vector_to_marker.str(std::string());
                    vector_to_marker << std::setprecision(4)
                                    << "rotx: " << std::setw(8) << rvecs[0](0);
                    cv::putText(frame_Aruco, vector_to_marker.str(),
                                cv::Point(10, 90), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                cv::Scalar(0, 252, 140), 1.5, CV_AVX);
                                                vector_to_marker.str(std::string());

                    vector_to_marker << std::setprecision(4)
                                    << "roty: " << std::setw(8) << rvecs[0](1);
                    cv::putText(frame_Aruco, vector_to_marker.str(),
                                cv::Point(10, 110), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                cv::Scalar(0, 252, 140), 1.5, CV_AVX);
                                                vector_to_marker.str(std::string());

                    vector_to_marker << std::setprecision(4)
                                    << "rotz: " << std::setw(8) << rvecs[0](2);
                    cv::putText(frame_Aruco, vector_to_marker.str(),
                                cv::Point(10, 130), cv::FONT_HERSHEY_SIMPLEX, 0.6,
                                cv::Scalar(0, 252, 140), 1.5, CV_AVX);
                }

            }
        }

        // Wall
        Scalar lower2(0, 80, 1); //set lower bound
        Scalar upper2(34, 240, 255); //set upper bound
        inRange(frame_HSV, lower2, upper2, frame_wall);//color filter for wall(orange)

        // cout<<"wall intensity:"<<cv::countNonZero(frame_wall)<<endl;
        if(countNonZero(frame_wall)>65000){
            wall = 1;
        }else{
            wall = 0;
        }


        // Show the frames
#ifdef SHOW_IMAGE
        // imshow("image", frame);
        // imshow("mask", frame_threshold);

        // namedWindow("Trackbars", (640, 200));
        imshow("Trackbars",frame_threshold);

        // namedWindow( "keypoints", WINDOW_NORMAL);
        // resizeWindow("keypoints",500,500);
        // imshow("keypoints", frame_key);

        namedWindow( "Detected markers", WINDOW_NORMAL);//create window
        resizeWindow("Detected markers",500,500);//set window size
        imshow("Detected markers", frame_Aruco);//show image

        namedWindow( "Detected ball", WINDOW_NORMAL);
        resizeWindow("Detected ball",500,500);
        imshow("Detected ball", frame_single_blob);

        namedWindow( "Wall Detection", WINDOW_NORMAL);
        resizeWindow("Wall Detection",500,500);
        imshow("Wall Detection", frame_wall);

        usleep(50000);// 50ms delay
#endif
        // exit main loop
        waitKey(1);
        char key = (char)waitKey(30);
        frame_count++;
        if (key == 'q' || key == 27)
        {
            break;
        }
    }

    cap.release();
    return 0;
}
