#include <stdio.h>
//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <math.h>
#include <sys/select.h>
#include <termios.h>
//#include <stropts.h>
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

#define MAP_SIZE 4096UL
#define MAP_MASK (MAP_SIZE - 1)

#include <time.h>
#include <memory>
using namespace cv;

extern "C"
{
#include "serial_dev.h"
}

using namespace cv;
using namespace std;

union float_char
{
    float val[2];
    char bytes[8];
};

#define SHOW_IMAGE 
char SERFILE[] = "/dev/ttyAMA1";
#define SERIALBUFFSIZE 1024
float cx; //position of the center(in percentage)
float cy; //position of the center(in percentage)
union float_char to_send;
union float_char received;
int frame_count = 0;
int max_index=0;
float x,y;
float size = 0;
std::vector<cv::KeyPoint> keypoints;
int charsread = 0;

int fd;  // variables used to map to physical memory addresses
void *map_base;
off_t target;
int32_t *myuart2;



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
    	/* ****************************************************************/
	/*  Memory map to physical memory spaces of the UART2 Registers*/	
	if((fd = open("/dev/mem", O_RDWR | O_SYNC)) == -1) {
		printf("/dev/mem could not be opened.\n");
		exit(1);
	} else {
		printf("/dev/mem opened.\n");
	}
	fflush(stdout);
	
	//target = 0x7e201400;  //UART2 of Raspberry pi 4.0 only  !!!!!
	target = 0xFE201000;
	//target = 0x7e200000;
	/* Map one page for shared memory structure*/
	map_base = mmap(0, MAP_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, fd, target);
	if(map_base == (void *) -1) {
		printf("Memory map failed.\n");
		exit(1);
	} else {
		printf("gpio Struct mapped at address %p.\n", map_base);
	}
	fflush(stdout);
	myuart2 = (int32_t *) map_base;
	
	printf("\n\n\n\nIntDiv=%d,FracDiv=%d\n\n\n\n",myuart2[265],myuart2[266]);

	


	to_send.val[0] = 0.1234;
    to_send.val[1] = 0.4567;
    printf("Initializing serial port driver %s...\n", "/dev/ttyAMA1");
    setup_serial();
    printf("...OK\n");

    //sd_set_blocking();
    printf(".\n");
    sd_ioflush();

    myuart2[256+12] &= ~0x301; // Clear enable bits
    usleep(100000);
    myuart2[256+11] &= ~0x10;  // clear FIFO    
    usleep(100000);
    printf("%d,%d\n",0,myuart2[256+0]);
    printf("%d,%d\n",1,myuart2[256+1]);
    printf("%d,%d\n",6,myuart2[256+6]);
    printf("%d,%d\n",8,myuart2[256+8]);
    printf("%d,%d\n",9,myuart2[256+9]);
    printf("%d,%d\n",10,myuart2[256+10]);
    printf("%d,%d\n",11,myuart2[256+11]);
    printf("%d,%d\n",12,myuart2[256+12]);
    printf("%d,%d\n",13,myuart2[256+13]);
    printf("%d,%d\n",14,myuart2[256+14]);
    printf("%d,%d\n",15,myuart2[256+15]);
    printf("%d,%d\n",16,myuart2[256+16]);
    printf("%d,%d\n",17,myuart2[256+17]);
    printf("%d,%d\n",18,myuart2[256+18]);
    printf("%d,%d\n",32,myuart2[256+32]);
    printf("%d,%d\n",33,myuart2[256+33]);
    printf("%d,%d\n",34,myuart2[256+34]);
    printf("%d,%d\n",35,myuart2[256+35]);
    usleep(100000);
	myuart2[256+9] = 1;
	myuart2[256+10] = 28;//59;  // 28 is 2,083,333  59 is 1,562,500
    usleep(100000);
    myuart2[256+11] |= 0x10;  //enable fifo
    usleep(100000);
    myuart2[256+12] |= 0x301; // Set enable bits
    usleep(100000);
    

    printf("%d,%d\n",0,myuart2[256+0]);
    printf("%d,%d\n",1,myuart2[256+1]);
    printf("%d,%d\n",6,myuart2[256+6]);
    printf("%d,%d\n",8,myuart2[256+8]);
    printf("%d,%d\n",9,myuart2[256+9]);
    printf("%d,%d\n",10,myuart2[256+10]);
    printf("%d,%d\n",11,myuart2[256+11]);
    printf("%d,%d\n",12,myuart2[256+12]);
    printf("%d,%d\n",13,myuart2[256+13]);
    printf("%d,%d\n",14,myuart2[256+14]);
    printf("%d,%d\n",15,myuart2[256+15]);
    printf("%d,%d\n",16,myuart2[256+16]);
    printf("%d,%d\n",17,myuart2[256+17]);
    printf("%d,%d\n",18,myuart2[256+18]);
    printf("%d,%d\n",32,myuart2[256+32]);
    printf("%d,%d\n",33,myuart2[256+33]);
    printf("%d,%d\n",34,myuart2[256+34]);
    printf("%d,%d\n",35,myuart2[256+35]);




	//myuart2[265] = 3;
	//myuart2[266] = 0;//59;
	
	
	printf("\n\n\n\nIntDiv=%d,FracDiv=%d\n\n\n\n",myuart2[265],myuart2[266]);
	//printf("\n\n\n\nIntDiv=%d,FracDiv=%d\n\n\n\n",myuart2[9],myuart2[10]);
	// exit(1);



    printf("Starting\n");
    VideoCapture cap(2);
    cap.set(CAP_PROP_FRAME_WIDTH,320);
    cap.set(CAP_PROP_FRAME_HEIGHT,240);
    Mat img;
    //below value selected with trackbar
    int hmin = 45, smin = 52, vmin = 47;
    int hmax = 106, smax = 255, vmax = 255;

    // //create trackbar to select values, hue max is 179, saturation and value are 255
#ifdef SHOW_IMAGE
    namedWindow("Trackbars", (640, 200));
    createTrackbar("Hue Min", "Trackbars", &hmin, 179);
    createTrackbar("Hue Max", "Trackbars", &hmax, 179);
    createTrackbar("Sat Min", "Trackbars", &smin, 255);
    createTrackbar("Sat Max", "Trackbars", &smax, 255);
    createTrackbar("Val Min", "Trackbars", &vmin, 255);
    createTrackbar("Val Max", "Trackbars", &vmax, 255);
#endif

	cv::SimpleBlobDetector::Params params;
	params.minDistBetweenBlobs = 40.0f;
	params.filterByInertia = false;
	params.filterByConvexity = false;
	params.filterByColor = false;
	params.filterByCircularity = false;
	params.filterByArea = true;
	params.minArea = 2000.0f;
	params.maxArea = 100000.0f;
    Mat frame, frame_HSV, frame_threshold, frame_key;
    Ptr<SimpleBlobDetector> blob_detector = SimpleBlobDetector::create(params);
    while (true)
    {
        frame_count++;
		printf("Framecount = %d\n");
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
        for (int i=0; i<keypoints.size(); i++){
			x = keypoints[i].pt.x; 
			y = keypoints[i].pt.y;
            size = keypoints[i].size;
            if (size > keypoints[max_index].size) {
                max_index = i;
            }
            printf("keypoints %d ,x %.3f, y %.3f, size %f \n",keypoints.size(),x,y,size);

		}
        if(keypoints.size()>0) {
            to_send.val[0] = keypoints[max_index].pt.x;
            to_send.val[1] = keypoints[max_index].pt.y;
            sd_write(asterisk);
            sd_write(asterisk);
            sd_writen(to_send.bytes,8);
            //usleep(5000);
            sd_write(exclamation);
            sd_write(exclamation);
            //usleep(5000);
            charsread = sd_readn(received.bytes,8);
			if (charsread != 8) {
				printf("charsread = %d\n",charsread);
				while(sd_readn(received.bytes,8) > 0) {} 
			} else {
				printf("value1 = %.3f value2 = %.3f\n",received.val[0],received.val[1]);
			}

printf("\n\n\n\nIntDiv=%d,FracDiv=%d\n\n\n\n",myuart2[265],myuart2[266]);

        }
        // Show the frames
#ifdef SHOW_IMAGE
        // imshow("image", frame);
        imshow("mask", frame_threshold);
        imshow("mark", frame_key);
#endif
        waitKey(1);
        char key = (char)waitKey(30);
        frame_count++;
        //if (frame_count >= 8) {
        //     break;
        //}
        //break;
        if (key == 'q' || key == 27)
        {
            break;
        }
    }
    return 0;
}
