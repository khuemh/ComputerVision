#include <iostream>
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;


// Create HSV value
struct HSV_Val
{
        int H_max;
        int H_min;
        int S_max;
        int S_min;
        int V_max;
        int V_min;
};

struct HSV_Val hsv;

// Create HSV Trackbar function
void Create_HSV_Trackbar()
{
        // Create control WINDOW
        const char* ctrl_window = "HSV CONTROL";
        namedWindow(ctrl_window, CV_WINDOW_AUTOSIZE);

        cvCreateTrackbar("Low H", ctrl_window, &hsv.H_min, 179);
        cvCreateTrackbar("High H", ctrl_window, &hsv.H_max, 179);
        cvCreateTrackbar("Low S", ctrl_window, &hsv.S_min, 255);
        cvCreateTrackbar("High S", ctrl_window, &hsv.S_max, 255);
        cvCreateTrackbar("Low V", ctrl_window, &hsv.V_min, 255);
        cvCreateTrackbar("High V", ctrl_window, &hsv.V_max, 255);
}


/* 
    MAIN FUNCTION
*/
int main()
{
        printf("Hello world!!!\n");
        // Get video from CAM
        VideoCapture cap(0);
        if (cap.isOpened() == false)
        {
                cout << "ERROR" << endl;
                cin.get();
                return -1;
        }
        double dWidth = cap.get(CAP_PROP_FRAME_WIDTH);
        double dHeight = cap.get(CAP_PROP_FRAME_HEIGHT);

        cout << "Resolution of the video: " << dWidth << " x " << dHeight << endl;

        // Create main WINDOW
        String main_window = "Cam FEED";
        namedWindow(main_window);
        
        // Create HSV trackbar
        Create_HSV_Trackbar();
        
        while(cap.isOpened())
        {
                Mat frameORG;
                bool isSuccess = cap.read(frameORG);
                if(isSuccess == false)
                {
                        cout << "Camera was disconnected" << endl;
                        cin.get();
                        break;
                }

                // Flip the frame
                flip(frameORG, frameORG, 1);

                // Convert BGR to HSV
                Mat frameHSV;
                cvtColor(frameORG, frameHSV, COLOR_BGR2HSV);

                // Theshold the frame
                Mat frameThresh;
                inRange(frameHSV, Scalar(hsv.H_min, hsv.S_min, hsv.V_min), Scalar(hsv.H_max, hsv.S_max, hsv.V_max), frameThresh);

                // Morphological Opening
                erode(frameThresh, frameThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
                dilate(frameThresh, frameThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

                // Morphological Closing
                dilate(frameThresh, frameThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));
                erode(frameThresh, frameThresh, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)));

                // Show result frame
                imshow(main_window, frameThresh);

                // Press ESC to STOP
                if(waitKey(10) == 27)
                {
                        cout << "Stop Stream" << endl;
                        break;
                }              
        }
        return 0;
}
