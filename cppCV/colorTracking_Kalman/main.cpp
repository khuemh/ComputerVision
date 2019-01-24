/******************************************
 * Ball Tracking using                    *
 * Kalman Filter                          *
 ******************************************/

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;


/*
    Global variables declaration
*/
#define CAM_INDEX               0           // Select camera device
#define MIN_H_BLUE              200         // Color to be 
#define MAX_H_BLUE              300         // tracked


/*
    Functions Declaration
*/
Mat img_processing(Mat frame_in);

void end_prog();


/*
    MAIN FUNCTION
*/
int main()
{
    // >>>> Create Kalman Filter
    int stateSize = 6;
    int measSize = 4;
    int contrSize = 0;
    unsigned int type = CV_32F;

    KalmanFilter kf(stateSize, measSize, contrSize, type);

    Mat state(stateSize, 1, type);  // [x,y,v_x,v_y,w,h]
    Mat meas(measSize, 1, type);    // [z_x,z_y,z_w,z_h]
    //cv::Mat procNoise(stateSize, 1, type)
    // [E_x,E_y,E_v_x,E_v_y,E_w,E_h]

    /* Transition State Matrix A
    Note: set dT at each processing step!
        [ 1 0 dT 0  0 0 ]
        [ 0 1 0  dT 0 0 ]
        [ 0 0 1  0  0 0 ]
        [ 0 0 0  1  0 0 ]
        [ 0 0 0  0  1 0 ]
        [ 0 0 0  0  0 1 ] */
    setIdentity(kf.transitionMatrix);

    /* Measure Matrix H
        [ 1 0 0 0 0 0 ]
        [ 0 1 0 0 0 0 ]
        [ 0 0 0 0 1 0 ]
        [ 0 0 0 0 0 1 ] */
    kf.measurementMatrix = Mat::zeros(measSize, stateSize, type);
    kf.measurementMatrix.at<float>(0) = 1.0f;
    kf.measurementMatrix.at<float>(7) = 1.0f;
    kf.measurementMatrix.at<float>(16) = 1.0f;
    kf.measurementMatrix.at<float>(23) = 1.0f;

    /* Process Noise Covariance Matrix Q
        [ Ex   0   0     0     0    0  ]
        [ 0    Ey  0     0     0    0  ]
        [ 0    0   Ev_x  0     0    0  ]
        [ 0    0   0     Ev_y  0    0  ]
        [ 0    0   0     0     Ew   0  ]
        [ 0    0   0     0     0    Eh ] */
    //cv::setIdentity(kf.processNoiseCov, cv::Scalar(1e-2));
    kf.processNoiseCov.at<float>(0) = 1e-2;
    kf.processNoiseCov.at<float>(7) = 1e-2;
    kf.processNoiseCov.at<float>(14) = 5.0f;
    kf.processNoiseCov.at<float>(21) = 5.0f;
    kf.processNoiseCov.at<float>(28) = 1e-2;
    kf.processNoiseCov.at<float>(35) = 1e-2;

    // Measures Noise Covariance Matrix R
    setIdentity(kf.measurementNoiseCov, cv::Scalar(1e-1));
    
    // >>>> Camera settings
    VideoCapture cap(CAM_INDEX);
    if (cap.isOpened() == false)
    {
        cout << "Webcam not connected.\n" << "Please verify\n";
        return EXIT_FAILURE;
    }
    cout << "\nHit ESC to exit...\n";
    

    double ticks = 0;
    bool found = false;
    int notFoundCount = 0;
    
    // >>>>> MAIN LOOP
    while (cap.isOpened())
    {
        /* dT calculation */
        double precTick = ticks;
        ticks = (double)getTickCount();
        double dT = (ticks - precTick) / getTickFrequency(); //seconds

        /* Frame acquisition */
        Mat frame;
        bool isSuccess = cap.read(frame);
        if(isSuccess == false)
        {
            cout << "Camera was disconnected" << endl;
            cin.get();
            break;
        }
        flip(frame, frame, 1);

        /* KALMAN Prediction Phase */
        Mat res;
        frame.copyTo(res);
        if (found)
        {
            // >>>> Matrix A
            kf.transitionMatrix.at<float>(2) = dT;
            kf.transitionMatrix.at<float>(9) = dT;
            // <<<< Matrix A

            //cout << "dT:" << endl << dT << endl;

            state = kf.predict(); // [x,y,v_x,v_y,w,h]
            //cout << "State post:" << endl << state << endl;

            Rect predRect; // Create prediction rectangle
            predRect.width = state.at<float>(4);
            predRect.height = state.at<float>(5);
            predRect.x = state.at<float>(0) - predRect.width / 2;
            predRect.y = state.at<float>(1) - predRect.height / 2;

            Point center;
            center.x = state.at<float>(0);
            center.y = state.at<float>(1);
            circle(res, center, 2, CV_RGB(255,0,0), -1);
            // Draw prediction Rectangle
            rectangle(res, predRect, CV_RGB(255,0,0), 2);
        }
        
        /* Image Processing */
        Mat rangeRes = img_processing(frame);
        
        /* Contours detection */
        vector < vector<Point> > contours; // vector of many contours
        findContours(rangeRes, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
        
        /* Filtering */
        vector < vector<Point> > balls;
        vector <Rect> ballsBox;
        
        for (size_t i = 0; i < contours.size(); i++)
        {
            Rect bBox;
            bBox = boundingRect(contours[i]);

            float ratio = (float) bBox.width / (float) bBox.height;
            if (ratio > 1.0f)
                ratio = 1.0f / ratio;

            // Searching for a bBox almost square
            if (ratio > 0.75 && bBox.area() >= 400)
            {
                balls.push_back(contours[i]); // Adds a contour at the end of the vector balls
                ballsBox.push_back(bBox);
            }
        }

        // cout << "Balls found:" << ballsBox.size() << endl;

        /* Detection result */
        for (size_t i = 0; i < balls.size(); i++)
        {
            drawContours(res, balls, i, CV_RGB(20,150,20), 1);
            rectangle(res, ballsBox[i], CV_RGB(0,255,0), 2);

            Point center;
            center.x = ballsBox[i].x + ballsBox[i].width / 2;
            center.y = ballsBox[i].y + ballsBox[i].height / 2;
            circle(res, center, 2, CV_RGB(20,150,20), -1);

            stringstream sstr;
            sstr << "(" << center.x << "," << center.y << ")";
            putText(res, sstr.str(), Point(center.x + 3, center.y - 3), FONT_HERSHEY_SIMPLEX, 0.5, CV_RGB(20,150,20), 2);
        }


        /* KALMAN Update Phase */
        if (balls.size() == 0)
        {
            notFoundCount++;
            // cout << "notFound Count:" << notFoundCount << endl;
            if( notFoundCount >= 100 )
            {
                found = false;
            }
            /*else
                kf.statePost = state;*/
        }
        else
        {
            notFoundCount = 0;

            meas.at<float>(0) = ballsBox[0].x + ballsBox[0].width / 2;
            meas.at<float>(1) = ballsBox[0].y + ballsBox[0].height / 2;
            meas.at<float>(2) = (float)ballsBox[0].width;
            meas.at<float>(3) = (float)ballsBox[0].height;

            if (!found) // First detection!
            {
                // >>>> Initialization
                kf.errorCovPre.at<float>(0) = 1; // px
                kf.errorCovPre.at<float>(7) = 1; // px
                kf.errorCovPre.at<float>(14) = 1;
                kf.errorCovPre.at<float>(21) = 1;
                kf.errorCovPre.at<float>(28) = 1; // px
                kf.errorCovPre.at<float>(35) = 1; // px

                state.at<float>(0) = meas.at<float>(0);
                state.at<float>(1) = meas.at<float>(1);
                state.at<float>(2) = 0;
                state.at<float>(3) = 0;
                state.at<float>(4) = meas.at<float>(2);
                state.at<float>(5) = meas.at<float>(3);
                // <<<< Initialization

                kf.statePost = state;
                
                found = true;
            }
            else
                kf.correct(meas); // Kalman Correction

            // cout << "Measure matrix:" << endl << meas << endl;
        }

        /* Show Final result */
        imshow("Tracking", res);

        // Press ESC to STOP
        end_prog();
    }
    return EXIT_SUCCESS;
}


/*
    FUNCTIONS
*/

/* Image processing function */
Mat img_processing(Mat frame_in)
{
    // Step 1: Noise smoothing
    Mat frame_blur;
    GaussianBlur(frame_in, frame_blur, Size(5, 5), 3.0, 3.0);
    // Step 2: HSV conversion
    Mat frame_HSV;
    cvtColor(frame_blur, frame_HSV, CV_BGR2HSV);
    // Step 3: Color Thresholding
    // Note: change parameters for different colors
    Mat frame_res = Mat::zeros(frame_in.size(), CV_8UC1);
    inRange(frame_HSV, Scalar(MIN_H_BLUE / 2, 100, 80), Scalar(MAX_H_BLUE / 2, 255, 255), frame_res);
    // Step 4: Morphological transformation
    erode(frame_res, frame_res, Mat(), Point(-1, -1), 2);
    dilate(frame_res, frame_res, Mat(), Point(-1, -1), 2);

    // Thresholding viewing
    imshow("Threshold", frame_res);
    return frame_res;
}

/* END function */
void end_prog()
{
    if(waitKey(10) == 27)
    {
        cout << "END PROGRAM" << endl;
        exit(EXIT_SUCCESS);
    }          
}