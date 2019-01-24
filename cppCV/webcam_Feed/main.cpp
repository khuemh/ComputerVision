#include <iostream>
#include <opencv2/opencv.hpp>


using namespace cv;
using namespace std;




int main()
{
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

        // Create WINDOW
        String window_name = "Cam FEED";
        namedWindow(window_name);

        while(1)
        {
                Mat frame;
                bool isSuccess = cap.read(frame);

                flip(frame, frame, 1);

                if(isSuccess == false)
                {
                        cout << "Camera was disconnected" << endl;
                        cin.get();
                        break;
                }

                imshow(window_name, frame);

                // Press ESC to STOP
                if(waitKey(10) == 27)
                {
                        cout << "Stop Stream" << endl;
                        break;
                }              
        }
        return 0;
}
