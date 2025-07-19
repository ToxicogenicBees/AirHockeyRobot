
#include <opencv2/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <stdio.h>

#define ET_BUFFER_LENGTH 30

using namespace cv;
using namespace std;

int main(int, char**)
{
    // declare variables for measuring execution time
    auto start = std::chrono::high_resolution_clock::now();
    auto end = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::milli> ms_elapsed = end - start;
    double executionTimesBuffer[ET_BUFFER_LENGTH];
    int etIndex = 0;

    // image handling variables
    cv::cuda::GpuMat frame, frame_inv, hsv_inv;
    cv::cuda::GpuMat mask;
    Moments m;
    Point2d centroid;

    //--- INITIALIZE VIDEOCAPTURE
    VideoCapture cap;
    // open the default camera using default API
    // cap.open(0);
    // OR advance usage: select any API backend
    int deviceID = 0;             // 0 = open default camera
    int apiID = cv::CAP_ANY;      // 0 = autodetect default API
    // open selected camera using selected API
    cap.open(deviceID, apiID);
    // check if we succeeded
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    // prevent camera from autoadjusting to lighting conditions
    // cap.set(CAP_PROP_AUTO_EXPOSURE, 0.25);
    cap.set(CAP_PROP_AUTO_EXPOSURE, 0);


    //--- GRAB AND WRITE LOOP
    cout << "Start grabbing" << endl
        << "Press any key to terminate" << endl;
    for (;;)
    {
        // wait for a new frame from camera and store it into 'frame'
        cap.read(frame);
        // check if we succeeded
        if (frame.empty()) {
            cerr << "ERROR! blank frame grabbed\n";
            break;
        }

        // to measure execution time of object tracking algorithm
        start = std::chrono::high_resolution_clock::now();

        // Find red in image frame
        frame_inv = ~frame;  // invert to turn red to cyan: https://stackoverflow.com/a/32523532
        cvtColor(frame_inv, hsv_inv, COLOR_BGR2HSV);

        inRange(hsv_inv, Scalar(90 - 10, 50, 50), Scalar(90 + 10, 255, 255), mask); // Cyan is 90

        // imshow("Mask", mask);

        // Find and draw contours around objects
        // vector<vector<Point> > contours;
        // vector<Vec4i> hierarchy;
        // findContours(mask, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE);

        // drawContours(mask, contours, -1, Scalar(128, 255, 255),
        //    3, LINE_AA, hierarchy, 0);

        // get centroid
        m = moments(mask, true);
        centroid = Point2d(m.m10 / m.m00, m.m01 / m.m00);

        end = std::chrono::high_resolution_clock::now();

        ms_elapsed = end - start;
        executionTimesBuffer[etIndex++ % ET_BUFFER_LENGTH] = ms_elapsed.count();

        // get average execution time
        double avg = 0;
        for (int i = 0; i < ET_BUFFER_LENGTH; i++) {
            avg += executionTimesBuffer[i];
        }
        avg /= ET_BUFFER_LENGTH;

        // return and clearline
        std::cout << avg << " [ms]                          " << '\r';

        // draw tracked object
        circle(mask, centroid, 5, Scalar(128, 0, 0), -1);
        imshow("Mask", mask);

        // show live and wait for a key with timeout long enough to show images
        imshow("Live", frame);
        if (waitKey(5) >= 0)
            break;
    }
    // the camera will be deinitialized automatically in VideoCapture destructor
    return 0;
}
