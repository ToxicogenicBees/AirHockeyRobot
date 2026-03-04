#include "Motion/PuckTracker.h"
#include "Motion/Puck.h"
#include <iostream>
#include <cassert>

PuckTracker::PuckTracker(int device_id, int api_id): _DEVICE_ID(device_id), _API_ID(api_id) {}

void PuckTracker::init() {
    // Open selected camera using selected API
    _capture.open(_DEVICE_ID, _API_ID);
    // Check if we succeeded
    if (!_capture.isOpened()) {
        std::cerr << "ERROR! Unable to open camera\n";
        exit(-1);
    }
    
    // Set the camera's resolution
    _capture.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
    _capture.set(cv::CAP_PROP_FRAME_HEIGHT, 720);

    // Set the camera's frames per second (fps)
    _capture.set(cv::CAP_PROP_FPS, 100);

    // Set camera to auto adjust exposure
    _capture.set(cv::CAP_PROP_AUTO_EXPOSURE, 0.75);

    // Set filtering range
    filterForYellow();

    // Debugging log
    /*
        int frame_width = (int)_capture.get(cv::CAP_PROP_FRAME_WIDTH);
        int frame_height = (int)_capture.get(cv::CAP_PROP_FRAME_HEIGHT);
        std::clog << "Frame size: " << Point2<int>(frame_width, frame_height) << "\n";

        double fps = _capture.get(cv::CAP_PROP_FPS);
        std::clog << "FPS: " << fps << "\n";

        double exposure = _capture.get(cv::CAP_PROP_EXPOSURE);
        std::clog << "Exposure: " << exposure << "\n";
    */
}

void PuckTracker::captureFrame() {
    // Wait for a new frame from camera and store it into 'frame'
    _capture.read(_frame);

    // Check if we succeeded
    if (_frame.empty()) {
        std::cerr << "ERROR! blank frame grabbed\n";
        return;
    }

    // Convert inverted captured image from BGR to HSV colorspace
    cv::cvtColor(_frame, _frame_hsv, cv::COLOR_BGR2HSV);

    // Filter image to find the specific color
    cv::inRange(_frame_hsv, _min, _max, _frame_threshold);

    // Filter image to show only specific colors from orignal image
    _frame_filtered = cv::Mat();
    cv::bitwise_and(_frame, _frame, _frame_filtered, _frame_threshold);

    // Find moments of the image
    cv::Moments m = cv::moments(_frame_threshold, true);
    cv::Point p(m.m10 / m.m00, m.m01 / m.m00);
    
    // Convert the pixel value to inches
    Point2<double> inches(
        _INCHES_PER_PIXEL * (p.y - 45),
        _INCHES_PER_PIXEL * (p.x - 105)
    );

    // If the color is found, update the puck
    // if puck only moves 1 or two pixels then ignore
    auto dif = _prev_pixels - p;
    if (dif.dot(dif) <= 4)
        return;

    if (inches.x >= 0 && inches.y >= 0) {
        Puck::moveTo(inches);
        _prev_pixels = p;
    }
}

void PuckTracker::displayFrame() {
    // Ignore if there's no data to display
    if (_frame.empty()) {
        return;
    }

    // Show the image with a point mark at the centroid
    cv::circle(_frame_filtered, _prev_pixels, 5, {128, 0, 0}, -1);
    cv::circle(_frame, _prev_pixels, 5, {128, 0, 0}, -1);

    // Show live and wait for a key with timeout long enough to show images
    cv::imshow("Live", _frame);
    cv::imshow("Filtered", _frame_filtered);
    cv::waitKey(1);
}

void PuckTracker::setFilteringRange(const cv::Scalar& min, const cv::Scalar& max) {
    _min = min;
    _max = max;
}

void PuckTracker::filterForYellow() {
    setFilteringRange({20, 100, 100}, {40, 255, 255});
}

void PuckTracker::filterForRed() {
    setFilteringRange({80, 100, 50}, {100, 255, 255});
}

void PuckTracker::filterForGreen() {
    setFilteringRange({70, 50, 50}, {100, 255, 255});
}

std::pair<cv::Scalar, cv::Scalar> PuckTracker::getFilteringRange() {
    return {_min, _max};
}

int PuckTracker::getDeviceId() {
    return _DEVICE_ID;
}

int PuckTracker::getApiId() {
    return _API_ID;
}
