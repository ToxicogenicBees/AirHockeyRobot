#include <iostream>
#include <cassert>

#include "Tracking/CameraTracker.hpp"
#include "Motion/MovingObject.hpp"
#include "Motion/Table.hpp"
#include "Constants.hpp"

namespace{
    const Point2<int> PIXEL_OFFSET = {30, 110};
    const double INCHES_PER_PIXEL = 1.0 / 21.0;

    cv::Point inchesToPixels(const Point2<double>& inches) {
        return {
            (int) (inches.y / INCHES_PER_PIXEL) + PIXEL_OFFSET.y,
            (int) (inches.x / INCHES_PER_PIXEL) + PIXEL_OFFSET.x
        };
    }

    Point2<double> pixelsToInches(const cv::Point& pixels) {
        return {
            INCHES_PER_PIXEL * (double) (pixels.y - PIXEL_OFFSET.x),
            INCHES_PER_PIXEL * (double) (pixels.x - PIXEL_OFFSET.y)
        };
    }
}

CameraTracker::CameraTracker(const cv::Scalar& min_color, const cv::Scalar& max_color)
    : _filter(min_color, max_color), _overlay(inchesToPixels) {}

void CameraTracker::init() {
    // Open selected camera using selected API
    _capture.open(Constants::Camera::DEVICE_ID, Constants::Camera::API_ID);
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

    // Overlay settings
    _overlay.malletTarget({{255, 255, 255}, 2});
    _overlay.puckTrajectory({{0, 0, 255}, 2});
    _overlay.puck({{0, 0, 255}, 2});
    _overlay.mallet({{255, 255, 255}, 2});
    _overlay.humanGoal({{255, 127, 0}, 3});
    _overlay.robotGoal({{0, 255, 0}, 3});
}

void CameraTracker::track() {
    // Wait for a new frame from camera and store it into 'frame'
    _capture.read(_frame);

    // Check if we succeeded
    if (_frame.empty()) {
        std::cerr << "ERROR! blank frame grabbed\n";
        return;
    }

    // Filter the camera image
    _filter.filter(_frame);

    // Find moments of the image
    cv::Moments m = cv::moments(_filter.threshold(), true);

    // Invalid centroid
    if (std::abs(m.m00) <= 1e-6) {
        // update internal timer
        _puck->moveTo(_puck->position());

        return;
    }
    
    // Convert the pixel value at the center of the object to inches
    cv::Point pixels(m.m10 / m.m00, m.m01 / m.m00);
    auto inches = pixelsToInches(pixels);

    // If the color is found, update the puck
    // if puck only moves 1 or two pixels then ignore
    auto dif = _prev_pixels - pixels;
    if (dif.dot(dif) <= 4) {
        // update internal timer
        _puck->moveTo(_puck->position());

        return;
    }

    if (inches.x >= 0 && inches.y >= 0) {
        _puck->moveTo(inches);
        _prev_pixels = pixels;
    } else {
        // update internal timer
        _puck->moveTo(_puck->position());
    }
}

void CameraTracker::display() {
    // Ignore if there's no data to display
    if (_frame.empty()) {
        std::clog << "Empty frame\n";
        return;
    }

    // Overlay puck position on both the live and filtered frames
    auto filtered_frame = _filter.filtered();
    cv::circle(filtered_frame, _prev_pixels, 5, {128, 0, 0}, -1);
    cv::circle(_frame, _prev_pixels, 5, {128, 0, 0}, -1);

    // Overlay table info onto live video
    _overlay.overlay(_frame);

    // Show the modified visuals
    cv::imshow("Filtered", filtered_frame);
    cv::imshow("Live", _frame);
    cv::waitKey(1);
}
