#include "Visuals/Table.h"
#include "Motion/Mallet.h"
#include "Motion/Puck.h"
#include "Constants.h"

const uint8_t IMG_SCALE = 10;

cv::Mat Table::_canvas = cv::Mat::zeros(IMG_SCALE * Constants::Table::SIZE.y, IMG_SCALE * Constants::Table::SIZE.x, CV_8UC3);

bool Table::_show_relative_times = true;
bool Table::_show_target_mallet = true;
bool Table::_show_target_puck = true;

void Table::render() {
    // Clear image
    _canvas.setTo(cv::Scalar(0, 0, 0));

    // Fetch puck/mallet locations
    Point2<double> mallet_pos = Mallet::position();
    Point2<double> puck_pos = Puck::position();

    // Draw puck
    cv::Point puck_center(IMG_SCALE * puck_pos.x, IMG_SCALE * (Constants::Table::SIZE.y - puck_pos.y));
    cv::circle(_canvas, puck_center, IMG_SCALE * Constants::Puck::RADIUS, cv::Scalar(0, 0, 255), 2);

    // Draw puck trajectory
    std::vector<Point3<double>> ts = Puck::estimateTrajectory(false);
    if (_show_target_puck) {
        for (const Point3<double> t : ts) {
            cv::Point point_center(IMG_SCALE * t.x, IMG_SCALE * (Constants::Table::SIZE.y - t.y));
            cv::circle(_canvas, point_center, 2, cv::Scalar(0, 0, 255), 2);
        }
    }

    // Draw mallet
    cv::Point mallet_center(IMG_SCALE * mallet_pos.x, IMG_SCALE * (Constants::Table::SIZE.y - mallet_pos.y));
    cv::circle(_canvas, mallet_center, IMG_SCALE * Constants::Mallet::RADIUS, cv::Scalar(255, 255, 255), 2);

    // Draw mallet trajectory
    if (_show_target_mallet) {
        Point2<double> t = Mallet::target();
        cv::Point point_center(IMG_SCALE * t.x, IMG_SCALE * (Constants::Table::SIZE.y - t.y));
        cv::circle(_canvas, point_center, 3, cv::Scalar(255, 255, 255), 3);
    }

    // Show updated image
    cv::imshow("Table Render", _canvas);
    cv::waitKey(1);
}

void Table::setRelativeTimes(bool state) {
    _show_relative_times = state;
}

void Table::setMalletTarget(bool state) {
    _show_target_mallet = state;
}

void Table::setPuckTargets(bool state) {
    _show_target_puck = state;
}