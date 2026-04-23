#include "Tracking/TrackerOverlay.hpp"
#include "Motion/Table.hpp"
#include "Constants.hpp"

TrackerOverlay::TrackerOverlay(Converter converter)
    :_converter(converter) {}

void TrackerOverlay::overlay(cv::Mat& mat) {
    auto overlay_goal = [&](Point2<double> goal, std::optional<Setting> setting) {
        if (setting) {
            auto p1 = _converter( goal - 0.5 * Constants::Table::GOAL_WIDTH * Point2<double>::xAxis() );
            auto p2 = _converter( goal + 0.5 * Constants::Table::GOAL_WIDTH * Point2<double>::xAxis() );
            cv::line(mat, p1, p2, setting->first, setting->second);
        }
    };
    
    overlay_goal(Constants::Table::HUMAN_GOAL, _human_goal);
    overlay_goal(Constants::Table::ROBOT_GOAL, _robot_goal);

    auto pixel_radius = [&](Point2<double> center, double radius) {
        auto offset = radius * Point2<double>::xAxis();
        return cv::norm(_converter(center) - _converter(center + offset));
    };

    if (_mallet && Table::mallet().isValid()) {
        auto pos = Table::mallet().position();
        int radius = std::round(pixel_radius(pos, Constants::Mallet::RADIUS));
        cv::circle(mat, _converter(pos), radius, _mallet->first, _mallet->second);
    }

    if (_puck && Table::puck().isValid()) {
        auto pos = Table::puck().position();
        auto radius = pixel_radius(pos, Constants::Puck::RADIUS);
        cv::circle(mat, _converter(pos), radius, _puck->first, _puck->second);
    }

    if (_mallet_target && Table::mallet().isValid()) {
        cv::circle(mat, _converter(Table::routine()->target().first), 2, _mallet_target->first, _mallet_target->second);
    }

    if (_puck_trajectory && Table::puck().isValid()) {
        // Fetch timestamps
        auto timestamps = Table::puck().trajectory(true);

        if (!timestamps.empty() > 0) {
            auto [_, first_orientation] = timestamps[0];
            auto prev_point = _converter(first_orientation.position);

            auto valid_point = [&](const cv::Point& p) {
                return std::isfinite(p.x) && std::isfinite(p.y) &&
                    p.x >= 0 && p.y >= 0 &&
                    p.x < mat.cols && p.y < mat.rows;
            };

            for (const auto& [_, orientation] : timestamps) {
                cv::Point cur_point = _converter(orientation.position);
                cv::circle(mat, cur_point, 2, _puck_trajectory->first, _puck_trajectory->second);
                
                const int thickness = 1 + _puck_trajectory->second / 2;

                if (valid_point(prev_point) && valid_point(cur_point)) {
                    cv::line(mat, prev_point, cur_point, _puck_trajectory->first, thickness);
                    prev_point = cur_point;
                }
            }
        }
    }
}

void TrackerOverlay::robotGoal(const Setting& setting) {
    _robot_goal = setting;
}

void TrackerOverlay::humanGoal(const Setting& setting) {
    _human_goal = setting;
}

void TrackerOverlay::malletTarget(const Setting& setting) {
    _mallet_target = setting;
}

void TrackerOverlay::puckTrajectory(const Setting& setting) {
    _puck_trajectory = setting;
}

void TrackerOverlay::mallet(const Setting& setting) {
    _mallet = setting;
}

void TrackerOverlay::puck(const Setting& setting) {
    _puck = setting;
}
