#include "Visuals/Overlay.hpp"
#include "Motion/Table.hpp"
#include "Constants.hpp"

Overlay::Overlay(Converter converter)
    :_converter(converter) {}

void Overlay::overlay(cv::Mat& mat) {
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

    if (_mallet) {
        auto pos = Table::mallet().position();
        int radius = std::round(pixel_radius(pos, Constants::Mallet::RADIUS));
        cv::circle(mat, _converter(pos), radius, _mallet->first, _mallet->second);
    }

    if (_puck) {
        auto pos = Table::puck().position();
        auto radius = pixel_radius(pos, Constants::Puck::RADIUS);
        cv::circle(mat, _converter(pos), radius, _puck->first, _puck->second);
    }

    if (_mallet_target) {
        cv::circle(mat, _converter(Table::routine()->target()), 2, _mallet_target->first, _mallet_target->second);
    }

    if (_puck_trajectory) {
        // Fetch timestamps
        auto timestamps = Table::puck().trajectory(true);

        if (!timestamps.empty() > 0) {
            auto [_, first_orientation] = timestamps[0];
            auto prev_point = _converter(first_orientation.position);

            auto validPoint = [&](const cv::Point& p) {
                return std::isfinite(p.x) && std::isfinite(p.y) &&
                    p.x >= 0 && p.y >= 0 &&
                    p.x < mat.cols && p.y < mat.rows;
            };

            for (const auto& [_, orientation] : timestamps) {
                cv::Point cur_point = _converter(orientation.position);
                cv::circle(mat, cur_point, 2, _puck_trajectory->first, _puck_trajectory->second);
                
                const int thickness = 1 + _puck_trajectory->second / 2;

                if (validPoint(prev_point) && validPoint(cur_point)) {
                    cv::line(mat, prev_point, cur_point, _puck_trajectory->first, thickness);
                    prev_point = cur_point;
                }
            }
        }
    }
}

void Overlay::robotGoal(const Setting& setting) {
    _robot_goal = setting;
}

void Overlay::humanGoal(const Setting& setting) {
    _human_goal = setting;
}

void Overlay::malletTarget(const Setting& setting) {
    _mallet_target = setting;
}

void Overlay::puckTrajectory(const Setting& setting) {
    _puck_trajectory = setting;
}

void Overlay::mallet(const Setting& setting) {
    _mallet = setting;
}

void Overlay::puck(const Setting& setting) {
    _puck = setting;
}
