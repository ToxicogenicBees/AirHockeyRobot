#pragma once

#include "Types/Point2.hpp"
#include "Motion/Mallet.h"
#include "Motion/Puck.h"

class Collision {
    private:
        Point2<double> _getRequiredPuckVel(const Point2<double>& target_pos, const Point2<double>& norm);
        
    
    public:

};

Point2<double> Collision::_getRequiredPuckVel(const Point2<double>& target_pos, const Point2<double>& norm) {
    // Fetch puck location data
    Point2<double> puck_vel = Puck::velocity();
    Point2<double> puck_pos = Puck::position();

    // Determine desired puck displacement
    Point2<double> disp = target_pos - puck_pos;

    // Determine required velocity vector needed to move to this target
    // using the provided collision normal
    return Point2<double>();
}