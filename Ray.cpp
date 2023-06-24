/*
 * Ray.cpp
 *
 *  Created on: Jun 25, 2016
 *      Author: ericwadkins
 */

#include "Ray.h"

Ray::Ray(double _angle, double _maxDistance) {
	angle = _angle;
	maxDistance = _maxDistance;
}

double Ray::getAngle() {
	return angle;
}

double Ray::getMaxDistance() {
	return maxDistance;
}
