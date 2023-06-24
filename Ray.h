/*
 * Ray.h
 *
 *  Created on: Jun 25, 2016
 *      Author: ericwadkins
 */

#ifndef RAY_H_
#define RAY_H_

class Ray {
public:
	Ray(double angle, double maxDistance);
	double getAngle();
	double getMaxDistance();
private:
	double angle;
	double maxDistance;
};

#endif /* RAY_H_ */
