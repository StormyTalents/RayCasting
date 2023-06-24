/*
 * SpatialMapping.cpp
 *
 *  Created on: Jun 25, 2016
 *      Author: ericwadkins
 */

#include "SpatialMapping.h"

const bool SpatialMapping::allowLocationAccess = true;
const bool SpatialMapping::allowMapAccess = false;
const bool SpatialMapping::allowImageAccess = true;

SpatialMapping::SpatialMapping(const char* imagePath)
			: Simulation(imagePath, allowLocationAccess, allowMapAccess, allowImageAccess) {
	start = cv::Point(-1, -1);
	goal = cv::Point(-1, -1);
}

void SpatialMapping::onStart() {
	observedMap = cv::Mat1f(getMapHeight(), getMapWidth(), float(0));
	cv::namedWindow("Observed Map", CV_WINDOW_AUTOSIZE);

	int numRays = 12;
	double maxDistance = 100;

	for (int i = 0; i < numRays; i++) {
		registerRay(M_PI * i / numRays * 2, maxDistance);
	}

	std::vector<std::pair<cv::Scalar, cv::Scalar> > ranges;
	ranges.push_back(std::pair<cv::Scalar, cv::Scalar>(cv::Scalar(100, 127, 127), cv::Scalar(130, 255, 255)));
	std::vector<std::pair<cv::Point, double> > blueBlobs = getBlobs(ranges);

	ranges.clear();
	ranges.push_back(std::pair<cv::Scalar, cv::Scalar>(cv::Scalar(45, 127, 127), cv::Scalar(75, 255, 255)));
	std::vector<std::pair<cv::Point, double> > greenBlobs = getBlobs(ranges);

	if (blueBlobs.size() > 0) {
		cv::Point startingPosition = blueBlobs[0].first;
		setStartingPosition(startingPosition.x, startingPosition.y);
	}
	if (greenBlobs.size() > 0) {
		goal = greenBlobs[0].first;
	}
	else {
		kill("Goal not found! Add a green blob to the image.");
	}

	start = getPosition();
}

void SpatialMapping::onData(std::vector<std::pair<Ray, double> > rayMap) {
	cv::Point origin = getCenterPosition();
	for (int i = 0; i < rayMap.size(); i++) {
		Ray ray = rayMap[i].first;
		double dist = rayMap[i].second;
		double angle = ray.getAngle();
		if (dist > -1) {
			int targetX = origin.x + std::cos(angle) * dist;
			int targetY = origin.y + std::sin(angle) * dist;
			if (targetX >= 0 && targetX < observedMap.cols
					&& targetY >= 0 && targetY < observedMap.rows) {
				observedMap(targetY, targetX) =
						std::min(1.0, observedMap(targetY, targetX) + 1.0 / 3);
			}
		}
	}

	double rotateSpeed = 1.0;
	rotate(M_PI / 180 * rotateSpeed);

	cv::Mat display;
	cv::cvtColor(observedMap, display, CV_GRAY2BGR);
	cv::circle(display, origin, 3, cv::Scalar(0, 255, 0), -1);


	cv::imshow("Observed Map", display);
}

void SpatialMapping::onDisplayBackground(cv::Mat display, double scale) {
	int size = 7;
	cv::circle(display, cv::Point(start.x * scale, start.y * scale), size, cv::Scalar(255, 127, 0), -1);
	cv::circle(display, cv::Point(goal.x * scale, goal.y * scale), size, cv::Scalar(0, 255, 0), -1);
	cv::circle(display, cv::Point(start.x * scale, start.y * scale), size, cv::Scalar(0, 0, 0), 2);
	cv::circle(display, cv::Point(goal.x * scale, goal.y * scale), size, cv::Scalar(0, 0, 0), 2);

	/*cv::rectangle(display, cv::Rect(start.x * scale - size, start.y * scale - size, size * 2, size * 2), cv::Scalar(255, 127, 0), -1);
	cv::rectangle(display, cv::Rect(goal.x * scale - size, goal.y * scale - size, size * 2, size * 2), cv::Scalar(0, 255, 0), -1);
	cv::rectangle(display, cv::Rect(start.x * scale - size, start.y * scale - size, size * 2, size * 2), cv::Scalar(0, 0, 0), 2);
	cv::rectangle(display, cv::Rect(goal.x * scale - size, goal.y * scale - size, size * 2, size * 2), cv::Scalar(0, 0, 0), 2);*/
}

void SpatialMapping::onDisplayForeground(cv::Mat display, double scale) {
}

void SpatialMapping::keyListener(int key) {
	switch(key) {
	case 63232: moveUp(); break; // up
	case 63233: moveDown(); break; // down
	case 63234: moveLeft(); break; // left
	case 63235: moveRight(); break; // right
	case 27: exit(1); break; // esc
	default: std::cout << "Key pressed: " << key << std::endl;
	}
}

int main(int argc, const char *argv[]) {

	// Check arguments
	if (argc < 2) {
		std::cerr << "ERROR No input image provided" << std::endl;
		std::cout << "Usage: ./SpatialMapping <image path>" << std::endl;
		exit(1);
	}

	// Get image path
	const char* imagePath = argv[1];
	Simulation* sim = new SpatialMapping(imagePath);
	sim->start();
}
