/*
 * LocationEstimation.cpp
 *
 *  Created on: Jun 24, 2016
 *      Author: ericwadkins
 */

#include "Simulation.h"

int Simulation::simulationCount = 0;

const int Simulation::STARTING = 0;
const int Simulation::STARTED = 1;

Simulation::Simulation(const char* imagePath, bool allowLocationAccess, bool allowMapAccess, bool allowImageAccess)
			: locationAccess(allowLocationAccess), mapAccess(allowMapAccess), imageAccess(allowImageAccess) {
	simulationCount++;
	this->simulationId = simulationCount;
	this->imagePath = imagePath;

	state = STARTING;

	maxWindowWidth = 600;
	maxWindowHeight = 400;

	desiredMapSize = 200 * 200;

	cursorSize = 9;

	rayWidth = 2;
	targetPointSize = 4;

	enableRayNoise = false;
	enableRayFailure = false;
	rayFailureRate = 0.01;
	rayMaxNoise = 2;
}

cv::Rect getMaxRectSingleContour(const cv::Mat1b& src) {
    cv::Mat1f width(src.rows, src.cols, float(0));
    cv::Mat1f height(src.rows, src.cols, float(0));

    cv::Rect maxRect(0, 0, 0, 0);
    double maxArea = 0;

    for (int r = 0; r < src.rows; r++) {
        for (int c = 0; c < src.cols; c++) {
            if (src(r, c) == 0) {
            	height(r, c) = 1.0 + ((r>0) ? height(r - 1, c) : 0);
                width(r, c) = 1.0 + ((c>0) ? width(r, c - 1) : 0);
            }
            float minw = width(r, c);
            for (int h = 0; h < height(r, c); h++) {
                minw = std::min(minw, width(r - h, c));
                float area = (h + 1) * minw;
                if (area > maxArea) {
                    maxArea = area;
                    maxRect = cv::Rect(cv::Point(c - minw + 1, r - h),
                    		cv::Point(c + 1, r + 1));
                }
            }
        }
    }

    return maxRect;
}

cv::Rect getMaxRect(cv::Mat binaryImg) {
	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(binaryImg, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	cv::Rect maxRect(0, 0, 0, 0);
	int maxArea = 0;
	for (int i = 0; i < contours.size(); ++i) {
		std::cout << i << std::endl;
		cv::Mat maskSingleContour(binaryImg.rows, binaryImg.cols, uchar(0));
		cv::drawContours(maskSingleContour, contours, i, cv::Scalar(255),
				CV_FILLED);

		cv::Rect rect = getMaxRectSingleContour(~maskSingleContour);
		int area = rect.width * rect.height;
		if (area > maxArea) {
			maxRect = rect;
			maxArea = area;
		}
	}
	return maxRect;
}

void Simulation::start() {
	cv::namedWindow("Simulation " + std::to_string(simulationId), CV_WINDOW_AUTOSIZE);

	std::cout << "Image path: " << imagePath << std::endl;

	// Load source image, and scale to desired map size
	srcImg = cv::imread(imagePath, cv::IMREAD_UNCHANGED);
	double scale = std::sqrt((float) desiredMapSize / (srcImg.cols * srcImg.rows));
	cv::resize(srcImg, srcImg, cv::Size(srcImg.cols * scale, srcImg.rows * scale), 0, 0, cv::INTER_NEAREST);
	std::cout << srcImg.cols << " x " << srcImg.rows << std::endl;

	// Create grayscale and then black and white image
	//cv::Mat grayImg;
	//cv::cvtColor(srcImg, grayImg, cv::COLOR_BGR2GRAY);
	cv::Mat hsv;
	cv::cvtColor(srcImg, hsv, cv::COLOR_BGR2HSV);
	cv::Mat binaryImg;
	cv::inRange(hsv, cv::Scalar(0, 0, 0), cv::Scalar(180, 255, 50), binaryImg);
	binaryImg = ~binaryImg;

	// Save copy of the map
	cv::cvtColor(binaryImg, map, cv::COLOR_GRAY2BGR);

	// Determine starting position (largest available rectangle)
	cv::Rect maxRect = getMaxRect(binaryImg);
	//cv::rectangle(map, maxRect, cv::Scalar(220, 255, 220), -1); // Draws starting area
	pos = cv::Point(maxRect.x + (maxRect.width - 1) / 2,
			maxRect.y + (maxRect.height - 1) / 2);

	onStart();
	state = STARTED;

	long count = 0;
	while (true) {

		cv::Mat display = map.clone();

		// Create the display image
		double scale = std::max((double) maxWindowWidth / display.cols,
				(double) maxWindowHeight / display.rows);
		cv::resize(display, display,
				cv::Size(display.cols * scale, display.rows * scale),
				0, 0, cv::INTER_NEAREST);

		onDisplayBackground(display, scale);

		// Gets data from rays and draws them
		std::vector<std::pair<Ray, double> > rayMap;
		for (int i = 0; i < rays.size(); i++) {
			Ray ray = rays[i];
			cv::Point center = getCenterPosition();
			double angle = ray.getAngle();
			double maxDistance = ray.getMaxDistance();

			double dist = rayCast(center.x, center.y, angle, maxDistance);
			rayMap.push_back(std::pair<Ray, double>(ray, dist));

			double startX = center.x;
			double startY = center.y;
			double endX;
			double endY;
			if (dist > -1) {
				endX = startX + std::cos(angle) * dist;
				endY = startY + std::sin(angle) * dist;
			}
			else {
				endX = startX + std::cos(angle) * maxDistance;
				endY = startY + std::sin(angle) * maxDistance;
			}

			cv::line(display, cv::Point(startX * scale, startY * scale),
					cv::Point(endX * scale, endY * scale),
					cv::Scalar(0, 0, 255), rayWidth, 4);
			if (dist > -1) {
				cv::circle(display, cv::Point(endX * scale, endY * scale),
									targetPointSize, cv::Scalar(0, 0, 255), -1);
			}

			std::cout << dist << "\t";
		}
		std::cout << std::endl;

		// Draw cursor
		cv::circle(display, cv::Point((pos.x + 0.5) * scale, (pos.y + 0.5) * scale),
				cursorSize, cv::Scalar(255, 0, 0), -1);
		/*cv::rectangle(display, cv::Rect((pos.x + 0.5) * scale - cursorSize,
				(pos.y + 0.5) * scale - cursorSize, cursorSize * 2,
				cursorSize * 2), cv::Scalar(255, 0, 0), -1);*/
		if (cursorSize >= 5) {
			cv::circle(display, cv::Point((pos.x + 0.5) * scale, (pos.y + 0.5) * scale),
								2, cv::Scalar(0, 0, 255), -1);
		}

		onDisplayForeground(display, scale);

		// Display image
		cv::imshow("Simulation " + std::to_string(simulationId), display);

		onData(rayMap);

		// Handle keyboard input
		int key = cv::waitKey(1);
		if (key > -1) {
			keyListener(key);
		}

		count++;
	}
}

double Simulation::rayCast(int x, int y, double angle, double maxDist) {
	double stepSize = 0.5;
	for (int i = 0; i * stepSize <= maxDist; i++) {
		double dist = i * stepSize;
		int targetX = x + std::cos(angle) * dist;
		int targetY = y + std::sin(angle) * dist;
		if (_isBarrier(targetX, targetY)) {
			if (enableRayFailure && rand() / (RAND_MAX + 1.0) < rayFailureRate) {
				return -1;
			}
			return enableRayNoise ?
					dist + rand() / (RAND_MAX + 1.0) * rayMaxNoise : dist;
		}
	}
	if (enableRayFailure && rand() / (RAND_MAX + 1.0) < rayFailureRate) {
		return rand() / (RAND_MAX + 1.0) * maxDist;
	}
	return -1;
}

void Simulation::registerRay(double angle, double maxDistance) {
	Ray ray = Ray(angle, maxDistance);
	rays.push_back(ray);
}

cv::Point Simulation::getPosition() {
	if (!locationAccess) kill("Location access not allowed");
	return pos;
}

cv::Point Simulation::getCenterPosition() {
	if (!locationAccess) kill("Location access not allowed");
	return cv::Point(pos.x + 0.5, pos.y + 0.5);
}

cv::Mat Simulation::getMap() {
	if (!mapAccess) kill("Map access not allowed");
	return map;
}

int Simulation::getMapWidth() {
	return map.cols;
}

int Simulation::getMapHeight() {
	return map.rows;
}

bool Simulation::canMove(int x, int y) {
	if (!mapAccess) kill("Map access not allowed");
	if (x < 0 || x >= map.cols || y < 0 || y >= map.rows) {
		return false;
	}
	return map.at<cv::Vec3b>(cv::Point(x, y)) != cv::Vec3b(0, 0, 0);
}

bool Simulation::_canMove(int x, int y) {
	if (x < 0 || x >= map.cols || y < 0 || y >= map.rows) {
		return false;
	}
	return map.at<cv::Vec3b>(cv::Point(x, y)) != cv::Vec3b(0, 0, 0);
}

bool Simulation::isBarrier(int x, int y) {
	if (!mapAccess) kill("Map access not allowed");
	if (x < 0 || x >= map.cols || y < 0 || y >= map.rows) {
		return false;
	}
	return map.at<cv::Vec3b>(cv::Point(x, y)) == cv::Vec3b(0, 0, 0);
}

bool Simulation::_isBarrier(int x, int y) {
	if (x < 0 || x >= map.cols || y < 0 || y >= map.rows) {
		return false;
	}
	return map.at<cv::Vec3b>(cv::Point(x, y)) == cv::Vec3b(0, 0, 0);
}

void Simulation::setStartingPosition(int x, int y) {
	if (state == STARTING) {
		if (_canMove(x, y)) {
			pos.x = x;
			pos.y = y;
		}
	}
	else {
		kill("Cannot set starting position, simulation has already started");
	}
}

bool Simulation::move(int x, int y) {
	if (_canMove(x, y)) {
		pos = cv::Point(x, y);
		return true;
	}
	return false;
}

bool Simulation::moveUp() {
	if (_canMove(pos.x, pos.y - 1)) {
		pos = cv::Point(pos.x, pos.y - 1);
		return true;
	}
	return false;
}

bool Simulation::moveDown() {
	if (_canMove(pos.x, pos.y + 1)) {
		pos = cv::Point(pos.x, pos.y + 1);
		return true;
	}
	return false;
}

bool Simulation::moveLeft() {
	if (_canMove(pos.x - 1, pos.y)) {
		pos = cv::Point(pos.x - 1, pos.y);
		return true;
	}
	return false;
}

bool Simulation::moveRight() {
	if (_canMove(pos.x + 1, pos.y)) {
		pos = cv::Point(pos.x + 1, pos.y);
		return true;
	}
	return false;
}

void Simulation::rotate(double angle) {
	for (int i = 0; i < rays.size(); i++) {
		rays[i] = Ray(rays[i].getAngle() - angle, rays[i].getMaxDistance());
	}
}

bool compareArea(std::pair<cv::Point, double> a, std::pair<cv::Point, double> b) {
	return a.second > b.second;
}

std::vector<std::pair<cv::Point, double> > Simulation::getBlobs(std::vector<std::pair<cv::Scalar, cv::Scalar> > ranges) {
	if (!imageAccess) kill("Image access not allowed");

	cv::Mat blurred;
	cv::medianBlur(srcImg, blurred, 3);
	//cv::namedWindow("Thresholded", CV_WINDOW_AUTOSIZE);
	cv::Mat hsv;
	cv::cvtColor(blurred, hsv, cv::COLOR_BGR2HSV);

	cv::Mat thresholded(hsv.rows, hsv.cols, CV_8UC1, cv::Vec3b(0, 0, 0));
	for (int i = 0; i < ranges.size(); i++) {
		cv::Mat img;
		cv::inRange(hsv, ranges[i].first, ranges[i].second, img);
		cv::addWeighted(thresholded, 1.0, img, 1.0, 0.0, thresholded);
	}
	//cv::imshow("Thresholded", thresholded);
	//cv::waitKey(0);

	//cv::Mat display;
	//cv::cvtColor(thresholded, display, CV_GRAY2BGR);

	std::vector<std::vector<cv::Point> > contours;
	cv::findContours(thresholded, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

	std::vector<cv::Moments> m(contours.size());
	for (int i = 0; i < contours.size(); i++) {
		m[i] = moments(contours[i], false);
	}

	///  Get the mass centers:
	std::vector<std::pair<cv::Point, double> > contourAreaMap;
	for (int i = 0; i < contours.size(); i++) {
		cv::Point center = cv::Point(m[i].m10 / m[i].m00 , m[i].m01 / m[i].m00);
		double area = m[i].m00;
		contourAreaMap.push_back(std::pair<cv::Point, double>(center, area));
		//cv::circle(display, center, std::sqrt(area), cv::Scalar(0, 0, 255), 2);
	}

	std::sort(contourAreaMap.begin(), contourAreaMap.end(), compareArea);


	//cv::imshow("Thresholded", display);
	//cv::waitKey(0);

	return contourAreaMap;
}
void Simulation::kill() {
	exit(1);
}

void Simulation::kill(const char* msg) {
	std::cerr << "KILLING SIMULATION: " + std::string(msg) << std::endl;
	kill();
}
