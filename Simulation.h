/*
 * LocationEstimation.h
 *
 *  Created on: Jun 24, 2016
 *      Author: ericwadkins
 */

#ifndef SIMULATION_H_
#define SIMULATION_H_

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>

#include <iostream>
#include <vector>
#include <string>
#include <functional>
#include <math.h>

#include "Ray.h"

class Simulation {
public:
	Simulation(const char* imagePath, bool allowLocationAccess, bool allowMapAccess, bool allowImageAccess);
	virtual ~Simulation() {}
	void start();
	double rayCast(int x, int y, double angle, double maxDist);
	void registerRay(double angle, double maxDist);
	cv::Point getPosition();
	cv::Point getCenterPosition();
	cv::Mat getMap();
	int getMapWidth();
	int getMapHeight();
	void setStartingPosition(int x, int y);
	bool move(int x, int y);
	bool moveUp();
	bool moveDown();
	bool moveLeft();
	bool moveRight();
	void rotate(double angle);
	bool canMove(int x, int y);
	bool isBarrier(int x, int y);
	std::vector<std::pair<cv::Point, double> > getBlobs(std::vector<std::pair<cv::Scalar, cv::Scalar> > ranges);
	virtual void onStart() = 0;
	virtual void onData(std::vector<std::pair<Ray, double> > rayMap) = 0;
	virtual void onDisplayBackground(cv::Mat display, double scale) = 0;
	virtual void onDisplayForeground(cv::Mat display, double scale) = 0;
	virtual void keyListener(int key) = 0;
	void kill();
	void kill(const char* msg);
private:
	static int simulationCount;

	int simulationId;
	const bool locationAccess;
	const bool mapAccess;
	const bool imageAccess;

	int state;
	static const int STARTING;
	static const int STARTED;

	const char* imagePath;

	int maxWindowWidth;
	int maxWindowHeight;

	int desiredMapSize;

	int cursorSize;

	std::vector<Ray> rays;

	int rayWidth;
	int targetPointSize;

	bool enableRayNoise;
	bool enableRayFailure;
	double rayFailureRate;
	double rayMaxNoise;

	cv::Mat srcImg;
	cv::Mat map;
	cv::Point pos;

	bool _canMove(int x, int y);
	bool _isBarrier(int x, int y);
};

#endif /* SIMULATION_H_ */
