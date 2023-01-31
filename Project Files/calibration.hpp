// Project 4 of CS 5330: Pattern Recognition and Computer Vision
// Created by Dhruvil Parikh

#ifndef calibration_hpp
#define calibration_hpp

// Importing Libraries
#include <iostream>

#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <dirent.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui/highgui_c.h>

#include "calibration.cpp"

using namespace std;
using namespace cv;

// Function to write calibration data into a .csv file
void writeData(const Mat &camera_matrix, const Mat &dist_coeffs);

// Function to read calibration data from a .csv file
void readData(Mat &camera_matrix, Mat &dist_coeffs);

// Function to draw coordinate axis at origin of the checkerboard
void drawAxesCheckerboard(Mat &frame, vector<Point2f> &image_set);

// Function to draw Traingular Prism on checkerboard
void drawTraingularPrism(Mat &frame, vector<Point2f> &image_object);

// Function to get vertex and vertex indices from .obj file
void getObjectParams(char* filename, vector< Point3f > &vertices, vector< Vec3i > &vertexIndices);

// Function to actually draw and project the parameters obtained from .obj file
void drawObject(Mat &frame, vector<Point2f> &image_points, vector<Vec3i> &vertexIndices);

#endif