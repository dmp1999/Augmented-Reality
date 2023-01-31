// Project 4 of CS 5330: Pattern Recognition and Computer Vision
// Created by Dhruvil Parikh

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

using namespace std;
using namespace cv;

// Function to write calibration data into a .csv file
void writeData(const Mat &camera_matrix, const Mat &dist_coeffs) {

    FILE *fp;
    char filename[255] = "calibration_data.csv";
    
    fp = fopen( filename, "w" );

    if(!fp) {
        printf("Unable to open output file %s\n", filename );
        exit(-1);
    }

    int i = 0, j = 0;
    double temp;

    // Write the Camera Matrix to a file:
    for(i = 0; i < camera_matrix.rows; i++) {
        for(j = 0; j < camera_matrix.cols; j++) {
            temp = camera_matrix.at<double>(i, j);
            fprintf(fp, "%lf\n", temp);
        }
    }

    // Write the Distortion Matrix to a file:
    for(i = 0; i < dist_coeffs.rows; i++){
        temp = dist_coeffs.at<double>(i, 0);
        fprintf(fp, "%lf\n", temp);
    }

}

// Function to read calibration data from a .csv file
void readData(Mat &camera_matrix, Mat &dist_coeffs) {

    FILE *fp;
    char filename[255] = "calibration_data.csv";
    
    fp = fopen( filename, "r" );

    if(!fp) {
        printf("Unable to open output file %s\n", filename );
        exit(-1);
    }

    int i = 0, j = 0;
    double temp;

    // Write the Camera Matrix to a file:
    for(i = 0; i < camera_matrix.rows; i++) {
        for(j = 0; j < camera_matrix.cols; j++) {
            fscanf(fp, "%lf\n", &temp);
            camera_matrix.at<double>(i, j) = temp;
        }
    }

    // Write the Distortion Matrix to a file:
    for(i = 0; i < dist_coeffs.rows; i++){
        fscanf(fp, "%lf\n", &temp);
        dist_coeffs.at<double>(i, 0) = temp;
    }

}

// Function to draw coordinate axis at origin of the checkerboard
void drawAxesCheckerboard(Mat &frame, vector<Point2f> &image_set) {

    arrowedLine(frame, image_set.at(0), image_set.at(1), Scalar(0, 0, 255), 5);
    arrowedLine(frame, image_set.at(2), image_set.at(3), Scalar(0, 255, 0), 5);
    arrowedLine(frame, image_set.at(4), image_set.at(5), Scalar(255, 0, 0), 5);

}

// Function to draw Traingular Prism on checkerboard
void drawTraingularPrism(Mat &frame, vector<Point2f> &image_object) {

    line(frame, image_object.at(0), image_object.at(1), Scalar(0, 255, 255), 3);
    line(frame, image_object.at(1), image_object.at(2), Scalar(0, 255, 255), 3);
    line(frame, image_object.at(2), image_object.at(0), Scalar(0, 255, 255), 3);
    line(frame, image_object.at(3), image_object.at(4), Scalar(0, 255, 255), 3);
    line(frame, image_object.at(4), image_object.at(5), Scalar(0, 255, 255), 3);
    line(frame, image_object.at(5), image_object.at(3), Scalar(0, 255, 255), 3);
    line(frame, image_object.at(0), image_object.at(3), Scalar(0, 255, 255), 3);
    line(frame, image_object.at(1), image_object.at(4), Scalar(0, 255, 255), 3);
    line(frame, image_object.at(2), image_object.at(5), Scalar(0, 255, 255), 3);

}

// Function to get vertex and vertex indices from .obj file
void getObjectParams(char* filename, vector< Point3f > &vertices, vector< Vec3i > &vertexIndices) {
    
    FILE *fp;
    // char filename[255] = "diamond.obj";
    // char filename[255] = "teddybear.obj";    
    // char filename[255] = "skyscraper.obj";
    // char filename[255] = "spaceshuttle.obj";

    double scaling_factor = 1;

    if ( strcmp( filename, "diamond.obj" ) == 0 ) {
        scaling_factor = 45;
    }

    if ( strcmp( filename, "skyscraper.obj" ) == 0 ) {
        scaling_factor = 7;
    }

    if ( strcmp( filename, "teddybear.obj" ) == 0 ) {
        scaling_factor = 5;
    }

    if ( strcmp( filename, "spaceshuttle.obj" ) == 0 ) {
        scaling_factor = 2;
    }

    fp = fopen( filename, "r" );

    if(!fp) {
        printf("Unable to open output file %s\n", filename );
        exit(-1);
    }

    while(true) {

        char lineHeader[128];

        // Read the first word of the line
        int res = fscanf(fp, "%s", lineHeader);
        
        if (res == EOF)
            // EOF = End Of File. Quit the loop.
            break;

        // else : parse lineHeader
        if ( strcmp( lineHeader, "v" ) == 0 ){
            Point3f vertex;
            fscanf(fp, "%f %f %f\n", &vertex.x, &vertex.y, &vertex.z );
            vertex.x /= scaling_factor;
            vertex.y /= scaling_factor;
            vertex.z /= scaling_factor;
            vertex.x += 3;
            vertex.y += 4;
            vertex.z += 3;
            vertices.push_back(vertex);

        }

    }

    fp = fopen( filename, "r" );

    if(!fp) {
        printf("Unable to open output file %s\n", filename );
        exit(-1);
    }

    while(true) {

        char lineHeader[128];

        // Read the first word of the line
        int res = fscanf(fp, "%s", lineHeader);
        
        if (res == EOF)
            // EOF = End Of File. Quit the loop.
            break;

        if ( strcmp( lineHeader, "f" ) == 0 ) {

            string vertex1, vertex2, vertex3;
            Vec3i vertexIndex;
            int matches = fscanf(fp, "%d %d %d\n", &vertexIndex[0], &vertexIndex[1], &vertexIndex[2] );
            vertexIndex[0] -= 1;
            vertexIndex[1] -= 1;
            vertexIndex[2] -= 1;
            vertexIndices.push_back(Vec3i(vertexIndex));
        }
        
    }

}

// Function to actually draw and project the parameters obtained from .obj file
void drawObject(Mat &frame, vector<Point2f> &image_points, vector<Vec3i> &vertexIndices) {

    for(int i = 0; i < vertexIndices.size(); i++) {

        line(frame, image_points.at(vertexIndices[i][0]), image_points.at(vertexIndices[i][1]), Scalar(0, 255, 255), 1);
        line(frame, image_points.at(vertexIndices[i][1]), image_points.at(vertexIndices[i][2]), Scalar(0, 255, 255), 1);
        line(frame, image_points.at(vertexIndices[i][2]), image_points.at(vertexIndices[i][0]), Scalar(0, 255, 255), 1);

    }

}