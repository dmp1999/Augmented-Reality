// Project 4 of CS 5330: Pattern Recognition and Computer Vision
// Created by Dhruvil Parikh

#include "calibration.hpp"

int main(){

    Mat frame, original;
    frame = imread("checkerboard2.jpeg", IMREAD_COLOR);
    resize(frame, frame, Size(640, 480));
    frame.copyTo(original);
    Mat frame_gray;

    int i = 0, j = 0, k = 0, c = 0;

    // Initializations for Task 1 ##############################################
    // Number of Interior Corners
    Size pattern_size(9, 6);
    // Vector will be filled by detected corners
    vector<Point2f> corner_set;

    // Initializations for Task 2 #############################################
    vector<Vec3f> point_set;

    // Defining the Point Set
    for(i = 0; i < 6; i++) {
        for(j = 0; j < 9; j++) {
            point_set.push_back(Vec3f(i, j, 0));
        }
    }

    // Initializations for Task 3 #######################################
    // Camera Matrix
    Mat camera_matrix(3, 3, CV_64FC1);
    // Distortion Coefficients
    Mat dist_coeffs;
    dist_coeffs = Mat::zeros(5, 1, CV_64F);
    Mat rvec, tvec;

    // Reading Calibration Data
    readData(camera_matrix, dist_coeffs);

    // Prinitng the Camera Matrix
    cout << "Printing the Camera Matrix read from the calibration_data.csv file:" << endl;
    for(i = 0; i < camera_matrix.rows; i++) {
        for(j = 0; j < camera_matrix.cols; j++) {
            cout << camera_matrix.at<double>(i, j) << " ";
        }
        cout << endl;
    }

    // Prinitng the Distortion Matrix
    cout << "Printing the Distortion Matrix read from the calibration_data.csv file:" << endl;
    for(i = 0; i < dist_coeffs.rows; i++){
        cout << dist_coeffs.at<double>(i, 0) << " ";
    }
    cout << endl;

    // Initializations for Task 5 #######################################
    vector<Point2f> image_set;
    vector<Vec3f> object_set;

    for(i = 0; i < 2; i++) {
            object_set.push_back(Vec3f(i*2, 0, 0));
    }

    for(j = 0; j < 2; j++) {
            object_set.push_back(Vec3f(0, j*2, 0));
    }

    for(k = 0; k < 2; k++) {
            object_set.push_back(Vec3f(0, 0, k*2));
    }

    // Initializations for Task 6 #######################################
    vector<Point2f> image_object;
    vector<Vec3f> world_object;

    world_object.push_back(Vec3f(0, 4, 0));
    world_object.push_back(Vec3f(5, 0, 0));
    world_object.push_back(Vec3f(5, 8, 0));
    world_object.push_back(Vec3f(0, 4, 3));
    world_object.push_back(Vec3f(5, 0, 3));
    world_object.push_back(Vec3f(5, 8, 3));    

    // Initializations for Extension #######################################
    vector< Point3f > world_points;
    vector< Point2f > image_points;
    vector< Vec3i > vertexIndices;
    char filename[255];

    std::cout << "Usage:" << std::endl;
    std::cout << "1) Press \"a\" to insert axes." << std::endl;
    std::cout << "2) Press \"b\" to insert traingular prism." << std::endl;
    std::cout << "3) Press \"c\" to insert diamond." << std::endl;
    std::cout << "4) Press \"d\" to insert teddybear." << std::endl;
    std::cout << "5) Press \"e\" to insert skyscraper." << std::endl;
    std::cout << "6) Press \"f\" to insert space shuttle." << std::endl;

    cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

    bool pattern_found = findChessboardCorners(frame_gray, pattern_size, corner_set, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

    if(pattern_found)
        cornerSubPix(frame_gray, corner_set, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

    // Drawing the chessboard corners
    drawChessboardCorners(frame, pattern_size, Mat(corner_set), pattern_found);

    if(pattern_found) {

        solvePnP(point_set, corner_set, camera_matrix, dist_coeffs, rvec, tvec);
        
        char key;

        cout << "Press the desired key: ";
        cin >> key;
        
        switch(key) {

            case 'a':
                // Drawing the Axes
                projectPoints(object_set, rvec, tvec, camera_matrix, dist_coeffs, image_set);
                drawAxesCheckerboard(frame, image_set);
                break;
            
            case 'b':
                // Drawing the Figure
                projectPoints(world_object, rvec, tvec, camera_matrix, dist_coeffs, image_object);
                drawTraingularPrism(frame, image_object);
                break;
                
            case 'c':
                // Drawing the Object
                strcpy(filename, "diamond.obj");
                getObjectParams(filename, world_points, vertexIndices);
                projectPoints(world_points, rvec, tvec, camera_matrix, dist_coeffs, image_points);
                drawObject(frame, image_points, vertexIndices);
                break;
            
            case 'd':
                // Drawing the Object
                strcpy(filename, "teddybear.obj");
                getObjectParams(filename, world_points, vertexIndices);
                projectPoints(world_points, rvec, tvec, camera_matrix, dist_coeffs, image_points);
                drawObject(frame, image_points, vertexIndices);
                break;
            
            case 'e':
                // Drawing the Object
                strcpy(filename, "skyscraper.obj");
                getObjectParams(filename, world_points, vertexIndices);
                projectPoints(world_points, rvec, tvec, camera_matrix, dist_coeffs, image_points);
                drawObject(frame, image_points, vertexIndices);
                break;
            
            case 'f':
                // Drawing the Object
                strcpy(filename, "spaceshuttle.obj");
                getObjectParams(filename, world_points, vertexIndices);
                projectPoints(world_points, rvec, tvec, camera_matrix, dist_coeffs, image_points);
                drawObject(frame, image_points, vertexIndices);
                break;

        }

    } else {
        cout << "Please provide an image with visible checkerboard pattern." << endl;
        exit(-1);
    }

    imshow("Original", original);
    imshow("Inserted Object", frame);
    waitKey();
    imwrite("Virtual Object Inserted.jpg", frame);

    return 0;

}