// Project 4 of CS 5330: Pattern Recognition and Computer Vision
// Created by Dhruvil Parikh

#include "calibration.hpp"

int main(){
    VideoCapture *cap;
    cap = new VideoCapture(0);    

    if( !cap->isOpened() ) {
        printf("Unable to open video device\n");
        return(-1);
    }

    Size refS( (int) cap->get(CAP_PROP_FRAME_WIDTH ), (int) cap->get(CAP_PROP_FRAME_HEIGHT));
    printf("Expected size: %d %d\n", refS.width, refS.height);

    Mat frame;
    Mat frame_gray;

    namedWindow("Video Stream", 1);

    int cnt_save = 0;
    bool flag_save = 0;
    bool flag_exit = 0;
    bool flag_a = 0;
    bool flag_b = 0;
    bool flag_c = 0;
    bool flag_d = 0;
    bool flag_e = 0;
    bool flag_f = 0;

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

    // Reading the Calibration Data
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

    // Creating points for 3D axes
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

    // Creating points for Triangular Prism
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

    char filename[255] = "teddybear.obj";

    cout << "Usage:" << endl;
    cout << "Teddy Bear is displayed by default :)" << endl;
    cout << "1) Press \"a\" to insert axes." << endl;
    cout << "2) Press \"b\" to insert traingular prism." << endl;
    cout << "3) Press \"c\" to insert diamond." << endl;
    cout << "4) Press \"d\" to insert teddybear." << endl;
    cout << "5) Press \"e\" to insert skyscraper." << endl;
    cout << "6) Press \"f\" to insert space shuttle." << endl;

    // Getting vertices and vertex indices from .obj file
    getObjectParams(filename, world_points, vertexIndices);

    // // Printing the vertices or the world points:
    // for(int i = 0; i < world_points.size(); i++) {
    //     cout << world_points[i].x << " " << world_points[i].y << " " << world_points[i].z << endl;
    // }

    // // Printing the set of indices collected from the faces that need to be joined:
    // for(int i = 0; i < vertexIndices.size(); i++) {
    //     cout << vertexIndices[i][0] << " " << vertexIndices[i][1] << " " << vertexIndices[i][2] << endl;
    // }

    // Starting the video feed
    while (true) {

        *cap >> frame;
        if( frame.empty() ) {
            printf("Frame is empty.\n");
            break;
        }

        // ########################################################### extractCorners(frame);

        cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

        bool pattern_found = findChessboardCorners(frame_gray, pattern_size, corner_set, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

        if(pattern_found)
            cornerSubPix(frame_gray, corner_set, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

        // Drawing the chessboard corners
        drawChessboardCorners(frame, pattern_size, Mat(corner_set), pattern_found);

        if(pattern_found) {

            if(flag_a) {
                    // Drawing the Axes
                    solvePnP(point_set, corner_set, camera_matrix, dist_coeffs, rvec, tvec);
                    projectPoints(object_set, rvec, tvec, camera_matrix, dist_coeffs, image_set);
                    drawAxesCheckerboard(frame, image_set);
            }

            else if(flag_b) {
                    // Drawing the Figure
                    solvePnP(point_set, corner_set, camera_matrix, dist_coeffs, rvec, tvec);
                    projectPoints(world_object, rvec, tvec, camera_matrix, dist_coeffs, image_object);
                    drawTraingularPrism(frame, image_object);
            }

            else {

                    solvePnP(point_set, corner_set, camera_matrix, dist_coeffs, rvec, tvec);
                    
                    // // Printing the rvec and the tvec:
                    // cout << "Printing the rvec:" << endl;
                    // cout << rvec.at<double>(0, 0) << " " << rvec.at<double>(0, 1) << " " << rvec.at<double>(0, 2) << endl;
                    // cout << "Printing the tvec:" << endl;
                    // cout << tvec.at<double>(0, 0) << " " << tvec.at<double>(0, 1) << " " << tvec.at<double>(0, 2) << endl;

                    // Drawing the Object
                    projectPoints(world_points, rvec, tvec, camera_matrix, dist_coeffs, image_points);
                    drawObject(frame, image_points, vertexIndices);

            }

        }

        imshow("Video Stream", frame);

        if(flag_save) {

            if(pattern_found) {
                
                imwrite("Image_" + to_string(cnt_save) + ".jpg", frame);
                cnt_save++;

                cout << "Frame " << cnt_save << " successfully saved." << endl;

            } else {
                cout << "Please make sure the checkerboard is clearly visible." << endl;
            }

        }

        switch(waitKey(10)) {

            case 'q':
                cout << "Exiting the Program..." << endl;
                flag_exit = 1;
                break;

            case 's':
                flag_save = 1;
                break;

            case 'a':
                flag_a = 1;
                flag_b = 0;
                break;
            
            case 'b':
                flag_a = 0;
                flag_b = 1;
                break;
                
            case 'c':
                flag_a = 0;
                flag_b = 0;
                strcpy(filename, "diamond.obj");
                world_points.clear();
                vertexIndices.clear();
                getObjectParams(filename, world_points, vertexIndices);
                break;
            
            case 'd':
                flag_a = 0;
                flag_b = 0;
                strcpy(filename, "teddybear.obj");
                world_points.clear();
                vertexIndices.clear();
                getObjectParams(filename, world_points, vertexIndices);
                break;
            
            case 'e':
                flag_a = 0;
                flag_b = 0;
                strcpy(filename, "skyscraper.obj");
                world_points.clear();
                vertexIndices.clear();
                getObjectParams(filename, world_points, vertexIndices);
                break;
            
            case 'f':
                flag_a = 0;
                flag_b = 0;
                strcpy(filename, "spaceshuttle.obj");
                world_points.clear();
                vertexIndices.clear();
                getObjectParams(filename, world_points, vertexIndices);
                break;

            default:
                flag_save = 0;
        }

        if (flag_exit) {
            break;
        }

    }

    delete cap;
    return 0;

}