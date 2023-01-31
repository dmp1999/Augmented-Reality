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
    int flag_calibrate = 0;
    int flag_write = 0;
    bool flag_exit = 0;

    int i = 0, j = 0, k = 0, c = 0;

    // Initializations for Task 1 ##############################################
    // Number of Interior Corners
    Size pattern_size(9, 6);
    // Vector will be filled by detected corners
    vector<Point2f> corner_set;

    // Initializations for Task 2 #############################################
    vector<Vec3f> point_set;
    vector<vector<Vec3f> > point_list;
    vector<vector<Point2f> > corner_list;

    // Defining the Point Set
    for(i = 0; i < 6; i++) {
        for(j = 0; j < 9; j++) {
            point_set.push_back(Vec3f(i, j, 0));
        }
    }

    // // Printing the Point Set
    // for(i = 0; i < 54; i++) {
    //         cout << point_set.at(i) << endl;
    // }

    // Initializations for Task 3 #######################################
    double center_cols = refS.width/2;
    double center_rows = refS.height/2;
    double data[3][3] = {{1, 0, center_cols}, {0, 1, center_rows}, {0, 0, 1}};
    Mat camera_matrix(3, 3, CV_64FC1, &data);
    // Printing out the values of the Camera Matrix
    for(i = 0; i < camera_matrix.rows; i++) {
        for(j = 0; j < camera_matrix.cols; j++) {
            cout << camera_matrix.at<double>(i, j) << " ";
        }
        cout << endl;
    }

    // Distortion Coefficients
    Mat dist_coeffs;
    dist_coeffs = Mat::zeros(5, 1, CV_64F);
    vector<Mat> rvecs, tvecs;

    // Starting the video feed
    while (true) {

        *cap >> frame;
        if( frame.empty() ) {
            printf("Frame is empty.\n");
            break;
        }

        // ########################################################### extractCorners(frame);

        cvtColor(frame, frame_gray, COLOR_BGR2GRAY);

        //CALIB_CB_FAST_CHECK saves a lot of time on images that do not contain any chessboard corners

        bool pattern_found = findChessboardCorners(frame_gray, pattern_size, corner_set, CALIB_CB_ADAPTIVE_THRESH + CALIB_CB_NORMALIZE_IMAGE + CALIB_CB_FAST_CHECK);

        if(pattern_found)
            cornerSubPix(frame_gray, corner_set, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

        // Drawing the chessboard corners
        drawChessboardCorners(frame, pattern_size, Mat(corner_set), pattern_found);

        // // Printing the size of the corner_set and the first coordinate
        // cout << corner_set.size() << endl;
        // if(corner_set.size()) {
        //     cout << "X - Coordinate: " << corner_set[0].x << endl;
        //     cout << "Y - Coordinate: " << corner_set[0].y << endl;
        // }

        imshow("Video Stream", frame);

        if(flag_save) {

            if(pattern_found) {
                
                imwrite("/home/dhruvil/PRCV/Project4/Calibration_Images/Image_" + to_string(cnt_save) + ".jpg", frame);
                cnt_save++;

                cout << "Frame " << cnt_save << " successfully saved for Calibration." << endl;

                // Saving the corner_set into the corner_list and the point set into the point list
                corner_list.push_back(vector<Point2f>(corner_set));
                point_list.push_back(vector<Vec3f>(point_set));
            
            } else {
                cout << "Please make sure the checkerboard is clearly visible." << endl;
            }

        }

        if(flag_calibrate) {

            double error = calibrateCamera(point_list, corner_list, refS, camera_matrix, dist_coeffs, rvecs, tvecs, CALIB_FIX_ASPECT_RATIO + CALIB_FIX_K3);
            
            // Prinitng the Camera Matrix
            cout << "Printing the Camera Matrix:" << endl;
            for(i = 0; i < camera_matrix.rows; i++) {
                for(j = 0; j < camera_matrix.cols; j++) {
                    cout << camera_matrix.at<double>(i, j) << " ";
                }
                cout << endl;
            }

            // Prinitng the Distortion Matrix
            cout << "Printing the Distortion Matrix:" << endl;
            for(i = 0; i < dist_coeffs.rows; i++){
                cout << dist_coeffs.at<double>(i, 0) << " ";
            }
            cout << endl;

            // Priniting the Calibration Error
            cout << "Error: " << error << endl;

        }

        switch(waitKey(10)) {

            case 'q':
                cout << "Exiting the Program..." << endl;
                flag_exit = 1;
                break;

            case 's':
                flag_save = 1;
                break;

            case 'c':
                if(cnt_save < 5) {
                    cout << "Please choose atleast 5 frames for calibration." << endl;
                } else {
                    flag_calibrate = 1;
                    cout << "Calibrating the Camera..." << endl;
                }
                break;

            case 'w':
                writeData(camera_matrix, dist_coeffs);
                break;

            default:
                flag_save = 0;
                flag_calibrate = 0;
        }

        if (flag_exit) {
            break;
        }

    }

    delete cap;
    return 0;

}