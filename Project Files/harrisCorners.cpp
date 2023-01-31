#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>

using namespace cv;
using namespace std;

int main() {

    // Intialisation to store Image:
    Mat src, src_gray;

    // Reading the Image:
    src = imread( "pic.1005.jpg" );
    
    // Converting to Grayscale
    cvtColor( src, src_gray, COLOR_BGR2GRAY );
    
    // Initalising Parameters for Harris Corner Function:
    int threshold = 200;
    int blockSize = 2;
    int apertureSize = 7;
    double k = 0.04;
    Mat dst = Mat::zeros( src.size(), CV_32FC1 );
    
    // Calling Harris Corner Function:
    cornerHarris( src_gray, dst, blockSize, apertureSize, k );
    
    // Initialising Matrix to draw and display Harris Corners
    Mat dst_norm, dst_norm_scaled;
    
    // Processing the image
    normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
    convertScaleAbs( dst_norm, dst_norm_scaled );
    
    // Drawing Harris Corners
    for( int i = 0; i < dst_norm.rows ; i++ )
    {
        for( int j = 0; j < dst_norm.cols; j++ )
        {
            if( (int) dst_norm.at<float>(i,j) > threshold )
            {
                circle( dst_norm_scaled, Point(j,i), 5,  Scalar(255, 255, 255), 2, 8, 0 );
            }
        }
    }

    // Displaying the Images
    namedWindow( "Original" );
    imshow( "Original", src );
    namedWindow( "Harris Corners" );
    imshow( "Harris Corners", dst_norm_scaled );
    waitKey();

    destroyAllWindows();
    
    return 0;
}