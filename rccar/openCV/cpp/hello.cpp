#include <iostream>
#include "opencv2/opencv.hpp"

using namespace cv; 
using namespace std; 

int main(int argc, char* args[] ) {
    Mat img; 
    img = imread("../lena.png"); 
    imshow("Lena", img); 

    cout << "Hello OpenCV!" << endl; 
    
    Mat img1; 
    img1 = Scalar(10, 0, 0); 
    Mat img2(400, 640, CV_8UC1); 
    Mat img3(400, 640, CV_8UC3); 

    Mat img4(400, 640, CV_8UC1, Scalar(100)); 

    // imshow("img1", img1); 
    imshow("img2", img2); 
    imshow("img3", img3); 
    imshow("img4", img4); 

    waitKey(0); 

    return 0; 
}