#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/freetype.hpp" // 한글 폰트를 위해 추가 

using namespace cv; 
using namespace std; 

void onMouse(int event, int x, int y, int flags, void *data); 

void onMouse(int event, int x, int y, int flags, void *data) {
    Mat *img = (Mat *)data; 
    Mat movImg; 
    movImg = (*img).clone();

    switch(event) {
        case EVENT_LBUTTONDOWN:
            cout << x << ", " << y << endl; 
            circle(*img, Point(x, y), 10, Scalar(255, 0, 0), 2); 
            imshow("img3", *img); 
            break; 
        case EVENT_MOUSEMOVE :
            rectangle(movImg, Point(x-15, y-15), Point(x+15, y+15), Scalar(0, 0, 200), 3); 
            imshow("img3", movImg); 
            break; 
    }
} 

int main(int argc, char* args[] ) {
    cout << "Hello OpenCV!" << endl; 
    Mat img = imread("/home/song/Downloads/lenna.bmp", IMREAD_GRAYSCALE); 

    Mat noise(img.size(), CV_32SC1); 
    randn(noise, 0, 10); 
    Mat ori = img.clone(); 
    add(img, noise, img, Mat(), CV_8U); 

    cout << "Hello OpenCV!" << endl; 
    
    Mat dest1, dest2; 
    int sigma = 3; 

    // 가우시안 필터 => 잡음 제거, 에지 약해짐 
    // 양방향 필터 : 잡음 제거, 에지 유지 
    GaussianBlur(img, dest1, Size(0, 0), sigma); 
    bilateralFilter(img, dest2, -1, 10, 5); 

    imshow("ori", ori); 
    imshow("img", img); 
    imshow("gaussian", dest1); 
    imshow("bilater", dest2); 

    
    waitKey(0); 

    return 0; 
}