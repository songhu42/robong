#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/freetype.hpp" // 한글 폰트를 위해 추가 

using namespace cv; 
using namespace std; 

void onMouse(int event, int x, int y, int flags, void *data); 


Mat src, origin; 
bool flag = false; 
Point2f srcPts[4], destPts[4];

int main(int argc, char* args[] ) {
    cout << "Hello OpenCV!" << endl; 
    src = imread("/home/song/Downloads/card.bmp", IMREAD_COLOR); 
    destPts[0] = Point2f(0, 0); 
    destPts[1] = Point2f(200-1, 0); 
    destPts[2] = Point2f(200-1, 300-1); 
    destPts[3] = Point2f(0, 300-1); 

    origin = src.clone(); 

    imshow("src", src); 
    // mouse event 
    namedWindow("src");
    setMouseCallback("src", onMouse, (void*)&src); 

    waitKey(0); 

    return 0; 
}

void transferCard() {
    Mat dst;
    // 3점 기준으로 이미지 변환 처리 .. 
    Mat M = getPerspectiveTransform(srcPts, destPts);
    warpPerspective(origin, dst, M, Size(200, 300));

    imshow("card", dst); 
}

void onMouse(int event, int x, int y, int flags, void *data) {
    static int cnt = 0; 
    Mat *img = (Mat *)data; 

    switch(event) {
        case EVENT_LBUTTONDOWN:
            cout << x << ", " << y << endl; 
            srcPts[cnt++] = Point(x, y); 
            cout << "Add Point : " << cnt << endl; 
            circle(*img, Point(x, y), 3, Scalar(255, 0, 0), 2); 
            if( cnt == 4 ) {
                transferCard(); 
                cnt = 0;
            } 
            imshow("src", *img);

            break; 
    }
} 