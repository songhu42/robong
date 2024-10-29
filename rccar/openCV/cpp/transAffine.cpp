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
    Mat src = imread("/home/song/Downloads/lenna.bmp", IMREAD_GRAYSCALE); 
    
    Mat dst, dst2;
    Point2f srcPts[3], dstPts[3];
    srcPts[0] = Point2f(0, 0);
    srcPts[1] = Point2f(src.cols - 1, 0);
    srcPts[2] = Point2f(src.cols - 1, src.rows - 1);
    dstPts[0] = Point2f(src.cols / 2, 0);
    dstPts[1] = Point2f(src.cols - 50, 0);
    dstPts[2] = Point2f(src.cols - 1, src.rows - 1);

    // 3점 기준으로 이미지 변환 처리 .. 
    Mat M = getAffineTransform(srcPts, dstPts);
    warpAffine(src, dst, M, Size());

    // 이미지 회전 처리 .. 
    Point2d cp(src.cols / 2., src.rows / 2.);
    Mat M2 = getRotationMatrix2D(cp, 30, 0.6);
    warpAffine(src, dst2, M2, Size());

    imshow("src", src);
    imshow("dst", dst);
    imshow("dst2", dst2);

    waitKey(0); 

    return 0; 
}