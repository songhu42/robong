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
    
    Mat img2(400, 640, CV_8UC1); 
    Mat img3(400, 640, CV_8UC3); // 3 channels 
    Mat img4(400, 640, CV_8UC1, Scalar(100)); 

    // draw line... 
    line(img3, Point(10, 10), Point(200, 200), Scalar(255, 0, 0), 5); 
    circle(img3, Point(100, 100), 50, Scalar(255, 0, 0), 3); 

    // 한글 폰트를 위해 추가 
    String txt = u8"안녕? 오픈 CV!"; 
    auto ft2 = freetype::createFreeType2(); 
    ft2->loadFontData("/home/song/Downloads/NanumPen.ttf", 0); 
    Size textSize = ft2->getTextSize(txt, 50, -1, 0); 
    ft2->putText(img3, txt, Point(80, 40), 50, Scalar(0, 255, 0), -1, LINE_AA, true); 
    putText(img3, "This is Circle", Point(80, 130), FONT_ITALIC, 4, Scalar(0, 255, 0), 3); 


    putText(img3, "This is Circle", Point(80, 130), FONT_ITALIC, 4, Scalar(0, 255, 0), 3); 

    namedWindow("img3"); 

    imshow("img3", img3); 

    // mouse event 
    setMouseCallback("img3", onMouse, (void*)&img3); 

    waitKey(0); 

    return 0; 
}