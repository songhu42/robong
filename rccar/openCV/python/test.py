import cv2
import numpy as np 

def onMouse(event, x, y, flags, param):
    img = param["img"]
    if event == cv2.EVENT_LBUTTONDOWN:
        print(x, y)
        cv2.circle(img, (x, y), 20, (255,0,0), 2)
        cv2.imshow("img2", img)

def main() :
    img = cv2.imread("../cpp/lena.png")
    cv2.imshow("image", img)

    # 희색 Mat 동일 코드 
    img2 = np.full((400, 640, 3), 255, dtype=np.uint8)
    cv2.line(img2, (10, 10), (200, 200), (255, 0, 0), 3)
    cv2.circle(img2, (100, 100), 50, (0, 255, 0), 2)
    cv2.putText(img2, "한글은", (30, 40), cv2.FONT_HERSHEY_COMPLEX, 3, (0, 255, 255))

    # pointer 형식으로 넘기기 위해 데이터를 dictionary로 넘겨야 .. 
    param = {"img":img2}
    cv2.namedWindow("img2")
    cv2.setMouseCallback("img2", onMouse, param)
    cv2.imshow("img2", img2)

    cv2.waitKey(0)

if __name__ == "__main__" :
    main()

print ("exit program.")
