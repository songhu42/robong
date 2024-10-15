import cv2
import numpy as np 

def main() :
    img = cv2.imread("../cpp/lena.png")
    cv2.imshow("image", img)

    # 희색 Mat 동일 코드 
    img2 = np.full((400, 640, 3), 255, dtype=np.uint8)
    cv2.line(img2, (10, 10), (200, 200), (255, 0, 0), 3)
    cv2.circle(img2, (100, 100), 50, (0, 255, 0), 2)
    cv2.putText(img2, "한글은", (30, 40), cv2.FONT_HERSHEY_COMPLEX, 3, (0, 255, 255))

    cv2.imshow("img2", img2)

    cv2.waitKey(0)

if __name__ == "__main__" :
    main()

print ("exit program.")
