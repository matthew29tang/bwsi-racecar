import cv2
import numpy as np

cap = cv2.VideoCapture(0)

def nothing(x):
    pass
# Creating a window for later use
cv2.namedWindow('result')

# Starting with 100's to prevent error while masking
h,s,v = 100,100,100

# Creating track bar

cv2.createTrackbar('h1', 'result',0,179,nothing)
cv2.createTrackbar('s1', 'result',0,255,nothing)
cv2.createTrackbar('v1', 'result',0,255,nothing)

cv2.createTrackbar('h', 'result',0,179,nothing)
cv2.createTrackbar('s', 'result',0,255,nothing)
cv2.createTrackbar('v', 'result',0,255,nothing)

while(1):

    _, frame = cap.read()

    #converting to HSV
    hsv = cv2.cvtColor(frame,cv2.COLOR_BGR2HSV)

    # get info from track bar and appy to result
    s = cv2.getTrackbarPos('s','result')
    h = cv2.getTrackbarPos('h','result')
    v = cv2.getTrackbarPos('v','result')
    h1 = cv2.getTrackbarPos('h1','result')
    s1 = cv2.getTrackbarPos('s1','result')
    v1 = cv2.getTrackbarPos('v1','result')

    print("lower = ", h,s,v)
    print("upper = ", h1,s1,v1)

    # Normal masking algorithm
    lower_blue = np.array([h,s,v])
    upper_blue = np.array([h1,s1,v1])

    mask = cv2.inRange(hsv,lower_blue, upper_blue)

    # result = cv2.bitwise_and(frame,frame,mask = mask)

    cv2.imshow('result', mask)

    k = cv2.waitKey(10) & 0xFF
    if k == 27:
        break

cap.release()

cv2.destroyAllWindows()
