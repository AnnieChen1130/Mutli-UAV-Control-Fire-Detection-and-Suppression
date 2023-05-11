import cv2
import numpy as np
import math

fileName = '82721fliped.avi' #'2021-09-10_08_53_35.avi'
cap = cv2.VideoCapture(fileName)

lower_red = np.array([0, 100, 100])
upper_red = np.array([179, 255, 255])

fps = cap.get(cv2.CAP_PROP_FPS)
w1 = cap.get(3)
h1 = cap.get(4)
centerx = w1/2
centery = h1/2
roistartw = cap.get(3) / 10
roistarth = cap.get(4) / 10
print(str(round(w1/2)) + " " + str(round(h1/2)))

print(cap.get(3))
print(cap.get(4))
pixel_len = 0.7372549 #cm and if object is drone is 8 meter altitutde
N = 0
E = 0
flag = 0
while True:
    ret, frame = cap.read()
    #frame = frame[int(roistarth) * 4:int(h1) - int(roistarth) * 4, int(roistartw) * 4:int(w1) - int(roistartw) * 4]
    frame1 = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(frame1, lower_red, upper_red)
    res = cv2.bitwise_and(frame1, frame1, mask=mask)
    gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)

    #cv2.imshow("og", frame)
    #frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    #ret, frame = cv2.threshold(frame, 255 * .6, 255, cv2.THRESH_BINARY)
    #print(cv2.countNonZero(frame))
    #cv2.imshow("binary", frame)
    nlabels, labels, stats, centroids = cv2.connectedComponentsWithStats(gray, None, None, None, 8, cv2.CV_32S)
    #cv2.imshow("connected", frame)
    contours, hierarchy = cv2.findContours(gray, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    print("Number of Countours: " + str(len(contours)))
    #cv2.imshow("countor", frame)
    if len(contours) == 1:
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            #if(w > 50 and h > 50):
            #print("x " + str(x))
            #print("y " + str(y))
            print("w " + str(w))
            print("h " + str(h))
            distance_w = w * pixel_len
            distance_h = h * pixel_len
            print('w ' + str(distance_w) + ' ' + 'h ' + str(distance_h))
            cv2.rectangle(frame, (x, y), (x + w, y + h), (255, 0, 0), 2)
            cx = round(centroids[1][0])
            cy = round(centroids[1][1])
            cv2.line(frame, (round(centerx), round(centery)), (cx, cy), color=(255, 0, 0), thickness=5)
            distance = math.sqrt(((centerx - cx) ** 2) + ((centery - cy) ** 2))
            print("line distance: " + str(distance*pixel_len))
            print("cy:" + str(cy) + " cx:" + str(cx))
            if (cy < centery and cx < centerx): #I
                #print("I")
                flag = flag + 1
                N = (abs(cy-centery) * pixel_len)/100
                E = -(abs(cx-centerx) * pixel_len)/100
            elif (cy < centery and cx >= centerx): #II
                #print("II")
                flag = flag + 1
                N = (abs(cy-centery) * pixel_len)/100
                E = (abs(cx-centerx) * pixel_len)/100
            elif (cy >= centery and cx < centerx): #III
                #print("III")
                flag = flag + 1
                N = -(abs(cy-centery) * pixel_len)/100
                E = -(abs(cx-centerx) * pixel_len)/100
            elif (cy >= centery and cx >= centerx): #IV
                #print("IV")
                flag = flag + 1
                N = -(abs(cy-centery) * pixel_len)/100
                E = (abs(cx-centerx) * pixel_len)/100
            print("N: " + str(N) + " " + "E:" + str(E))
            #if flag == 5:
                #cv2.imwrite('centering.png', frame)
                #print("flags measuerments")
                #print(distance*pixel_len)
                #print(N)
                #print(E)
    #cv2.imshow("rec", frame)
    #frame = cv2.circle(frame, (round(w1/2), round(h1/2)), radius=0, color=(255, 0, 0), thickness=5)
    #frame = cv2.line(frame, (240, 0), (240, 320), color=(255, 0, 0), thickness=5)
    #frame = cv2.line(frame, (0, 160), (480, 160), color=(255, 0, 0), thickness=5)
    #cv2.imwrite('centering.png', frame)
    #frame = cv2.circle(frame, (314, 35), radius=0, color=(255, 0, 0), thickness=5)
    #cv2.imshow("point", frame)
    cv2.waitKey(int((1 / int(fps)) * 1000))


cv2.waitKey(0)
cv2.destroyAllWindows()
