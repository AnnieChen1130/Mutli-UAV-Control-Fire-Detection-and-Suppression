import numpy as np
import cv2
#import imutils


scaling_factorx=0.5
scaling_factory=0.5


cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

print(cap.get(3))
print(cap.get(4))

writer = cv2.VideoWriter('11.avi', cv2.VideoWriter_fourcc(*"MJPG"), 20, (720, 480))

print(cap.get(3))
print(cap.get(4))

while (True):
    # Capture frame-by-frame
    ret, frame = cap.read()  # ret = 1 if the video is captured; frame is the image

    #frame = cv2.resize(frame, None, fx=scaling_factorx, fy=scaling_factory, interpolation=cv2.INTER_AREA)
    #frame = imutils.resize(frame, width=320)
    # Our operations on the frame come here
    #img = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    writer.write(frame)
    # Display the resulting image
    cv2.imshow('Video Capture', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):  # press q to quit
        status = cv2.imwrite('img.jpg', frame)
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()