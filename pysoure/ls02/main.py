## Import Modules
import cv2
import numpy as np 


cap = cv2.VideoCapture(0)

if cap.isOpened() == False:
    print("ERROR!")

while cap.isOpened():
    ret, frame = cap.read()
    frame = cv2.flip(frame, 1)

    if ret:
        ## BEGIN
        cv2.imshow("FrameWindow", frame)
        ## END
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
    else:
        break


cap.release()
cv2.destroyAllWindows()



if __name__ == "__main__":
    pass