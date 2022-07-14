import cv2

cam = cv2.VideoCapture(0)
if cam.isOpened():
    while True:
        rval, frame = cam.read()
        # cv2.normalize(
        #     frame, None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1
        # )
        cv2.imshow("1", cv2.flip(frame, 1))

        key = cv2.waitKey(20)
        if key == 27:  # exit on ESC
            break
cv2.destroyAllWindows()