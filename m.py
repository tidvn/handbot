import numpy as np
import mediapipe as mp
import cv2
from mediapipe.framework.formats import landmark_pb2
import serial
import time
import struct

prev_frame_time = 0
new_frame_time = 0
# arduino = serial.Serial(port='/dev/ttyUSB0', baudrate=9600, timeout=.1)

mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic
mp_drawing.DrawingSpec(color=(0, 0, 255), thickness=2, circle_radius=2)

cap = cv2.VideoCapture(0)
Wx = 1
Wy = 180


def cvx(x):
    if x > 90:
        return 90
        return -90
    else:
        return x


def get_angle(a, b, c):
    a = np.array(a)
    b = np.array(b)
    c = np.array(c)
    rad = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(rad * 180.0 / np.pi)
    if (angle > 180):
        angle = 360 - angle
    return angle


with mp_holistic.Holistic(min_detection_confidence=0.2, min_tracking_confidence=0.2) as holistic:
    while True:
        t1 = time.time()
        new_frame_time = time.time()
        fps = 1 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time
        ret, frame = cap.read()
        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = holistic.process(image)
        custom_pose_landmark = []
        if results.pose_landmarks is not None:
            for i in range(12, 22):
                custom_pose_landmark.append(results.pose_landmarks.landmark[i])

            landmark_pose_subset = landmark_pb2.NormalizedLandmarkList(landmark=custom_pose_landmark)

            image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            # mp_drawing.draw_landmarks(image=image, landmark_list=landmark_pose_subset)
            Xa1 = int(image.shape[1] * landmark_pose_subset.landmark[0].x)  # Vai
            Ya1 = int(image.shape[1] * landmark_pose_subset.landmark[0].y)
            Xb1 = int(image.shape[1] * landmark_pose_subset.landmark[2].x)  # Khuu Tay
            Yb1 = int(image.shape[1] * landmark_pose_subset.landmark[2].y)
            Xc1 = int(image.shape[1] * landmark_pose_subset.landmark[4].x)  # Co Tay
            Yc1 = int(image.shape[1] * landmark_pose_subset.landmark[4].y)

            cv2.line(image,
                     (int(image.shape[1] * landmark_pose_subset.landmark[0].x),
                      int(image.shape[0] * landmark_pose_subset.landmark[0].y)),
                     (int(image.shape[1] * landmark_pose_subset.landmark[4].x),
                      int(image.shape[0] * landmark_pose_subset.landmark[4].y)),
                     (0, 255, 0), thickness=3, lineType=8)

            custom_right_hand_landmark = []
            if results.right_hand_landmarks is not None:
                custom_right_hand_landmark.append(results.right_hand_landmarks.landmark[0])
                custom_right_hand_landmark.append(results.right_hand_landmarks.landmark[4])
                custom_right_hand_landmark.append(results.right_hand_landmarks.landmark[8])

                landmark_right_hand_subset = landmark_pb2.NormalizedLandmarkList(landmark=custom_right_hand_landmark)

                T0x = int(image.shape[1] * landmark_right_hand_subset.landmark[0].x)
                T0y = int(image.shape[0] * landmark_right_hand_subset.landmark[0].y)
                T1x = int(image.shape[1] * landmark_right_hand_subset.landmark[1].x)
                T1y = int(image.shape[0] * landmark_right_hand_subset.landmark[1].y)
                T2x = int(image.shape[1] * landmark_right_hand_subset.landmark[2].x)
                T2y = int(image.shape[0] * landmark_right_hand_subset.landmark[2].y)

                T0 = [landmark_right_hand_subset.landmark[0].x, landmark_right_hand_subset.landmark[0].y]
                T1 = [landmark_right_hand_subset.landmark[1].x, landmark_right_hand_subset.landmark[1].y]
                T2 = [landmark_right_hand_subset.landmark[2].x, landmark_right_hand_subset.landmark[2].y]

                cv2.line(image,
                         (int(image.shape[1] * landmark_right_hand_subset.landmark[0].x),
                          int(image.shape[0] * landmark_right_hand_subset.landmark[0].y)),
                         (int(image.shape[1] * landmark_right_hand_subset.landmark[2].x),
                          int(image.shape[0] * landmark_right_hand_subset.landmark[2].y)),
                         (0, 255, 0), thickness=3, lineType=8)
                cv2.line(image,
                         (int(image.shape[1] * landmark_right_hand_subset.landmark[0].x),
                          int(image.shape[0] * landmark_right_hand_subset.landmark[0].y)),
                         (int(image.shape[1] * landmark_right_hand_subset.landmark[1].x),
                          int(image.shape[0] * landmark_right_hand_subset.landmark[1].y)),
                         (0, 255, 0), thickness=3, lineType=8)

                cv2.circle(image,
                           (int(image.shape[1] * landmark_pose_subset.landmark[0].x),
                            int(image.shape[0] * landmark_pose_subset.landmark[0].y)),
                           abs(Wy), (255, 0, 0), 2)
                Mx = cvx(int(((Xc1 - Xa1) / Wy) * 90)) + 90
                My = cvx(int(((Yc1 - Ya1) / Wy) * 90)) + 90

                s0 = Mx
                if (My < 90):
                    s1 = 90
                    s2 = My + 90
                    s3 = 90
                elif (My > 90):
                    s1 = int(((My - 90) / 3) + 90)
                    s2 = 180
                    s3 = int(((My - 90) / 9) + 90)
                else:
                    s1 = 90
                    s2 = 180
                    s3 = 90
                if (get_angle(T1, T0, T2) > 30 ):
                    s4 = 90
                else:
                    s4 = 180
                print(s0,s1,s2,s3,s4)

                # arduino.write(struct.pack('>BBBBB', s0,s1,s2,s3,s4))



                t2 = 1 / (time.time() - t1)

        if cv2.waitKey(5) & 0xFF == ord('b'):
            Wx = Xc1 - Xa1
            Wy = Yc1 - Ya1

        cv2.putText(image, str(int(fps)), (7, 70), cv2.FONT_HERSHEY_SIMPLEX, 3, (100, 255, 0), 3, cv2.LINE_AA)
        # cv2.imshow('frame', cv2.flip(image, 1))
        cv2.imshow('frame', image)

        if cv2.waitKey(5) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()