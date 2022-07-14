import mediapipe as mp
import cv2
from mediapipe.framework.formats import landmark_pb2

mp_drawing = mp.solutions.drawing_utils
mp_holistic = mp.solutions.holistic
mp_drawing.DrawingSpec(color=(0,0,255), thickness=2, circle_radius=2)

cap = cv2.VideoCapture(0)
Wx=1
Wy=1
def cv(x):
    s=x*90
    if s >  90:
        return 90
    if s < -90:
        return -90
    else:
        return int(s)

with mp_holistic.Holistic(min_detection_confidence=0.2, min_tracking_confidence=0.2) as holistic:
  while True:
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
        Xa1 = int(image.shape[1] * landmark_pose_subset.landmark[0].x) # Vai
        Ya1 = int(image.shape[1] * landmark_pose_subset.landmark[0].y)
        Xb1 = int(image.shape[1] * landmark_pose_subset.landmark[2].x)  # Khuu Tay
        Yb1 = int(image.shape[1] * landmark_pose_subset.landmark[2].y)
        Xc1 = int(image.shape[1] * landmark_pose_subset.landmark[4].x)  # Co Tay
        Yc1 = int(image.shape[1] * landmark_pose_subset.landmark[4].y)


        # print((Xc1-Xa1),(Yc1-Ya1))
        #
        # print("-----------------")

        cv2.line(image,
             (int(image.shape[1] * landmark_pose_subset.landmark[0].x), int(image.shape[0] * landmark_pose_subset.landmark[0].y)),
             (int(image.shape[1] * landmark_pose_subset.landmark[4].x), int(image.shape[0] * landmark_pose_subset.landmark[4].y)),
             (0, 255, 0), thickness=3, lineType=8)
        # cv2.line(image,
        #      (int(image.shape[1] * landmark_pose_subset.landmark[2].x), int(image.shape[0] * landmark_pose_subset.landmark[2].y)),
        #      (int(image.shape[1] * landmark_pose_subset.landmark[4].x), int(image.shape[0] * landmark_pose_subset.landmark[4].y)),
        #      (0, 255, 0), thickness=3, lineType=8)

        custom_right_hand_landmark = []
        if results.right_hand_landmarks is not None:
            custom_right_hand_landmark.append(results.right_hand_landmarks.landmark[0])
            custom_right_hand_landmark.append(results.right_hand_landmarks.landmark[4])
            custom_right_hand_landmark.append(results.right_hand_landmarks.landmark[20])


            landmark_right_hand_subset = landmark_pb2.NormalizedLandmarkList(landmark=custom_right_hand_landmark)
            cv2.line(image,
                        (int(image.shape[1] * landmark_right_hand_subset.landmark[0].x), int(image.shape[0] * landmark_right_hand_subset.landmark[0].y)),
                        (int(image.shape[1] * landmark_right_hand_subset.landmark[2].x), int(image.shape[0] * landmark_right_hand_subset.landmark[2].y)),
                        (0, 255, 0), thickness=3, lineType=8)
            cv2.line(image,
                         (int(image.shape[1] * landmark_right_hand_subset.landmark[0].x), int(image.shape[0] * landmark_right_hand_subset.landmark[0].y)),
                         (int(image.shape[1] * landmark_right_hand_subset.landmark[1].x), int(image.shape[0] * landmark_right_hand_subset.landmark[1].y)),
                         (0, 255, 0), thickness=3, lineType=8)
            cv2.circle(image,
                         (int(image.shape[1] * landmark_pose_subset.landmark[0].x), int(image.shape[0] * landmark_pose_subset.landmark[0].y)),
                         abs(Wy), (255, 0, 0), 2)

        Mx = cv((Xc1 - Xa1) / Wy)
        My = cv((Yc1 - Ya1) / Wy)


        print(My,Mx)
    if cv2.waitKey(5) & 0xFF == ord('b'):
        Wx = Xc1 - Xa1
        Wy = Yc1 - Ya1
    cv2.imshow('frame',cv2.flip(image, 1))
    if cv2.waitKey(5) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()