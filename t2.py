from threading import Thread
import cv2, time
import mediapipe as mp
from mediapipe.framework.formats import landmark_pb2
import numpy as np
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles
mp_holistic = mp.solutions.holistic
mp_drawing.DrawingSpec(color=(0,0,255), thickness=2, circle_radius=2)



class VideoStreamWidget(object):
    def __init__ (self, cam):
        self.cam = cam
        self.capture = cv2.VideoCapture(cam)
        # Start the thread to read frames from the video stream
        self.thread = Thread(target=self.update, args=())
        self.thread.daemon = True
        self.thread.start()


    def update(self):
        # Read the next frame from the stream in a different thread
        while True:
            if self.capture.isOpened():
                (self.status, self.frame) = self.capture.read()
                self.frame= np.clip(self.frame, 0, 255)
            # time.sleep(.001)

    def show_frame(self):
        with mp_holistic.Holistic(min_detection_confidence=0.2, min_tracking_confidence=0.2) as holistic:
            image = self.frame
            image = cv2.resize(image, (720, 480), interpolation=cv2.INTER_AREA)
            image.flags.writeable = False
            image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
            results = holistic.process(image)
            custom_pose_landmark = []
            if results.pose_landmarks is not None:
                for i in range(12, 22):
                    custom_pose_landmark.append(results.pose_landmarks.landmark[i])

                landmark_pose_subset = landmark_pb2.NormalizedLandmarkList(landmark=custom_pose_landmark)


                # mp_drawing.draw_landmarks(image=image, landmark_list=landmark_pose_subset)
                Xa1 = int(image.shape[1] * landmark_pose_subset.landmark[0].x) # Vai
                Ya1 = int(image.shape[1] * landmark_pose_subset.landmark[0].y)
                Xb1 = int(image.shape[1] * landmark_pose_subset.landmark[2].x)  # Khuu Tay
                Yb1 = int(image.shape[1] * landmark_pose_subset.landmark[2].y)
                Xc1 = int(image.shape[1] * landmark_pose_subset.landmark[4].x)  # Co Tay
                Yc1 = int(image.shape[1] * landmark_pose_subset.landmark[4].y)


                cv2.line(image,
                    (int(image.shape[1] * landmark_pose_subset.landmark[0].x), int(image.shape[0] * landmark_pose_subset.landmark[0].y)),
                    (int(image.shape[1] * landmark_pose_subset.landmark[4].x), int(image.shape[0] * landmark_pose_subset.landmark[4].y)),
                    (0, 255, 0), thickness=3, lineType=8)
                

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
                        200, (255, 0, 0), 2)
        cv2.imshow('frame' + str(self.cam), image)
        if cv2.waitKey(5) & 0xFF == ord('q'):
            self.capture.release()
            cv2.destroyAllWindows()
            exit(1)


if __name__ == '__main__':
    video_stream_widget1 = VideoStreamWidget(0)
    while True:
        try:
            video_stream_widget1.show_frame()
        except AttributeError:
            pass