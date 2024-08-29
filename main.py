import cv2
from obstacle_detection import ObstacleDetection
from lane_detection import detect_lane
from traffic_signal_detection import detect_traffic_signal

def main():
#this detect obstacle 
    obstacle_detector = ObstacleDetection()
    cap = cv2.VideoCapture(0)

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        avg_distance = obstacle_detector.check_obstacle()
        lane_direction = detect_lane(frame)
        traffic_signal = detect_traffic_signal(frame)

        if traffic_signal == 'red':
            obstacle_detector.stop()
        elif traffic_signal == 'green':
            if avg_distance < 25:
                obstacle_detector.stop()
                time.sleep(1)
                obstacle_detector.backward()
                time.sleep(1.5)
                if lane_direction == 'left':
                    obstacle_detector.left()
                elif lane_direction == 'right':
                    obstacle_detector.right()
                time.sleep(1.5)
                obstacle_detector.stop()
            else:
                if lane_direction == 'left':
                    obstacle_detector.left()
                elif lane_direction == 'right':
                    obstacle_detector.right()
                else:
                    obstacle_detector.forward()

        cv2.imshow("Color Tracking", frame)
        if cv2.waitKey(100) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
    GPIO.cleanup()

if __name__ == "__main__":
    main()
