#!/usr/bin/env python3

import cv2


def display_video_by_idx(idx):
    cap = cv2.VideoCapture(idx)

    if cap.isOpened():
        print(f'Opened video {idx}')
        while True:
            ret, frame = cap.read()

            # Perform operations on the frame (e.g., display, process, etc.)
            cv2.imshow('Camera', frame)

            # Break the loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    else:
        print("Failed to open the camera.")

    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    display_video_by_idx(6)
