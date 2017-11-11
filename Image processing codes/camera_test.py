import cv2

'''
* Team Id: LM#438
* Authors: Srijal Poojari, Aditya Nair
* Filename: camera_test.py
* Theme: eYRC-Launch a Module, 2016
* Description: A basic program to get raw video feed from the overhead camera
*              for testing, positioning and adjusting the camera focus so that the
*              entire arena is captured properly.
'''

camera_port = 1

camera = cv2.VideoCapture(camera_port)

while True:
    ret, img = camera.read()
    cv2.imshow('raw', img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

camera.release()
cv2.destroyAllWindows()
