import cv2
camera = cv2.VideoCapture(0, cv2.CAP_DSHOW)
print(camera.isOpened())
ret, frame = camera.read()
print(ret, frame is not None if frame is not None else False)
camera.release()