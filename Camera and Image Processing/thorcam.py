import cv2
import numpy as np
from instrumental.drivers.cameras import uc480

# init camera
instruments = uc480.list_instruments()
print(instruments)
cam = uc480.UC480_Camera(instruments[0])

# params
cam.start_live_video(framerate = "10Hz")

while cam.is_open:
     
     frame = cam.grab_image(timeout='100s', copy=True, exposure_time='10ms')
     
     frame1 = np.stack((frame,) * 3,-1) #make frame as 1 channel image
     frame1 = frame1.astype(np.uint8)

     gray = cv2.cvtColor(frame1, cv2.COLOR_BGR2GRAY)

     #now u can apply opencv features

     cv2.imshow('Camera', gray)
     
     if cv2.waitKey(30) & 0xFF == ord('q'):
        break

cam.close()
cv2.destroyAllWindows()