import cv2
import numpy as np
import datetime
import time
ts = time.time()
st = datetime.datetime.fromtimestamp(ts).strftime('%Y-%m-%d-%H:%M:%S')

# Create a VideoCapture object
cap = cv2.VideoCapture(0)

# Check if camera opened successfully
if (cap.isOpened() == False):
  print("Unable to read camera feed")

#cap.set(cv2.cv.CAP_PROP_FRAME_WIDTH,480);
#cap.set(CAP_PROP_FRAME_HEIGHT,480);


params = cv2.SimpleBlobDetector_Params()
params.filterByColor = False
params.filterByInertia = False
params.filterByConvexity = False


# Change thresholds
params.minThreshold = 0
params.maxThreshold = 200

# filterByCircularity = True
# minCircularity = 0.8



# Filter by Area.
params.filterByArea = True
params.minArea = 10
params.maxArea = 500
detector = cv2.SimpleBlobDetector(params)
writer = cv2.VideoWriter("outpy.avi",-1,30,(640,480))



while(True):
  ret, frame = cap.read()
  if not ret :
    break

  small_cap = cv2.resize(frame, (64,48))
  gray = cv2.cvtColor( small_cap, cv2.COLOR_BGR2GRAY )
  
  ret,thresh1 = cv2.threshold(gray,100,255,cv2.THRESH_BINARY)

  # Detect blobs.
  keypoints = detector.detect(thresh1)

  # Draw detected blobs as red circles.
  # cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
  im_with_keypoints = cv2.drawKeypoints(thresh1, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

  # Show keypoints
  work = frame
  for keypoint in keypoints:
    x = int(keypoint.pt[0] * 10)
    y = int(keypoint.pt[1] * 10)
    radius = int(keypoint.size * 10)/2
    cv2.circle(work, (x,y), radius, (255,0,0), thickness=1, lineType=8, shift=0)
  cv2.imshow("Layover", work)
  cv2.imshow("thresh", thresh1)

  #cv2.imshow('Objects Detected', thresh1)
  # Press Q on keyboard to stop recording
  if cv2.waitKey(1) & 0xFF == ord('q'):
    break

  writer.write(work)
# When everything done, release the video capture and video write objects
cap.release()
writer.release()
# Closes all the frames
cv2.destroyAllWindows()

