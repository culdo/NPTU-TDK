# Black Grayscale Line Following Example
#
# Making a line following robot requires a lot of effort. This example script
# shows how to do the machine vision part of the line following robot. You
# can use the output from this script to drive a differential drive robot to
# follow a line. This script just generates a single turn value that tells
# your robot to go left or right.
#
# For this script to work properly you should point the camera at a line at a
# 45 or so degree angle. Please make sure that only the line is within the
# camera's field of view.

import sensor, image, time, math
from pyb import UART,delay

#initial the uart
uart = UART(3, 115200)
uart.init(115200, bits=8, parity=None, stop=1) # init with given parameters
# Tracks a black line. Use [(128, 255)] for a tracking a white line.
GRAYSCALE_THRESHOLD = [(0, 50)]

# Each roi is (x, y, w, h). The line detection algorithm will try to find the
# centroid of the largest blob in each roi. The x position of the centroids
# will then be averaged with different weights where the most weight is assigned
# to the roi near the bottom of the image and less to the next roi and so on.
ROIS = [ # [ROI, weight]
        (40, 100, 80, 20, 0.5), # You'll need to tweak the weights for your app
        (40,  50, 80, 20, 0.5), # depending on how your robot is setup.
        (40,   0, 80, 20, 0.5)
       ]
ROIS1 = [ # [ROI, weight]
        (0, 100, 160, 20, 1), # You'll need to tweak the weights for your app
        (0,  50, 160, 20, 1), # depending on how your robot is setup.
        (0,   0, 160, 20, 1)
       ]

# Compute the weight divisor (we're computing this so you don't have to make weights add to 1).
weight_sum = 0
weight_sum1 = 0
weight_sum2 = 0
weight_sum3 = 0
weight_sum4 = 0
PreErr=0
kp=2
kd=1
for r in ROIS: weight_sum += r[4] # r[4] is the roi weight.
for q in ROIS1: weight_sum1 += q[4] # r[4] is the roi weight.

# Camera setup...
sensor.reset() # Initialize the camera sensor.
sensor.set_pixformat(sensor.GRAYSCALE) # use grayscale.
sensor.set_framesize(sensor.QQVGA) # use QQVGA for speed.
sensor.skip_frames(time = 2000) # Let new settings take affect.
sensor.set_auto_gain(False) # must be turned off for color tracking
sensor.set_auto_whitebal(False) # must be turned off for color tracking
clock = time.clock() # Tracks FPS.

while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    img = sensor.snapshot() # Take a picture and return the image.

    centroid_sum = 0
    centroid_sum1 = 0
    for r in ROIS:
        blobs = img.find_blobs(GRAYSCALE_THRESHOLD, roi=r[0:4], merge=True) # r[0:4] is roi tuple.

        if blobs:
            # Find the blob with the most pixels.
            largest_blob = max(blobs, key=lambda b: b.pixels())
            # Draw a rect around the blob.
            img.draw_rectangle(largest_blob.rect())
            img.draw_cross(largest_blob.cx(),
                           largest_blob.cy())

            centroid_sum += largest_blob.cx() * r[4] # r[4] is the roi weight.
            centroid_sum1 += largest_blob.cy() * q[4] # r[4] is the roi weight.

            #a=([largest_blob.cy(),largest_blob.cy(),largest_blob.cy()])
            #t4=tuple(a)
           # a=largest_blob.cy()*1
            #print(a[1])
            #if (largest_blob.cy()>=100):
            #weight1=0.7
            #else
            #weight1=0

            #if (largest_blob.cy()>=50) and (largest_blob.cy()<=70):
            #weight2=0.3
            #else
            #weight2=0

            #if (largest_blob.cy()>=0) and (largest_blob.cy()<=20):
            #weight3=0.1
            #else
            #weight3=0

            #weight_sum=weight1+weight2+weight3
           # print(weight_sum1)
    center_pos = (centroid_sum / weight_sum) # Determine center of line.
    center_pos1 = (centroid_sum1 / weight_sum1) # Determine center of line.
   # print(centroid_sum1)

    if (centroid_sum1>=50) and (centroid_sum1<=90):
        weight_sum2 = 1
        center_pos = (centroid_sum / weight_sum2)
        deflection_angle = 0
        deflection_angle = -math.atan((center_pos-80)/60)
        deflection_angle = math.degrees(deflection_angle)
      #  deflection_angle = int(deflection_angle)
       # print("Turn Angle: %f" % deflection_angle)
       # uart.write(deflection_angle)
    elif (centroid_sum1>=100) and (centroid_sum1<=120):
        weight_sum3 = 1
        center_pos = (centroid_sum / weight_sum3)
        deflection_angle = 0
        deflection_angle = -math.atan((center_pos-80)/60)
        deflection_angle = math.degrees(deflection_angle)
      #  deflection_angle = int(deflection_angle)
        #print("Turn Angle: %f" % deflection_angle)
       # uart.write(deflection_angle)
    else:
        weight_sum4 = 1.5
        center_pos = (centroid_sum / weight_sum4)
        deflection_angle = 0
        deflection_angle = -math.atan((center_pos-80)/60)
        deflection_angle = math.degrees(deflection_angle)
      #  deflection_angle = int(deflection_angle)
      #  print("Turn Angle: %f" % deflection_angle)
      #  uart.write(deflection_angle)
   # if (centroid_sum1>=100) and (centroid_sum1<=120):
   # weight_sum3 = 0.8
   # center_pos = (centroid_sum / weight_sum3)
   # deflection_angle = 0
   # deflection_angle = -math.atan((center_pos-80)/60)
   # deflection_angle = math.degrees(deflection_angle)

    #weight_sum2=[0.8]

    #else
    #weight_sum2=[1.1]

     # Determine center of line.
   # weight_sum2=0
    #print(centroid_sum1)
    # Convert the center_pos to a deflection angle. We're using a non-linear
    # operation so that the response gets stronger the farther off the line we
    # are. Non-linear operations are good to use on the output of algorithms
    # like this to cause a response "trigger".
    #deflection_angle = 0

    # The 80 is from half the X res, the 60 is from half the Y res. The
    # equation below is just computing the angle of a triangle where the
    # opposite side of the triangle is the deviation of the center position
    # from the center and the adjacent side is half the Y res. This limits
    # the angle output to around -45 to 45. (It's not quite -45 and 45).
    #deflection_angle = -math.atan((center_pos-80)/60)

    # Convert angle in radians to degrees.
    #deflection_angle = math.degrees(deflection_angle)

    # Now you have an angle telling you how much to turn the robot by which
    # incorporates the part of the line nearest to the robot and parts of
    # the line farther away from the robot for a better prediction.
    #print("Turn Angle: %f" % deflection_angle)
  #  print(clock.fps()) # Note: Your OpenMV Cam runs about half as fast while
    # connected to your computer. The FPS should increase once disconnected.
    if (deflection_angle>53) or (deflection_angle<-53):
        yaw=1484
        print("yaw speed: %d" % yaw)
        uart.write("%d\n" % yaw)
    else:
        Error=deflection_angle
        Derivative=Error-PreErr
        Control=int(kp*Error+kd*Derivative)
        yaw=1484-Control
        PreErr=Error
        uart.write("%d\n" % yaw)
        print("yaw speed: %d" % yaw)
   # uart.write(deflection_angle)
   # if deflection_angle >= 15:
   #     uart.write("L")
   # elif deflection_angle <= -15:
   #     uart.write("R")
   # else:
   #     uart.write("S")
    #time.sleep(.100)
    delay(10) # 發送延遲magic
