BINARY_VIEW = True
GRAYSCALE_THRESHOLD = (240, 255)
MAG_THRESHOLD = 4
THETA_GAIN = 40.0
RHO_GAIN = -1.0
P_GAIN = 0.7
I_GAIN = 0.0
I_MIN = -0.0
I_MAX = 0.0
D_GAIN = 0.1
STEERING_SERVO_INDEX = 0

import sensor, image, time, math, pyb
from machine import I2C, Pin
from servo import Servos

i2c = I2C(sda=Pin('P5'), scl=Pin('P4'), freq=10000)
servos = Servos(i2c, address=0x40, freq=50, min_us=1000, max_us=2200, degrees=180)
servos.position(STEERING_SERVO_INDEX, 90)

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QQQVGA)
sensor.set_vflip(True)
sensor.set_hmirror(True)
sensor.skip_frames(time = 2000)
clock = time.clock()

def line_to_theta_and_rho(line):
    if line.rho() < 0:
        if line.theta() < 90:
            return (math.sin(math.radians(line.theta())),
                math.cos(math.radians(line.theta() + 180)) * -line.rho())
        else:
            return (math.sin(math.radians(line.theta() - 180)),
                math.cos(math.radians(line.theta() + 180)) * -line.rho())
    else:
        if line.theta() < 90:
            if line.theta() < 45:
                return (math.sin(math.radians(180 - line.theta())),
                    math.cos(math.radians(line.theta())) * line.rho())
            else:
                return (math.sin(math.radians(line.theta() - 180)),
                    math.cos(math.radians(line.theta())) * line.rho())
        else:
            return (math.sin(math.radians(180 - line.theta())),
                math.cos(math.radians(line.theta())) * line.rho())

def line_to_theta_and_rho_error(line, img):
    t, r = line_to_theta_and_rho(line)
    return (t, r - (img.width() // 2))

old_result = 0
old_time = pyb.millis()
i_output = 0
output = 90

while True:
    clock.tick()
    img = sensor.snapshot().histeq()
    if BINARY_VIEW:
        img.binary([GRAYSCALE_THRESHOLD])
        img.erode(1)

    line = img.get_regression([(255, 255)], robust=True)
    print_string = ""

    if line and (line.magnitude() >= MAG_THRESHOLD):
        img.draw_line(line.line(), color=127)

        t, r = line_to_theta_and_rho_error(line, img)
        new_result = (t * THETA_GAIN) + (r * RHO_GAIN)
        delta_result = new_result - old_result
        old_result = new_result

        new_time = pyb.millis()
        delta_time = new_time - old_time
        old_time = new_time

        p_output = new_result
        i_output = max(min(i_output + new_result, I_MAX), I_MIN)
        d_output = (delta_result * 1000) / delta_time
        pid_output = (P_GAIN * p_output) + (I_GAIN * i_output) + (D_GAIN * d_output)

        output = 90 + max(min(int(pid_output), 90), -90)
        print_string = "Line Ok - turn %d - line t: %d, r: %d" % (output, line.theta(), line.rho())
    else:
        print_string = "Line Lost - turn %d" % output

    servos.position(STEERING_SERVO_INDEX, output)
    print("FPS %f, %s" % (clock.fps(), print_string))
