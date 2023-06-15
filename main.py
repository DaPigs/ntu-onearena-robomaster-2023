import math
import time
# can change
max_marker_retry = 20
debug = True
max_moving_speed = 0.5 # 0.1 to 1
max_gripping_speed = 0.2 # 0.1 to 1
stop_distance = 0.12  # m
slow_distance = 0.6  # m
grip_distance = 3 # cm
claw_default = [180, -100, True]
claw_gripped = [180, 10, True]
claw_gripping = [180, -5, True]
boxes = [4, 5, 6, 7]

marker = {
    11: 1,
    12: 2,
    13: 3,
    14: 4,
    15: 5,
    16: 6,
    17: 7,
    47: "?",
    8: "heart",
}

# try not to change
num = None
# Define functions to move the robot
def move_forward(distance):
    chassis_ctrl.move_with_distance(0, distance)
    time.sleep(0.3)


def move_backward(distance):
    chassis_ctrl.move_with_distance(-180, distance)
    time.sleep(0.3)

def turn_left(angle):
    chassis_ctrl.rotate_with_degree(rm_define.anticlockwise, angle)


def turn_right(angle):
    chassis_ctrl.rotate_with_degree(rm_define.clockwise, angle)

# Define function to align the robot with the sign
class Marker:
    def __init__(self, sign, x, y, width, height):
        self.sign = marker[sign]
        self.x = x
        self.y = y
        self.width = width
        self.height = height

    def move(self):
        print("Moving to", self.sign)
        self.align()
        prev = float("infinity")
        while True:
            distance = ir_distance_sensor_ctrl.get_distance_info(1) / 100 - stop_distance
            if (distance < 0.05):
                return
            if(distance >= prev + 0.1):
                self.align()
                distance = ir_distance_sensor_ctrl.get_distance_info(1) / 100 - stop_distance
            prev = distance
            if(debug):
                print(f"Moving {distance}")
            if(distance >= slow_distance):
                speed = max_moving_speed
            else:
                speed = math.sin((distance/slow_distance) * math.pi / 2) * max_moving_speed
                if (speed <= 0.1):
                    speed = 0.1
            chassis_ctrl.set_trans_speed(speed)
            chassis_ctrl.move(0)

    def align(self):
        print("Aligning to", self.sign)
        while True:
            if (self.sign in boxes and self.x < 0.52 and self.x > 0.48):
                chassis_ctrl.stop()
                chassis_ctrl.set_trans_speed(1)
                return
            elif(self.x < 0.51 and self.x > 0.49):
                chassis_ctrl.stop()
                chassis_ctrl.set_trans_speed(1)
                return
            diff = abs(0.5 - self.x)
            if(debug):
                print(f"Diff {diff}")
            speed = math.sin(diff * math.pi) * max_moving_speed
            if (speed <= 0.1):
                speed = 0.1
            chassis_ctrl.set_trans_speed(speed)
            if (speed <= 0.1):
                time.sleep(0.05)
            if(debug):
                print(f"Aligning {speed}")
            if (self.x < 0.5):
                # move left
                chassis_ctrl.move(-90)
            else:
                # move right
                chassis_ctrl.move(90)
            markers = process_markers(vision_ctrl.get_marker_detection_info())
            for i in markers:
                if (i.sign == self.sign):
                    self = i
                    break

    def action(self):
        global num
        if (self.sign == 2 or (self.sign == "heart" and num in [5, 7])):
            turn_right(90)
        elif (self.sign == 3 or (self.sign == "heart" and num in [4, 6])):
            turn_left(90)
        elif (self.sign == 1):
            robotic_arm_ctrl.moveto(*claw_gripping)
            gripper_ctrl.open()
            time.sleep(0.5)
            move_backward(0.1)
            turn_left(90)
            robotic_arm_ctrl.moveto(*claw_default)
        elif (self.sign in boxes):
            aligned = False
            print("Gripping...")
            robotic_arm_ctrl.moveto(*claw_default)
            while True:
                distance = ir_distance_sensor_ctrl.get_distance_info(1)
                if(debug):
                    print(f"Gripping {distance}")
                if(not aligned and 5 <= distance <= 10):
                    self.align()
                    aligned = True
                if (distance < grip_distance):
                    break
                speed = math.sin((distance/20) * math.pi / 2) * max_gripping_speed
                if (speed <= 0.1):
                    speed = 0.1
                chassis_ctrl.set_trans_speed(speed)
                chassis_ctrl.move(0)
            robotic_arm_ctrl.moveto(*claw_gripping)
            gripper_ctrl.close()
            time.sleep(2)
            robotic_arm_ctrl.moveto(*claw_gripped)
            turn_left(180)
            num = self.sign
        elif (self.sign == "?"):
            if (num in [6, 7]):
                turn_left(90)
            elif (num in [4, 5]):
                turn_right(90)

    def __repr__(self):
        return str(self.sign)


def process_markers(li):
    markers = []
    n = li[0]
    del li[0]
    for i in range(0, n):
        i = i * 5
        markers.append(Marker(li[i], li[i + 1], li[i + 2], li[i + 3], li[i + 4]))
    markers.sort(key=lambda x:abs(0.5-x.x))
    return markers


def start():
    # definte stuff
    ir_distance_sensor_ctrl.enable_measure(1)
    chassis_ctrl.set_trans_speed(1)
    vision_ctrl.enable_detection(rm_define.vision_detection_marker)
    vision_ctrl.set_marker_detection_distance(1.5)
    chassis_ctrl.set_rotate_speed(45)
    robotic_arm_ctrl.moveto(*claw_default)
    gripper_ctrl.open()
    gripper_ctrl.update_power_level(4)
    time.sleep(2)
    retry = 0
    while True:
        markers = process_markers(vision_ctrl.get_marker_detection_info())
        if(retry >= max_marker_retry):
            chassis_ctrl.set_trans_speed(0.3)
            chassis_ctrl.move(0)
        if (markers):
            chassis_ctrl.stop()
            chassis_ctrl.set_trans_speed(1)
            print(f"Markers {markers}")
            markers[0].move()
            markers[0].action()
            retry = 0
        else:
            retry += 1
            print(f"No marker {retry}")