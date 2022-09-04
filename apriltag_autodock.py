# The MIT License (MIT)

# Copyright (c) 2013-2021 Ibrahim Abdelkader <iabdalkader@openmv.io>
# Copyright (c) 2013-2021 Kwabena W. Agyeman <kwagyeman@openmv.io>

# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE
#
#
# =============================================================================================================
# This is a slightly modified version of mavlink_apriltags_landing_target example which comes with OpenMV IDE
# Successful tests for AutoDocking have been conducted using this script
# This script sends out AprilTag detections using the MAVLink protocol to
# an ArduPilot/PixHawk controller for precision landing using your OpenMV Cam.
#
# P4 = TXD
#
# Make sure you have put the marker id and size in valid_tag_ids
# GREEN led blinks on the camera when the camera is able to detect any markers listed in valid_tag_ids
# RED led blinks when the camera could not detect any markers
# =============================================================================================================

import image, math, pyb, sensor, struct, time
from pyb import LED

# Parameters #################################################################

uart_baudrate = 115200

MAV_system_id = 1
MAV_component_id = 0x54

lens_mm = 2.8 # Standard Lens.
lens_to_camera_mm = 22 # Standard Lens.
sensor_w_mm = 4.592 # For OV5650 sensor
sensor_h_mm = 3.423 # For OV5650 sensor

# Only tags with a tag ID in the dictionary below will be accepted by this
# code. You may add as many tag IDs to the below dictionary as you want...

# For each tag ID you need to provide then length of the black tag border
# in mm. Any side of the tag black border square will work.

valid_tag_ids = {
                  1 : 380,
                }

##############################################################################

# Camera Setup

sensor.reset()
sensor.set_pixformat(sensor.GRAYSCALE)
sensor.set_framesize(sensor.QVGA)
sensor.skip_frames(time = 2000)

x_res = 320 # QVGA
y_res = 240 # QVGA
f_x = (lens_mm / sensor_w_mm) * x_res
f_y = (lens_mm / sensor_h_mm) * y_res
c_x = x_res / 2
c_y = y_res / 2
h_fov = 2 * math.atan((sensor_w_mm / 2) / lens_mm)
v_fov = 2 * math.atan((sensor_h_mm / 2) / lens_mm)

def translation_to_cm(translation, tag_size): # translation is in decimeters...
    return (((translation * 10) * tag_size) / 210)

# Link Setup

uart = pyb.UART(3, uart_baudrate, timeout_char = 1000)

# Helper Stuff

packet_sequence = 0

def checksum(data, extra): # https://github.com/mavlink/c_library_v1/blob/master/checksum.h
    output = 0xFFFF
    for i in range(len(data)):
        tmp = data[i] ^ (output & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    tmp = extra ^ (output & 0xFF)
    tmp = (tmp ^ (tmp << 4)) & 0xFF
    output = ((output >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return output

MAV_LANDING_TARGET_message_id = 149
MAV_LANDING_TARGET_min_distance = 1/100 # in meters
MAV_LANDING_TARGET_max_distance = 10000/100 # in meters
MAV_LANDING_TARGET_frame = 8 # MAV_FRAME_BODY_NED
MAV_LANDING_TARGET_extra_crc = 200

# http://mavlink.org/messages/common#LANDING_TARGET
# https://github.com/mavlink/c_library_v1/blob/master/common/mavlink_msg_landing_target.h
def send_landing_target_packet(tag, w, h, tag_size):
    global packet_sequence
    dist_cm = math.sqrt(translation_to_cm(tag.x_translation(), tag_size) ** 2 + translation_to_cm(tag.y_translation(), tag_size) ** 2 + translation_to_cm(tag.z_translation(), tag_size) ** 2)
    temp = struct.pack("<qfffffbb",
                       0,
                       ((tag.cx() / w) - 0.5) * h_fov,
                       ((tag.cy() / h) - 0.5) * v_fov,
                       dist_cm * 0.01,
                       0.0,
                       0.0,
                       0,
                       MAV_LANDING_TARGET_frame)
    temp = struct.pack("<bbbbb30s",
                       30,
                       packet_sequence & 0xFF,
                       MAV_system_id,
                       MAV_component_id,
                       MAV_LANDING_TARGET_message_id,
                       temp)
    temp = struct.pack("<b35sh",
                       0xFE,
                       temp,
                       checksum(temp, MAV_LANDING_TARGET_extra_crc))
    packet_sequence += 1
    uart.write(temp)

# Main Loop

clock = time.clock()
i = 0
led_num = 1 # red led
target_in_sight = False

while(True):
    clock.tick()
    img = sensor.snapshot()
    tags = sorted(img.find_apriltags(fx=f_x, fy=f_y, cx=c_x, cy=c_y), key = lambda x: x.w() * x.h(), reverse = True)

    if tags and (tags[0].id() in valid_tag_ids):
        target_in_sight = True
        send_landing_target_packet(tags[0], img.width(), img.height(), valid_tag_ids[tags[0].id()])
        img.draw_rectangle(tags[0].rect())
        img.draw_cross(tags[0].cx(), tags[0].cy())
        tag_size = valid_tag_ids[tags[0].id()]
        dist = math.sqrt(translation_to_cm(tags[0].x_translation(), tag_size) ** 2 + translation_to_cm(tags[0].y_translation(), tag_size) ** 2 + translation_to_cm(tags[0].z_translation(), tag_size) ** 2)
        print("Distance %f cm" % dist)
        #print("Tx: %.2f, Ty: %.2f, Tz: %.2f\n" % (translation_to_cm(tags[0].x_translation(), tag_size), translation_to_cm(tags[0].y_translation(), tag_size), translation_to_cm(tags[0].x_translation(), tag_size)))
        #print("Ax: %.2f, Ay: %.2f\n" % (((tags[0].cx() / img.width()) - 0.5) * h_fov, ((tags[0].cy() / img.height()) - 0.5) * v_fov))

    else:
        target_in_sight = False

    print("FPS %f" % clock.fps())

    # led indication
    i += 1
    if ((i % 4) & 1):
        LED(1 if target_in_sight else 2).off()
        LED(2 if target_in_sight else 1).toggle()
