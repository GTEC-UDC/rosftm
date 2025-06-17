#!/usr/bin/env python3


""" MIT License

Copyright (c) 2020 Group of Electronic Technology and Communications. University of A Coruna.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE. """


import rclpy
from rclpy.node import Node
import time
import os
import csv
import serial
import json
from gtec_msgs.msg import ESP32S2FTMRangingExtra, ESP32S2FTMFrame

class FTMReaderExtra(Node):

    def __init__(self):
        super().__init__('esp32s2_ftm_tag_reader_extra')
        
        # Declare and get parameters
        self.declare_parameter('serial', '/dev/ttyUSB0')
        serial_port = self.get_parameter('serial').get_parameter_value().string_value
        
        # Create publisher
        self.publisher = self.create_publisher(ESP32S2FTMRangingExtra, '/gtec/ftm', 10)
        
        # Initialize serial connection
        self.get_logger().info("=========== FTM Tag reader ============")
        self.get_logger().info(f"serial_port: {serial_port}")
        self.get_logger().info("=========== [---------------] ============")
        
        try:
            self.ser = serial.Serial(serial_port, 115200)
            self.ser.flushInput()
        except serial.SerialException as e:
            self.get_logger().error(f"Failed to open serial port: {e}")
            return
        
        # Create timer for reading serial data
        self.timer = self.create_timer(0.1, self.loop)  # 10Hz
    
    def loop(self):
        try:
            ser_bytes = self.ser.readline()
            dataStr = ser_bytes[0:len(ser_bytes)-2].decode('UTF-8')
            
            # Detect format based on first character
            if dataStr.strip().startswith('{'):
                # JSON format - clean malformed JSON first
                cleaned_json = dataStr.replace(',]', ']').replace(',}', '}')
                data = json.loads(cleaned_json)
                anchorId = str(data['id'])
                rtt_est = int(data['rtt_est'])
                rtt_raw = int(data['rtt_raw'])
                dist = int(data['dist_est'])/100.0
                own_dist = int(data['own_est'])/100.0
                numFrames = int(data['num_frames'])
                
                ftmRanging = ESP32S2FTMRangingExtra()
                ftmRanging.anchor_id = anchorId
                ftmRanging.rtt_est = rtt_est
                ftmRanging.rtt_raw = rtt_raw
                ftmRanging.dist_est = dist
                ftmRanging.own_est = own_dist
                ftmRanging.num_frames = numFrames
                ftmRanging.frames = []
                
                for frame_data in data['frames']:
                    aFrame = ESP32S2FTMFrame()
                    aFrame.rtt = int(frame_data['rtt'])
                    aFrame.rssi = int(frame_data['rssi'])
                    aFrame.t1 = int(frame_data['t1'])
                    aFrame.t2 = int(frame_data['t2'])
                    aFrame.t3 = int(frame_data['t3'])
                    aFrame.t4 = int(frame_data['t4'])
                    ftmRanging.frames.append(aFrame)
                
                self.get_logger().info(f"JSON format - AnchorId: {anchorId} rtt_est: {rtt_est} rtt_raw: {rtt_raw} dist: {dist} own_dist: {own_dist} numFrames: {numFrames}")
            else:
                # CSV format
                data = dataStr.split(',')
                anchorId = str(data[0])
                rtt_est = int(data[1])
                rtt_raw = int(data[2])
                dist = int(data[3])/100.0
                own_dist = int(data[4])/100.0
                numFrames = int(data[5])

                ftmRanging = ESP32S2FTMRangingExtra()
                ftmRanging.anchorid = anchorId
                ftmRanging.rtt_est = rtt_est
                ftmRanging.rtt_raw = rtt_raw
                ftmRanging.dist_est = dist
                ftmRanging.own_est = own_dist
                ftmRanging.num_frames = numFrames
                ftmRanging.frames = []
                
                for index in range(numFrames):
                    frameRtt = int(data[6+index*6])
                    frameRssi = int(data[6+index*6+1])
                    frameT1 = int(data[6+index*6+2])
                    frameT2 = int(data[6+index*6+3])
                    frameT3 = int(data[6+index*6+4])
                    frameT4 = int(data[6+index*6+5])
                    aFrame = ESP32S2FTMFrame()
                    aFrame.rtt = frameRtt
                    aFrame.rssi = frameRssi
                    aFrame.t1 = frameT1
                    aFrame.t2 = frameT2
                    aFrame.t3 = frameT3
                    aFrame.t4 = frameT4
                    ftmRanging.frames.append(aFrame)
                
                self.get_logger().info(f"CSV format - AnchorId: {anchorId} rtt_est: {rtt_est} rtt_raw: {rtt_raw} dist: {dist} own_dist: {own_dist} numFrames: {numFrames}")

            self.publisher.publish(ftmRanging)

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error parsing JSON data: {e}")
            self.get_logger().error(f"Raw data: {dataStr}")
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"Error parsing CSV data: {e}")
            self.get_logger().error(f"Raw data: {dataStr}")
        except Exception as e:
            self.get_logger().error(f"Error reading serial port: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    ftm_reader_extra = FTMReaderExtra()
    
    try:
        rclpy.spin(ftm_reader_extra)
    except KeyboardInterrupt:
        pass
    finally:
        ftm_reader_extra.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()