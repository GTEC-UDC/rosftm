#!/usr/bin/env python2


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


import rospy, time, serial, os, tf2_ros, csv
from gtec_msgs.msg import ESP32S2FTMRanging, ESP32S2FTMFrame

class FTMReader(object):

    def __init__(self, publisher):
        self.publisher = publisher
    
    def loop(self):
        try:
            ser_bytes = ser.readline()
            #decoded_bytes = float(ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
            #print(ser_bytes[0:len(ser_bytes)-2])
            dataStr = ser_bytes[0:len(ser_bytes)-2].decode('UTF-8')
            #print(dataStr)
            data = dataStr.split(',')
            anchorId = str(data[0])
            rtt_est = int(data[1])
            rtt_raw = int(data[2])
            dist = int(data[3])/100.0
            numFrames = int(data[4])

            ftmRanging = ESP32S2FTMRanging()
            ftmRanging.anchorId = anchorId
            ftmRanging.rtt_est = rtt_est
            ftmRanging.rtt_raw = rtt_raw
            ftmRanging.dist_est = dist
            ftmRanging.num_frames = numFrames
            ftmRanging.frames = []

            print("AnchorId: " + anchorId + " rtt_est: " + str(rtt_est) + " rtt_raw: " + str(rtt_raw) + " dist: " + str(dist) + " numFrames: " + str(numFrames))
            frames = []
            for index in range(numFrames):
                frameRtt = int(data[5+index*6])
                frameRssi = int(data[5+index*6+1])
                frameT1 = int(data[5+index*6+2])
                frameT2 = int(data[5+index*6+3])
                frameT3 = int(data[5+index*6+4])
                frameT4 = int(data[5+index*6+5])
                aFrame = ESP32S2FTMFrame()
                aFrame.rtt = frameRtt
                aFrame.rssi = frameRssi
                aFrame.t1 = frameT1
                aFrame.t2 = frameT2
                aFrame.t3 = frameT3
                aFrame.t4 = frameT4
                ftmRanging.frames.append(aFrame)

            #for frame in ftmRanging.frames:
            #    print("Frame: ["+ "rtt_est: "+ str(frame[0]) + " rssi: "+ str(frame[1]) + " t1: "+ str(frame[2]) + " t2: "+ str(frame[3]) + " t3: "+ str(frame[4]) + " t4: "+ str(frame[5]) + "]")
            self.publisher.publish(ftmRanging)

        except Exception as e:
            print("Error reading serial port." + str(e))
            #quit()

if __name__ == "__main__":

    rospy.init_node('ESP32S2FTMTagReader', anonymous=True)

    # Read parameters
    serial_port = rospy.get_param('~serial')
    

    pub_ranging = rospy.Publisher("/gtec/ftm/", ESP32S2FTMRanging, queue_size=100)
    rate = rospy.Rate(10) # 10hz


    print("=========== FTM Tag reader ============")
    print("serial_port: " + serial_port)
    print("=========== [---------------] ============")

    ser = serial.Serial(serial_port, 115200)
    ser.flushInput()

    ftmReader = FTMReader(pub_ranging)
    # rospy.spin()

    while not rospy.is_shutdown():
        ftmReader.loop()
        rate.sleep()