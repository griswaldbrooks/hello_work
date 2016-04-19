#!/usr/bin/env python

# Generic driver for the Neato XV-11 Robot Vacuum
# Copyright (c) 2010 University at Albany. All right reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the University at Albany nor the names of its
#       contributors may be used to endorse or promote products derived
#       from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL VANADIUM LABS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
# OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

"""
neato_driver.py is a generic driver for the Neato XV-11 Robotic Vacuum.
ROS Bindings can be found in the neato_node package.
"""

__author__ = "ferguson@cs.albany.edu (Michael Ferguson)"

import serial
import rospy
import time
import threading
import telnetlib

BASE_WIDTH = 248    # millimeters
MAX_SPEED = 300     # millimeters/second

xv11_analog_sensors = [ "WallSensorInMM",
                "BatteryVoltageInmV",
                "LeftDropInMM",
                "RightDropInMM",
                "RightMagSensor",
                "LeftMagSensor",
                "XTemp0InC",
                "XTemp1InC",
                "VacuumCurrentInmA",
                "ChargeVoltInmV",
                "NotConnected1",
                "BatteryTemp1InC",
                "NotConnected2",
                "CurrentInmA",
                "NotConnected3",
                "BatteryTemp0InC" ]

xv11_digital_sensors = [ "SNSR_DC_JACK_CONNECT",
                "SNSR_DUSTBIN_IS_IN",
                "SNSR_LEFT_WHEEL_EXTENDED",
                "SNSR_RIGHT_WHEEL_EXTENDED",
                "LSIDEBIT",
                "LFRONTBIT",
                "RSIDEBIT",
                "RFRONTBIT" ]

xv11_motor_info = [ "Brush_MaxPWM",
                "Brush_PWM",
                "Brush_mVolts",
                "Brush_Encoder",
                "Brush_RPM",
                "Vacuum_MaxPWM",
                "Vacuum_PWM",
                "Vacuum_CurrentInMA",
                "Vacuum_Encoder",
                "Vacuum_RPM",
                "LeftWheel_MaxPWM",
                "LeftWheel_PWM",
                "LeftWheel_mVolts",
                "LeftWheel_Encoder",
                "LeftWheel_PositionInMM",
                "LeftWheel_RPM",
                "RightWheel_MaxPWM",
                "RightWheel_PWM",
                "RightWheel_mVolts",
                "RightWheel_Encoder",
                "RightWheel_PositionInMM",
                "RightWheel_RPM",
                "Laser_MaxPWM",
                "Laser_PWM",
                "Laser_mVolts",
                "Laser_Encoder",
                "Laser_RPM",
                "Charger_MaxPWM",
                "Charger_PWM",
                "Charger_mAH" ]

xv11_charger_info = [ "FuelPercent",
                "BatteryOverTemp",
                "ChargingActive",
                "ChargingEnabled",
                "ConfidentOnFuel",
                "OnReservedFuel",
                "EmptyFuel",
                "BatteryFailure",
                "ExtPwrPresent",
                "ThermistorPresent[0]",
                "ThermistorPresent[1]",
                "BattTempCAvg[0]",
                "BattTempCAvg[1]",
                "VBattV",
                "VExtV",
                "Charger_mAH",
                "MaxPWM" ]


class Botvac():
    def __init__(self, telnet_ip="192.168.1.108", telnet_port="8711"):
        self.telnet_c = telnetlib.Telnet(telnet_ip, telnet_port)
        print(">>> telnet to server at %s:%s now..."%(telnet_ip, telnet_port))
        rospy.loginfo("Huey: telneted to server %s:%s now..." %(telnet_ip, telnet_port))


        # Storage for motor and sensor information
        self.state = {"LeftWheel_PositionInMM": 0, "RightWheel_PositionInMM": 0,"LSIDEBIT":0,"RSIDEBIT":0,"LFRONTBIT":0,"RFRONTBIT":0}
        self.stop_state = True
        # turn things on
        self.comsData = []
        self.responseData= []
        self.currentResponse=[]

        #self.port.flushInput()
        self.sendCmd("\n\n\n")
        #self.port.flushInput()

        #let's printout "getver" from Robot
        self.sendCmd("getversion")
        ver=self.telnet_c.read_until("@#", 5)
        print ("Robot version is %s" % ver)

        self.setTestMode("on")
        ver=self.telnet_c.read_until("@#", 5)
        self.setLDS("on")
        ver=self.telnet_c.read_until("@#", 5)

        time.sleep(0.5)
        self.setLed("Led1Blink")
        ver=self.telnet_c.read_until("@#", 5)

        self.base_width = BASE_WIDTH
        self.max_speed = MAX_SPEED

        # start a reading thread???
        '''
        self.reading = False
        self.readLock = threading.RLock()
        self.readThread = threading.Thread(None,self.read)
        self.readThread.start()
        '''
        #self.flush()
        rospy.loginfo("Init Done")


    def exit(self):
        self.setLDS("off")
        self.setLed("buttonoff")

        time.sleep(1)

        self.setTestMode("off")
        self.flush()

        '''
        self.reading=False
        self.readThread.join()

        self.telnet_c.close()
        '''

    def setTestMode(self, value):
        """ Turn test mode on/off. """
        self.sendCmd("testmode " + value)

    def setLDS(self, value):
        self.sendCmd("setldsrotation " + value )

    def requestScan(self):
        """ Ask neato for an array of scan reads. """
        self.sendCmd("getldsscan")

    #def getScanRanges(self):
    def getLdsScan(self):

        """ Read values of a scan -- call requestScan first! """
        ranges = list()
        angle = 0

        mVals = self.telnet_c.read_until("@#")

        '''
        if not self.readTo("AngleInDegrees"):
            self.flush()
            return []
        '''

        vals = mVals.splitlines()
        #print vals
        #print "before LDS while loop here"
        for l in vals: #angle < 360:
            if len(l) < 2:
                continue

            if "," not in l:
                continue

            vals = l.split(",")

            if (ord(vals[0][0])>=48 and ord(vals[0][0])<=57 ):
                #print angle, vals
                try:
                    a = int(vals[0])
                    r = int(vals[1])
                    e = int(vals[3])

                    while (angle < a):
                        ranges.append(0)
                        angle +=1

                    if(e==0):
                        ranges.append(r/1000.0)
                    else:
                        ranges.append(0)
                except:
                    ranges.append(0)
                angle += 1

        if len(ranges) <> 360:
            rospy.loginfo( "Missing laser scans: got %d points" %len(ranges))

        return ranges

    def setMotors(self, l, r, s):
        """ Set motors, distance left & right + speed """
        #This is a work-around for a bug in the Neato API. The bug is that the
        #robot won't stop instantly if a 0-velocity command is sent - the robot
        #could continue moving for up to a second. To work around this bug, the
        #first time a 0-velocity is sent in, a velocity of 1,1,1 is sent. Then,
        #the zero is sent. This effectively causes the robot to stop instantly.
        if (int(l) == 0 and int(r) == 0 and int(s) == 0):
            if (not self.stop_state):
                self.stop_state = True
                l = 1
                r = 1
                s = 1
        else:
            self.stop_state = False

        print("sendcmd: setmotor %s,%s,%s"%(str(int(l)), str(int(r)), str(int(s))))
        self.sendCmd("setmotor "+str(int(l))+" "+str(int(r))+" "+str(int(s)))

    def getMotors(self):
        """ Update values for motors in the self.state dictionary.
            Returns current left, right encoder values. """

        #print("calling flush()")
        self.flush()
        self.sendCmd("getmotors")
        mVars=self.telnet_c.read_until("@#")
        #print("getmotors: %s" % mVars)

        vals=mVars.splitlines()
        for i in vals:
            if len(i) > 2:
                values = i.split(",")
                #print values
                if values[0] in xv11_motor_info:
                    self.state[values[0]] = int(values[1])

        #print("getmotors return LeftWheel_PositionInMM:%d and RightWheel_PositionInMM:%d" % (self.state["LeftWheel_PositionInMM"],self.state["RightWheel_PositionInMM"]))
        return [self.state["LeftWheel_PositionInMM"],self.state["RightWheel_PositionInMM"]]


        '''
        if not self.readTo("Parameter"):
            #self.flush()
            return [0,0]
        last=False
        while not last:
        #for i in range(len(xv11_motor_info)):
            try:
                vals,last = self.getResponse()
                print vals,last
                values = vals.split(",")
                self.state[values[0]] = int(values[1])
            except Exception as ex:
                rospy.logerr("Exception Reading Neato motors: " + str(ex))

        return [self.state["LeftWheel_PositionInMM"],self.state["RightWheel_PositionInMM"]]
        '''

    def getAnalogSensors(self):
        """ Update values for analog sensors in the self.state dictionary. """

        self.sendCmd("getanalogsensors")
        mVars=self.telnet_c.read_until("@#")
        #print mVars

        vals=mVars.splitlines()
        #print vals
        for i in vals:
            if len(i) < 2:
                continue
            if "," not in i:
                continue

            values = i.split(",")
            #print values

            if values[0] in xv11_analog_sensors:
                self.state[values[0]] = int(values[1])

        #print("getAnalogSensors done")

        '''
        if not self.readTo("SensorName"):
            #self.flush()
            return

        last =False
        while not last: #for i in range(len(xv11_analog_sensors)):
            try:
                vals,last = self.getResponse()
                print vals,last
                values = vals.split(",")
                self.state[values[0]] = int(values[1])
            except Exception as ex:
                rospy.logerr("Exception Reading Neato Analog sensors: " + str(ex))
        '''

    def getDigitalSensors(self):
        """ Update values for digital sensors in the self.state dictionary. """
        self.sendCmd("getdigitalsensors")
        mVars=self.telnet_c.read_until("@#")
        #print mVars

        vals=mVars.splitlines()

        #print vals

        for i in vals:
            if len(i) > 2:
                if "," not in i:
                    continue
                values = i.split(",")
                #print values
                if values[0] in xv11_digital_sensors:
                    self.state[values[0]] = int(values[1])

        #print("getDigitalSensors done. LSIDEBIT:%d, RSIDEBIT:%d, LFRONTBIT:%d, RFRONTBIT:%d" %
        #      (self.state["LSIDEBIT"], self.state["RSIDEBIT"], self.state["LFRONTBIT"], self.state["RFRONTBIT"]))
        return [self.state["LSIDEBIT"], self.state["RSIDEBIT"], self.state["LFRONTBIT"], self.state["RFRONTBIT"]]

        '''
        if not self.readTo("Digital Sensor Name"):
             self.flush()
             return [0, 0, 0, 0]

        last =False
        while not last: #for i in range(len(xv11_digital_sensors)):
            try:
                vals,last = self.getResponse()
                print vals
                values = vals.split(",")
                self.state[values[0]] = int(values[1])
                #print "Got Sensor: %s=%s" %(values[0],values[1])
            except Exception as ex:
                rospy.logerr("Exception Reading Neato Digital sensors: " + str(ex))
        return [self.state["LSIDEBIT"], self.state["RSIDEBIT"], self.state["LFRONTBIT"], self.state["RFRONTBIT"]]
        '''

    def getButtons(self):
        return [0,0,0,0,0]

    def getCharger(self):
        """ Update values for charger/battery related info in self.state dictionary. """
        self.sendCmd("getcharger")
        '''
        if not self.readTo("Label"):
            self.flush()
            return

        last =False
        while not last: #for i in range(len(xv11_charger_info)):

            vals,last = self.getResponse()
            values=vals.split(",")
            try:
                self.state[values[0]] = int(values[1])

            except Exception as ex:
                rospy.logerr("Exception Reading Neato charger info: " + str(ex))
        '''

    #def setBacklight(self, value):
    #    if value > 0:
    #       self.sendCmd("setled backlighton")
    #    else:
    #       self.sendCmd("setled backlightoff")

    def setLed(self,cmd):
        self.sendCmd("setled %s" % cmd)
    
    def setLED(self,cmd):
        self.setLed(cmd)
	
    def sendCmd(self,cmd):
        #rospy.loginfo("Sent command: %s"%cmd)
        self.telnet_c.write("%s\n" % cmd)

    def readTo(self,tag,timeout=3):
        try:
            line,last = self.getResponse(timeout)
        except:
            return False

        if line=="":
            return False

        print("readTo tag %s and line is %s" % (tag, line))
        while line.split(",")[0] != tag:
            try:
                line,last = self.getResponse(timeout)
                print("readTo while loop: tag %s and line is %s" % (tag, line))

                if line=="":
                    return False
            except:
                return False

        print("readTo return true!  tag %s and line is %s" % (tag, line))
        return True


    # thread to read data from the telnet port
    # buffers each line in a list (self.comsData)
    # when an end of response ("@#") is read, adds the complete list of response 
    # lines to self.responseData and resets the comsData list for the next command response.
    def read(self):
        line=""

        while(self.reading):
            #self.reading = False
            #time.sleep(1)
            try:
               #print("get one line:")

               #val = self.telnet_c.expect(p) # read from telnet 1 char at a time so we can parse each character
               #val = self.telnet_c.rawq_getchar(self)
               line = self.telnet_c.read_until("\n")
            except Exception as ex:
                line=[]

            if len(line) > 0:

                if '\x1A' in line:
                    print "line %s has ^Z" % (line)
                    self.comsData.append(line) 
                    line = "" 

                    with self.readLock:
                        self.responseData.append(list(self.comsData))
                    self.comsData = []     # clear
                elif "$" in line:

                    print "line %s has $ sign" % (line)
                    self.comsData.append(line) 
                    line = "" 

                    with self.readLock:
                        self.responseData.append(list(self.comsData))
                    self.comsData = []     # clear
                elif "@#" in line:

                    print "line %s has <@#> sign" % (line)
                    self.comsData.append(line) 
                    line = "" 

                    with self.readLock:
                        self.responseData.append(list(self.comsData))
                    self.comsData = []     # clear
                elif '\n' in line:
                    print("got NL here and line %s, and len(line) is %d" % (line, len(line)))
                    self.comsData.append(line)
                    line = ""
                else:
                    print("no NL and ^Z, line %s" % (line))


    # thread to read data from the serial port
    # buffers each line in a list (self.comsData)
    # when an end of response (^Z) is read, adds the complete list of 
    # response lines to self.responseData and resets the comsData list for the next command response.
    '''
    def read(self):
        self.reading = True
        line=""

        while(self.reading and not rospy.is_shutdown()):
            try:
               val = self.port.read(1) # read from serial 1 char at a time so we can parse each character
            except Exception as ex:
                rospy.logerr("Exception Reading Neato Serial: " + str(ex))
                val=[]

            if len(val) > 0:

                if ord(val[0]) ==13: # ignore the CRs
                    pass

                elif ord(val[0]) == 26: # ^Z (end of response)
                    if len(line) > 0:
                        self.comsData.append(line) # add last line to response set if it is not empty
                        #print("Got Last Line: %s" % line)
                        line="" # clear the line buffer for the next line

                    #print ("Got Last")
                    with self.readLock: # got the end of the command response so add the full set of response data as a new item in self.responseData
                        self.responseData.append(list(self.comsData))

                    self.comsData = [] # clear the bucket for the lines of the next command response

                elif ord(val[0]) == 10: # NL, terminate the current line and add it to the response data list (comsData) (if it is not a blank line)
                    if len(line) > 0:
                        self.comsData.append(line)
                        #print("Got Line: %s" % line)
                        line = "" # clear the bufer for the next line
                else:
                    line=line+val # add the character to the current line buffer
    '''

    # read response data for a command
    # returns tuple (line,last)
    # line is one complete line of text from the command response
    # last = true if the line was the last line of the response data (indicated by a ^Z from the neato)
    # returns the next line of data from the buffer.
    # if the line was the last line last = true
    # if no data is avaialable and we timeout returns line=""
    def getResponse(self,timeout=1):

        # if we don't have any data in currentResponse, wait for more data to come in (or timeout) 
        while (len(self.currentResponse)==0) and (not rospy.is_shutdown()) and timeout > 0:

            with self.readLock: # pop a new response data list out of self.responseData (should contain all data lines returned for the last sent command)
               if len(self.responseData) > 0:
                  self.currentResponse = self.responseData.pop(0)
                  print "New Response Set"
               else:
                  self.currentResponse=[] # no data to get

            if len(self.currentResponse)==0: # nothing in the buffer so wait (or until timeout)
               time.sleep(0.010)
               timeout=timeout-0.010

        # default to nothing to return
        line = ""
        last=False

        # if currentResponse has data pop the next line 
        if not len(self.currentResponse)==0:
            line = self.currentResponse.pop(0)
            print line,len(self.currentResponse)
            if  len(self.currentResponse)==0:
                last=True  # if this was the last line in the response set the last flag
        else:
            print "Rx Time Out" # no data so must have timedout

        #rospy.loginfo("Got Response: %s, Last: %d" %(line,last))
        return (line,last)

    def flush(self):
        #print ("flush...")
        print self.telnet_c.read_eager()
        print self.telnet_c.read_lazy()
        '''
	    while(1):
          l,last= self.getResponse(1)
	    if l=="":
		    return	
        '''

#ButtonAmber - Start Button Amber (mutually exclusive of other Button options)
#ButtonGreen - Start Button Green (mutually exclusive of other Button options)
#LEDRed - Start Red LED (mutually exclusive of other Button options)
#LEDGreen - Start Green LED (mutually exclusive of other Button options)
#ButtonAmberDim - Start Button Amber Dim (mutually exclusive of other Button options)
#ButtonGreenDim - Start Button Green Dim (mutually exclusive of other Button options)
#ButtonOff - Start Button Off

