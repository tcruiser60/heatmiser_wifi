#!/usr/bin/env python
# coding=utf-8
#
# Version 1.4.4 - Should now support Wifi range of devices PRT PRT-e PRTHW TM1 
###############################################################################
#   - heatmiser_wifi -
#
#   Copyright 2020 by Joel Midstjärna (joel.midstjarna@gmail.com)
#
#   A Heatmiser WiFi Thermostat communication library. 
#
#   Supported Heatmiser Thermostats are DT, DT-E, PRT, PRT-E, PRTHW and TM1.
#
#   It is also possible to run this file as a command line executable.
#
#   All rights reserved.
#   This file is part of the heatmiser_wifi python library and is
#   released under the "MIT License Agreement". Please see the LICENSE
#   file that should have been included as part of this package.
###############################################################################
#                       Updates - 
# Updated from original to add extra functions and devices
# Changes by Tim Moore
# 
# V1.1   - Updated to allow for reading date and time values from PRTHW and TM1
# V1.2   - intergrate clock into info
# V1.3   - add clock write
# V1.3.1 - correct error in day of the week
# V1.4   - add in changes from Iain Bullock to read timer properties for HW on PRTHW
# V1.4.1 - display day of the week as Mon-Sun, remove print statements for Clock
# V1.4.3 - Adjust DCB read to include TM1
# V1.4.4 - Clean up removed objects
# V1.4.5 - Alter options in 'main' to allow -t no options
#          Add in set value to use 'clock' as per other set values
# 
# Tested on Wifi Touch Thermostats and timers 
# Updates have not been checked against DT & DTe devices
#
# Uses Heatmiser V3 Protocol
###############################################################################

import socket, time, sys, datetime
from optparse import OptionParser
from collections import OrderedDict
DoW = ["MON", "TUE", "WED", "THUR", "FRI", "SAT", "SUN"]

class CRC16:
    CRC16_LookupHigh = [0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70,
                        0x81, 0x91, 0xA1, 0xB1, 0xC1, 0xD1, 0xE1, 0xF1]
    CRC16_LookupLow  = [0x00, 0x21, 0x42, 0x63, 0x84, 0xA5, 0xC6, 0xE7,
                        0x08, 0x29, 0x4A, 0x6B, 0x8C, 0xAD, 0xCE, 0xEF]
                        
    CRC16_High = 0xff
    CRC16_Low  = 0xff
    
    def _CRC16_Update4Bits(self, val):
        t = (self.CRC16_High >> 4) & 0xff
        t = (t ^ val) & 0xff
        self.CRC16_High = ((self.CRC16_High << 4)|(self.CRC16_Low >> 4)) & 0xff
        self.CRC16_Low  = (self.CRC16_Low << 4) & 0xff
        self.CRC16_High = (self.CRC16_High ^ self.CRC16_LookupHigh[t]) & 0xff
        self.CRC16_Low  = (self.CRC16_Low  ^ self.CRC16_LookupLow[t]) & 0xff
        
    def _CRC16_Update(self, val):
        self._CRC16_Update4Bits((val >> 4) & 0x0f)
        self._CRC16_Update4Bits(val & 0x0f)
        
    def CRC16(self,bytes):
        self.CRC16_High = 0xff
        self.CRC16_Low  = 0xff
        for byte in bytes:
            self._CRC16_Update(byte)
        return (self.CRC16_Low, self.CRC16_High)
    

class HeatmiserTransport:
    ''' This class handles the Heatmiser transport protocol '''
    def __init__(self, host, port, pin):
        self.host = host
        self.port = port
        self.crc16 = CRC16()
        self.pin = pin
        
    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host,self.port))
        self.sock.settimeout(5)
        
    def disconnect(self):
        self.sock.close()
        
    def _send_read_request(self, dcb_start = 0x0, dcb_length = 0xffff):
        ''' dcb_length = 0xffff means the whole DCB. The Heatmiser v3 protocol
            specification recommends that the whole DCB shall be read at once.
            It seems that dcb_start > 0x0 has no effect.
        '''
        frame_list = [
            0x93,              # Operation 0x93=Read, 0xa3=Write
            0x0b,              # Frame length low byte inc CRC (0xb if no data)
            0x00,              # Frame length high byte inc CRC (0 if no data)
            int(self.pin) & 0xff,   # PIN code low byte
            int(self.pin) >> 8,     # PIN code high byte
            dcb_start & 0xff,  # DCB start low byte 
            dcb_start >> 8,    # DCB start high byte 
            dcb_length & 0xff, # DCB length low byte  (0xff for whole DCB)
            dcb_length >> 8]   # DCB length high byte (0xff for whole DCB) 
        frame = bytearray(frame_list)
        
        # Add CRC16 to the frame (16 bytes)
        (crc16_low, crc16_high) = self.crc16.CRC16(frame)
        frame.append(crc16_low)
        frame.append(crc16_high)       
        
        self.sock.send(frame)
        
    def _send_write_request(self, dcb_address, dcb_data):
        ''' dcb_address is address in DCB block (not index) '''
        dcb_length = len(dcb_data)
        length = dcb_length + 11
        frame_list = [
            0xa3,              # Operation 0x93=Read, 0xa3=Write
            length & 0xff,     # Frame length low byte inc CRC (11+dcb_data)
            length >> 8,       # Frame length high byte inc CRC (11+dcb_data)
            self.pin & 0xff,   # PIN code low byte
            self.pin >> 8,     # PIN code high byte
            1,                 # Nbr of items (only 1 in this impl)
            dcb_address & 0xff,# DCB address low byte 
            dcb_address >> 8,  # DCB address high byte 
            dcb_length]        # DCB length
        frame = bytearray(frame_list) + dcb_data
        
        # Add CRC16 to the frame (16 bytes)
        (crc16_low, crc16_high) = self.crc16.CRC16(frame)
        frame.append(crc16_low)
        frame.append(crc16_high)       
        
        self.sock.send(frame)        
        
    def _receive_dcb(self):
        data = self.sock.recv(1024)
        frame = bytearray(data)
        
        # Validate the CRC (two last bytes) and remove from frame
        received_crc16_high = frame.pop()
        received_crc16_low = frame.pop()
        (crc16_low, crc16_high) = self.crc16.CRC16(frame)
        if((received_crc16_high != crc16_high) or (received_crc16_low != crc16_low)):
            raise Exception("CRC16 mismatch in received data from Thermostat")
            
        # Validate frame head
        if(frame[0] != 0x94):
             raise Exception("Unknown type of message received from Thermostat")
             
        # Read out and validate the frame length
        frame_length = (frame[2] << 8) | frame[1]
        if(frame_length != (len(frame) + 2)): #+2 since CRC has been removed
            raise Exception("Invalid frame length in data received from Thermostat")
        
        # Read out DCB start address
        dcb_start = (frame[4] << 8) | frame[3]
        
        # Read out and validate DCB content length
        dcb_length = (frame[6] << 8) | frame[5]
        if(dcb_length == 0):
            raise Exception("Thermostat connected but reports wrong PIN code")
        if(dcb_length != (frame_length - 9)): #Overhead in frame is 9 bytes
            raise Exception("Invalid DCB length in data received from Thermostat")            
            
        # Read out DCB data
        dcb_data = frame[7:]
             
        return (dcb_start, dcb_data)
    
    def get_dcb(self):
        ''' Get whole DCB '''
        self._send_read_request()
        (dcb_start, dcb_data) = self._receive_dcb()
        return dcb_data
        
    def set_dcb(self, dcb_address, dcb_data):
        self._send_write_request(dcb_address, dcb_data)
        # Just get an ACK from the Thermostat (don't use the result)
        dcb = self._receive_dcb()
        


class Heatmiser(HeatmiserTransport):
    ''' This class handles the Heatmiser application (DCB) protocol '''
    
    def _get_info_time_triggers(self, dcb, first_index):
        index = first_index
        info = OrderedDict()
        for i in range (1,5):
            trigger = OrderedDict()
            trigger['hour'] = dcb[index]
            index = index + 1
            trigger['minute'] = dcb[index]
            index = index + 1     
            trigger['set_temp'] = dcb[index]
            index = index + 1
            info['time'+str(i)] = trigger
         
        return info 
        
# -------------- IB code to get Timer info from TM1 & PRTHW -------------
        
    def _get_info_time_triggers_hw(self, dcb, first_index):
        index = first_index
        info = OrderedDict()
        # Why is the loop 1 to 5 not 1 to 4?
        for i in range (1,5):
            trigger = OrderedDict()
            trigger['hour_on'] = dcb[index]
            index = index + 1
            trigger['minute_on'] = dcb[index]
            index = index + 1     
            trigger['hour_off'] = dcb[index]
            index = index + 1
            trigger['minute_off'] = dcb[index]
            index = index + 1            
            info['time'+str(i)] = trigger
        return info 
        
# ------------------------------------------------------------------------
    
    def get_info(self):
        ''' Returns an ordered dictionary with all Thermostat values '''
        dcb = self.get_dcb()
        
        if(len(dcb) < 41):
            raise Exception("Size of DCB received from Thermostat is too small")
        
        info = OrderedDict()
        
        if(dcb[2] == 0):
            info["vendor_id"] = "HEATMISER"
        else:
            info["vendor_id"] = "OEM"           
        info["version"] = dcb[3] & 0x7F  
        info["in_floor_limit_state"] = ((dcb[3] & 0x8F) > 0)
        if(dcb[4] == 0):
            info["model"] = "DT"
        elif(dcb[4] == 1):
            info["model"] = "DT-E"       
        elif(dcb[4] == 2):
            info["model"] = "PRT" 
        elif(dcb[4] == 3):
            info["model"] = "PRT-E"
        elif(dcb[4] == 4):
            info["model"] = "PRTHW"
        elif(dcb[4] == 5):
            info["model"] = "TM1"
        else:
            info["model"] = "Unknown"
# Set base address for objects in DCB 
# tb is time base address
# dl is DCB lenght  
# pm is program mode
# hr is holiday return base address
# oo is On/Off address
# kl is key lock address
# hwt is base address of hot water/timers in 7 day mode
# wt is base address of heating daily timers in 7 day mode
#
        tb = 41    
        dl = 72
        pm = 16
        hr = 25
        oo = 21
        kl = 22
        wt = 72
        if(dcb[4] == 4):
            tb = 44
            hwt = 191
            wt = 107
        if(dcb[4] == 5):
            tb = 19
            dl = 58
            pm = 6
            hr = 10
            oo = 8
            kl = 9
            hwt = 58
   
        if(dcb[4] <= 4): 
            if(dcb[5] == 0):
                info["temperature_format"] = "Celsius"
            else:
                info["temperature_format"] = "Fahrenheit"
            info["switch_differential"] = dcb[6]
            info["frost_protection_enable"] = (dcb[7] == 1)
            info["calibration_offset"] = ((dcb[8] << 8) | dcb[9])
            info["output_delay_in_minutes"] = dcb[10]
            # dcb[11] = address (not used)
            info['up_down_key_limit'] = dcb[12]
            if(dcb[13] == 0):
                info['sensor_selection'] = "Built in air sensor only"
            elif(dcb[13] == 1):
                info['sensor_selection'] = "Remote air sensor only"
            elif(dcb[13] == 2):
                info['sensor_selection'] = "Floor sensor only"
            elif(dcb[13] == 3):
                info['sensor_selection'] = "Built in air and floor sensor"
            elif(dcb[13] == 4):
                info['sensor_selection'] = "Remote air and floor sensor"
            else:
                info['sensor_selection'] = "Unknown"
                info['optimum_start'] = dcb[14]
                info['rate_of_change'] = dcb[15]
# Program mode is 16 for all devices except TM1 which is 6
# 'pm' is the value of program mode address
        if(dcb[pm] == 0):
            info['program_mode'] = "2/5 mode"
        else:
            info['program_mode'] = "7 day mode"
        if(dcb[4] <= 4):          
            info['frost_protect_temperature'] = dcb[17]
            info['set_room_temp'] = dcb[18]
            info['floor_max_limit'] = dcb[19]
            info['floor_max_limit_enable'] = (dcb[20] == 1)
   
#  On/Off is 21/21 (Write/Read) for all devices except TM1 which is 21/8
#  Key Lock is 22/22 for all devices except TM1 which is 22/9
        
        if(dcb[oo] == 1):
            info['on_off'] = "On"
        else:
            info['on_off'] = "Off"
        if(dcb[kl] == 0):
            info['key_lock'] = "Unlock"
        else:
            info['key_lock'] = "Lock"
        
        if(dcb[4] <= 4):                 
            if(dcb[23] == 0):
                info['run_mode'] = "Heating mode (normal mode)"
            else:
                info['run_mode'] = "Frost protection mode"
 
 # dcb[24] = away mode (not used)
        info['holiday_return_date_year'] = 2000 + dcb[hr]
        info['holiday_return_date_month'] = dcb[hr + 1]
        info['holiday_return_date_day_of_month'] = dcb[hr +2]
        info['holiday_return_date_hour'] = dcb[hr + 3]
        info['holiday_return_date_minute'] = dcb[hr + 4]
        info['holiday_enable'] = (dcb[hr + 5] == 1)
        
        if(dcb[4] <= 4):
            info['temp_hold_minutes'] = ((dcb[31] << 8) | dcb[32])
            if((dcb[13] == 1) or (dcb[13] == 4)):
                info['air_temp'] = (float((dcb[34] << 8) | dcb[33]) / 10.0)   
            if((dcb[13] == 2) or (dcb[13] == 3) or (dcb[13] == 4)):
                info['floor_temp'] = (float((dcb[36] << 8) | dcb[35]) / 10.0)               
            if((dcb[13] == 0) or (dcb[13] == 3)):
                info['air_temp'] = (float((dcb[38] << 8) | dcb[37]) / 10.0)
            info['error_code'] = dcb[39]
            info['heating_is_currently_on'] = (dcb[40] == 1)
        
        # Model DT and DT-E stops here
        if(dcb[4] <= 1):
            return info
# DCB Length is 72 except for TM! which is 52 
# Need to adjust code to accomadate TM1
        if(len(dcb) < dl):
            raise Exception("Size of DCB received from Thermostat is too small")

# Read the Time and Date values 
        info['year'] = 2000 + dcb[tb]
        info['month'] = dcb[tb + 1]
        info['day_of_month'] = dcb[tb + 2]
        info['weekday'] = dcb[tb + 3]
        info['hour'] = dcb[tb + 4]
        info['minute'] = dcb[tb + 5]
        info['second'] = dcb[tb + 6]
# collect each value for the date and time 
# convert to string values and combine in info as clock
        c1 = str(info['year'])
        c2 = str(info['month'])
        c3 = str(info['day_of_month'])
        c4 = (info['weekday'])
        c5 = str(info['hour'])
        c6 = str("{:02d}".format(info['minute']))
        c7 = str("{:02d}".format(info['second']))
        c4 = str(DoW[c4-1])
        info['clock'] = (c1)+'/'+(c2)+ '/'+ (c3) + ' ' + (c4) + ' ' + (c5) + ':' + (c6) + ':' + (c7)
        if(dcb[4] <= 4):
            info['weekday_triggers'] = self._get_info_time_triggers(dcb, 48)
            info['weekend_triggers'] = self._get_info_time_triggers(dcb, 60)
     
# -------------- IB code to put Timer info from TM1 & PRTHW into info -------------   
    
        if(dcb[4] == 4):     
            info['boost'] = ((dcb[41] << 8) | dcb[42])
            #info['hot_water_state'] = (dcb[43] == 1)
            if(dcb[43] == 1):
                info['hot_water_state'] = "On"
            else:
                info['hot_water_state'] = "Off"
            info['weekday_triggers'] = self._get_info_time_triggers(dcb, 51)
            info['weekend_triggers'] = self._get_info_time_triggers(dcb, 63) 
            info['weekday_hw_triggers'] = self._get_info_time_triggers_hw(dcb, 75)
            info['weekend_hw_triggers'] = self._get_info_time_triggers_hw(dcb, 91)   
        if(dcb[4] == 5):
            if(dcb[18] == 0):
                info['hot_water_state'] = "Off"
            else:
                info['hot_water_state'] = "On"
            info['weekday_hw_triggers'] = self._get_info_time_triggers_hw(dcb, 26)
            info['weekend_hw_triggers'] = self._get_info_time_triggers_hw(dcb, 42)
          
        
# ------------------------------------------------------------------------
        
        # If mode is 5/2 stop here
        if(dcb[pm] == 0):
            return info      
            
        if(len(dcb) < 156):
            raise Exception("Size of DCB received from Thermostat is too small")    
            
# Model PRT-HW has extra fields and offsets from the rest
# TM1 only has the HW/timer triggers
        if(dcb[4] <= 4):    
            info['mon_triggers'] = self._get_info_time_triggers(dcb, wt) 
            info['tue_triggers'] = self._get_info_time_triggers(dcb, wt + 12) 
            info['wed_triggers'] = self._get_info_time_triggers(dcb, wt + 24)
            info['thu_triggers'] = self._get_info_time_triggers(dcb, wt + 36)
            info['fri_triggers'] = self._get_info_time_triggers(dcb, wt + 48) 
            info['sat_triggers'] = self._get_info_time_triggers(dcb, wt + 60)
            info['sun_triggers'] = self._get_info_time_triggers(dcb, wt + 72)
        #if(dcb[4] == 4):
           # info['mon_triggers'] = self._get_info_time_triggers(dcb, 107) 
           # info['tue_triggers'] = self._get_info_time_triggers(dcb, 119) 
           # info['wed_triggers'] = self._get_info_time_triggers(dcb, 131)
           #info['thu_triggers'] = self._get_info_time_triggers(dcb, 143)
           # info['fri_triggers'] = self._get_info_time_triggers(dcb, 155) 
           #info['sat_triggers'] = self._get_info_time_triggers(dcb, 167)
           #info['sun_triggers'] = self._get_info_time_triggers(dcb, 179)  
        if(dcb[4] >= 4):
            info['mon_hw_triggers'] = self._get_info_time_triggers_hw(dcb, hwt) 
            info['tue_hw_triggers'] = self._get_info_time_triggers_hw(dcb, hwt + 16) 
            info['wed_hw_triggers'] = self._get_info_time_triggers_hw(dcb, hwt + 32)
            info['thu_hw_triggers'] = self._get_info_time_triggers_hw(dcb, hwt + 48)
            info['fri_hw_triggers'] = self._get_info_time_triggers_hw(dcb, hwt + 64) 
            info['sat_hw_triggers'] = self._get_info_time_triggers_hw(dcb, hwt + 80)
            info['sun_hw_triggers'] = self._get_info_time_triggers_hw(dcb, hwt + 96) 
        
        return info
        
    def set_clock(self):
		# import datetime
        # set tm to current time and date
        tm = datetime.datetime.now()
        # add each element to a varable year month day dayofweek hour minute seconds
        t1 = (int(tm.strftime("%y")))
        t2 = (int(tm.strftime("%m")))
        t3 = (int(tm.strftime("%d")))
        t4 = (int(tm.strftime("%w")))
        t5 = (int(tm.strftime("%H")))
        t6 = (int(tm.strftime("%M")))
        t7 = (int(tm.strftime("%S")))
        # Python day of the week returns '0' to '6' - Mon-Sun
        # Heatmiser uses day of the week 1 to 7 - Mon-Sun
        # convert Python to Heatmiser 
        t4 = t4 + 1
        # combine the varables into a list 
        value = [t1, t2, t3, t4, t5, t6, t7]
        #print ("set clock called")
        self.set_dcb(43,bytearray(value))

    def set_value(self, name, value):
        ''' Use the same name and value as returned in get_info. Only a few
            name/keys are supported in this implementation. Use the set_dcb
            method to set any value. '''
        if(name == "switch_differential"):
            self.set_dcb(6,bytearray([int(value)]))
        elif(name == "frost_protect_temperature"):
            self.set_dcb(17,bytearray([int(value)]))            
        elif(name == "set_room_temp"):
            self.set_dcb(18,bytearray([int(value)]))  
        elif(name == "floor_max_limit"):
            self.set_dcb(19,bytearray([int(value)]))  
        elif(name == "floor_max_limit_enable"):
            if((value == True) or (value == "True") or (value == "1") or (value == 1)):
                value = 1
            elif((value == False) or (value == "False") or (value == "0") or (value == 0)):
                value = 0
            else:
                raise Exception("'"+name+"' invalid value '"+str(value)+"'\n" +
                                "Valid values: True, 1, False or 0")
            self.set_dcb(20,bytearray([value]))  
        elif(name == "on_off"):
            if(value == "On"):
                value = 1
            elif(value == "Off"):
                value = 0
            else:
                raise Exception("'"+name+"' invalid value '"+str(value)+"'\n" +
                                "Valid values: 'On' or 'Off'")
            self.set_dcb(21,bytearray([value])) 
        elif(name == "key_lock"):
            if(value == "Lock"):
                value = 1
            elif(value == "Unlock"):
                value = 0
            else:
                raise Exception("'"+name+"' invalid value '"+str(value)+"'\n" +
                                "Valid values: 'Lock' or 'Unlock'")
            self.set_dcb(22,bytearray([value]))
        elif(name == "run_mode"):
            if(value == "Frost protection mode"):
                value = 1
            elif(value == "Heating mode (normal mode)"):
                value = 0
            else:
                raise Exception("'"+name+"' invalid value '"+str(value)+"'\n" +
                                "Valid values: \"Frost protection mode\" or " +
                                "\"Heating mode (normal mode)\"")
            self.set_dcb(23,bytearray([value]))   
        elif(name == "away_mode"):
            if(value == "off"):
                value = 0
            elif(value == "on"):
                value = 1
            else:
                raise Exception("'"+name+"' invalid value '"+str(value)+"'\n" +
                                "Valid values: 'on' or 'off'")
            self.set_dcb(31,bytearray([value]))         
        elif(name == "hot_water_state"):
            if(value == "off"):
                value = 2
            elif(value == "on"):
                value = 1
            elif(value == "prog"):
                value = 0    
            else:
                raise Exception("'"+name+"' invalid value '"+str(value)+"'\n" +
                                "Valid values: 'on', 'off' or 'prog'")
            self.set_dcb(42,bytearray([value]))     
        elif(name == "weekday_triggers"):
            self.set_dcb(47,bytearray(value))
        elif(name == "weekend_triggers"):
            self.set_dcb(59,bytearray(value))
        elif(name == "weekday_hw_triggers"):
            self.set_dcb(71,bytearray(value))
        elif(name == "weekday_hw_triggers"):
            self.set_dcb(87,bytearray(value))              
        elif(name == "mon_triggers"):
            self.set_dcb(103,bytearray(value))
        elif(name == "tue_triggers"):
            self.set_dcb(115,bytearray(value))
        elif(name == "wed_triggers"):
            self.set_dcb(127,bytearray(value))
        elif(name == "thu_triggers"):
            self.set_dcb(139,bytearray(value))            
        elif(name == "fri_triggers"):
            self.set_dcb(151,bytearray(value))
        elif(name == "sat_triggers"):
            self.set_dcb(163,bytearray(value))            
        elif(name == "sun_triggers"):
            self.set_dcb(175,bytearray(value))            
        elif(name == "mon_hw_triggers"):
            self.set_dcb(187,bytearray(value))
        elif(name == "tue_hw_triggers"):
            self.set_dcb(203,bytearray(value))
        elif(name == "wed_hw_triggers"):
            self.set_dcb(219,bytearray(value))
        elif(name == "thu_hw_triggers"):
            self.set_dcb(235,bytearray(value))            
        elif(name == "fri_hw_triggers"):
            self.set_dcb(251,bytearray(value))
        elif(name == "sat_hw_triggers"):
            self.set_dcb(267,bytearray(value))            
        elif(name == "sun_hw_triggers"):
            self.set_dcb(283,bytearray(value))
        elif(name == "clock"):
            self.set_clock()
        else:
            raise Exception("'"+name+"' not supported to be set")




###############################################################################
# Below is a command line tool for reading and setting parameters of a
# Heatmiser Wifi thermostat. It can also be seen as an example on how to use
# the library.


def print_dict(dict, level=""):
    for i in dict.items():
        if(isinstance(i[1],OrderedDict)):
            print(level+i[0]+":")
            print_dict(i[1],level + "    ")
        else:
            print(level+str(i[0])+" = "+str(i[1]))
        
def main():
    # This function shows how to use the Heatmiser class. 
    
    parser = OptionParser("Usage: %prog [options] <Heatmiser Thermostat address>")
    parser.add_option("-p", "--port", dest="port", type="int",
                      help="Port of HeatMiser Thermostat (default 8068)", default=8068) 
    parser.add_option("-c", "--pin", dest="pin", type="int",
                      help="Pin code of HeatMiser Thermostat (default 0000)", default=0)
    parser.add_option("-l", "--list", action="store_true", dest="list_all",
                      help="List all parameters in Thermostat", default=False)
    parser.add_option("-r", "--read",  dest="parameter",
                      help="Read one parameter in Thermostat (-r param)", default="")
    parser.add_option("-w", "--write",  dest="param_value", nargs=2,
                      help="Write value to parameter in Thermostat (-w param value, use the value now for the clock)")
    parser.add_option("-t", "--time", action="store_true", dest="time",
                      help="Update the current date & time on the Thermostat (-t), with no output")
    (options, args) = parser.parse_args()
    
    
    if (len(args) != 1):
        parser.error("Wrong number of arguments")
    
    host = args[0]
                      
    # Create a new Heatmiser object
    heatmiser = Heatmiser(host,options.port,options.pin)
    
    # Connect to Thermostat
    heatmiser.connect()
    
    # Read all parameters
    info = heatmiser.get_info()
    
    # Print all parameters in Thermostat
    if(options.list_all):
        print_dict(info)
        
    # set the date and time all of the Thermostat
    elif(options.time):
	    #print("set the clock")
	    heatmiser.set_clock()
	    #if (param == "clock"):
	    #info2 = heatmiser.get_info()
	    #param = "clock"
	    #print("Before change: " + param + " = " + str(info[param]))
	    #print("After change:  " + param + " = " + str(info2[param]))

        
    # Print one parameter in Thermostat
    elif(options.parameter != ""):
        if (options.parameter in info):
            print(options.parameter + " = " + str(info[options.parameter]))
        else:
            sys.stderr.write("Error!\n"+
            "1Parameter '"+options.parameter+"' does not exist\n")
    
    
    # Write value to one parameter in Thermostat
    elif(options.param_value != None):
        param = options.param_value[0]
        value = options.param_value[1]
        if (param in info):
            try:
                heatmiser.set_value(param,value)
                info2 = heatmiser.get_info()
                print("Before change: " + param + " = " + str(info[param]))
                print("After change:  " + param + " = " + str(info2[param]))
            except Exception as e:
                sys.stderr.write(e.args[0]+"\n")
        else:
            sys.stderr.write("Error!\n"+
                "Parameter '"+param+"' does not exist\n")
    heatmiser.disconnect()
        
if __name__ == '__main__':
    main()    