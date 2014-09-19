#!/usr/bin/env python
# /* -*-  indent-tabs-mode:t; tab-width: 8; c-basic-offset: 8  -*- */
# /*
# Copyright (c) 2014, Daniel M. Lofaro <dan (at) danLofaro (dot) com>
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the author nor the names of its contributors may
#       be used to endorse or promote products derived from this software
#       without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
# PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
# LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
# OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
# ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# */
import diff_drive
import ach
import sys
import time
from ctypes import *
import socket
import cv2.cv as cv
import cv2
import numpy as np

import actuator_sim as ser
#-----------------------------------------------------
#--------[ Do not edit above ]------------------------
#-----------------------------------------------------

# Add imports here
from robot_command import createCommandPacket
from params import multiplyVelocity


#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
dd = diff_drive
ref = dd.H_REF()
tim = dd.H_TIME()

ROBOT_DIFF_DRIVE_CHAN   = 'robot-diff-drive'
ROBOT_CHAN_VIEW   = 'robot-vid-chan'
ROBOT_TIME_CHAN  = 'robot-time'
# CV setup 
r = ach.Channel(ROBOT_DIFF_DRIVE_CHAN)
r.flush()
t = ach.Channel(ROBOT_TIME_CHAN)
t.flush()

i=0


print '======================================'
print '============= Robot-View ============='
print '========== Daniel M. Lofaro =========='
print '========= dan@danLofaro.com =========='
print '======================================'
ref.ref[0] = 0
ref.ref[1] = 0
while True:
	[status, framesize] = t.get(tim, wait=False, last=True)
	if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
		pass
		#print 'Sim Time = ', tim.sim[0]
	else:
		raise ach.AchException( v.result_string(status) )

#-----------------------------------------------------
#--------[ Do not edit above ]------------------------
#-----------------------------------------------------
    # Main Loop
    # Def:
    # tim.sim[0] = Sim Time
	
	#wait for sim time to correct; it always seems to start at zero regardless of actual sim time
	prev_tim = tim.sim[0]	
	while prev_tim == 0:
		time.sleep(0x05)
		[status, framesize] = t.get(tim, wait=False, last=True)
		if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
			pass
			#print 'Sim Time = ', tim.sim[0]
		else:
			raise ach.AchException( v.result_string(status) )
		prev_tim = tim.sim[0]

	print 'Sim Time = ', tim.sim[0]
	
	# Sleeps

	#time.sleep(0.1)
	
	i = 0
	while i < 4:
		j = 0
		while j < 4:
			#move straight
			fullSpeedAhead = [0xFF, 0x07]
			fullReverse = [0xFF, 0x03]
			fullStop = [0x00, 0x00]
			buff = createCommandPacket(0x00, 4, 0x20, fullSpeedAhead)
			ref = ser.serial_sim(r,ref,buff)
			buff = createCommandPacket(0x01, 4, 0x20, fullSpeedAhead)
			ref = ser.serial_sim(r,ref,buff)

			#wait for a couple seconds before slowing down down 
			prev_tim = tim.sim[0]			
			while tim.sim[0] < prev_tim + 4.0:
				time.sleep(0.05)
				[status, framesize] = t.get(tim, wait=False, last=True)
				if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
					pass
					print 'Moving straight, Sim Time = ', tim.sim[0]
				else:
					raise ach.AchException( v.result_string(status) )
			
			#slow down before turning
			k = 160
			while True: #do-while loop.  condition ends at k=0
				k -=1
				if (k==0):
					break
				time.sleep(0.05)
				[status, framesize] = t.get(tim, wait=False, last=True)
				if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
					pass
					print 'Slowing movement, Sim Time = ', tim.sim[0]
				else:
					raise ach.AchException( v.result_string(status) )
				slowDown = multiplyVelocity(fullSpeedAhead, 1.0-(1.0/float(k)))
				buff = createCommandPacket(0x00, 4, 0x20, slowDown)
				ref = ser.serial_sim(r,ref,buff)
				buff = createCommandPacket(0x01, 4, 0x20, slowDown)
				ref = ser.serial_sim(r,ref,buff)
			
			#full stop to stabilize  --turns out stopping is the fastest way to destabilize this robot
#			buff = createCommandPacket(0x00, 4, 0x20, fullStop)
#			ref = ser.serial_sim(r,ref,buff)
#			buff = createCommandPacket(0x01, 4, 0x20, fullStop)
#			ref = ser.serial_sim(r,ref,buff)
#			prev_tim = tim.sim[0]
#			while tim.sim[0] < prev_tim + 1.0:
#				time.sleep(0.05)
#				[status, framesize] = t.get(tim, wait=False, last=True)
#				if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
#					pass
#					print 'Full stop, Sim Time = ', tim.sim[0]
#				else:
#					raise ach.AchException( v.result_string(status) )

			#turn
			buff = createCommandPacket(0x00, 4, 0x20, fullReverse)
			ref = ser.serial_sim(r,ref,buff)
			buff = createCommandPacket(0x01, 4, 0x20, fullSpeedAhead)
			ref = ser.serial_sim(r,ref,buff)
			
			#wait for the time it takes to turn 90 degrees.
#			prev_tim = tim.sim[0]
#			while tim.sim[0] < prev_tim + 0.5:
#				time.sleep(0.05)
#				[status, framesize] = t.get(tim, wait=False, last=True)
#				if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
#					pass
#					print 'Turning, Sim Time = ', tim.sim[0]
#				else:
#					raise ach.AchException( v.result_string(status) )
			
			k = 45
			while True: #do-while loop.  condition ends at k=0
				k -=1
				if (k==0):
					break
				time.sleep(0.05)
				[status, framesize] = t.get(tim, wait=False, last=True)
				if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
					pass
					print 'slowing turn, Sim Time = ', tim.sim[0]
				else:
					raise ach.AchException( v.result_string(status) )
				slowDown = multiplyVelocity(fullReverse, 1.0-(1.0/float(k)))
				buff = createCommandPacket(0x00, 4, 0x20, slowDown)
				ref = ser.serial_sim(r,ref,buff)
				slowDown = multiplyVelocity(fullSpeedAhead, 1.0-(1.0/float(k)))
				buff = createCommandPacket(0x01, 4, 0x20, slowDown)
				ref = ser.serial_sim(r,ref,buff)
			
			#full stop to stabilize
			buff = createCommandPacket(0x00, 4, 0x20, fullStop)
			ref = ser.serial_sim(r,ref,buff)
			buff = createCommandPacket(0x01, 4, 0x20, fullStop)
			ref = ser.serial_sim(r,ref,buff)
			prev_tim = tim.sim[0]
			while tim.sim[0] < prev_tim + 2.0:
				time.sleep(0.05)
				[status, framesize] = t.get(tim, wait=False, last=True)
				if status == ach.ACH_OK or status == ach.ACH_MISSED_FRAME or status == ach.ACH_STALE_FRAMES:
					pass
					print 'Full stop, Sim Time = ', tim.sim[0]
				else:
					raise ach.AchException( v.result_string(status) )

			j = j+1
		i = i+1
	break
buff = createCommandPacket(0x00, 4, 0x20, [0x00, 0x00])
ref = ser.serial_sim(r,ref,buff)
buff = createCommandPacket(0x01, 4, 0x20, [0x00, 0x00])
ref = ser.serial_sim(r,ref,buff)

#-----------------------------------------------------
#--------[ Do not edit below ]------------------------
#-----------------------------------------------------
