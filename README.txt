#
#  Program name: serial
#  Platform: GNU/Linux
#  Author: Zhuo Li
#  Late Modification: March, 1st, 2015
#  Brief Description: This program can realize the communication with 
#                     NISTICA WSS board via serial port under Linux environment. 
#  Copyright: Lightwave Research Laboratory @ Columbia Unviersity
#

A. Input format
    ./serial <command> [option]

	current support <command>:
	1. modinfo                                        get module information
	2. channel.<WSS ID>.<channel ID>.<Port ID>        switch channel
	3. attenua.<WSS ID>.<channel ID>.<Attenuation>    set attenuation

	note:
	1) <WSS ID> can only be 0 or 1
	2) <channel ID> should be in range of 0 ~ 95. It should always be of two digits. e.g. channel 2 should be input as 02
	3) <Port ID> can only be 0, 1 or 2. 0 for OFF status.
	4) <Attenuation> has a unit of 0.1dB. e.g. if the input is 20, it means the attenuation of 2dB

examples:

./serial channel.0.07.1
This means "Switch 07 channel in WSS A to port 1

./serial attenua.1.14.30
This means "Attenuate 14 channel in WSS B by 3dB

B. Other notes
In order to communiate with the board via serial port, the baud rate can only be setted as 115200
