# https://mjoldfield.com/atelier/2009/02/arduino-cli.html

# target: dependencies
#			action
# -------------------------------------------------- #


# variables
BOARD = arduino:avr:mega
PORT = /dev/ttyACM*
BAUD = 115200

#PYTHON_SCRIPT = ./custom_arduino-cli_read_serial_port.py


# build commands
build:
	arduino-cli compile -b ${BOARD} . && echo Build Complete

upload:
	arduino-cli compile -b ${BOARD} . && echo Build Complete && echo Uploading... && arduino-cli upload --verify -p ${PORT} -b ${BOARD} . && echo Upload Complete

# CTRL+C stops reading serial data, window closes.
serial:
	arduino-cli compile -b ${BOARD} . && echo Build Complete && echo Uploading... && arduino-cli upload --verify -p ${PORT} -b ${BOARD} . && echo Upload Complete && gnome-terminal -e 'bash -c "stty -F ${PORT} ${BAUD} raw -echo && cat ${PORT}"'

#CTRL+C stops reading serial data, window stays open. (usefull to view data that prints "too fast")
pause:
	arduino-cli compile -b ${BOARD} . && echo Build Complete && echo Uploading... && arduino-cli upload --verify -p ${PORT} -b ${BOARD} . && echo Upload Complete && gnome-terminal -e 'bash -c "trap bash SIGINT; stty -F ${PORT} ${BAUD} raw -echo && cat ${PORT}"'

clean:
	rm ..*.*.*.* 



# NOTE:
# 
# To use a python script for serial monitoring,
# instead of bash, replace the line under 'serial:'
# with:
#
#   	arduino-cli compile -b ${BOARD} . && echo Build Complete && echo Uploading... && arduino-cli upload --verify -p ${PORT} -b ${BOARD} . && echo Upload Complete && gnome-terminal -e 'bash -c " ./${PYTHON_SCRIPT} ${PORT} ${BAUD}"'
# 
# And use the following python script:


# #!/usr/bin/env python
# # python2.7

# import serial
# import time
# import sys


# PORT = sys.argv[1]
# BAUD  = sys.argv[2]

# try:
# 	ser = serial.Serial(PORT, BAUD)

# 	while True:
# 	    serial_line = ser.readline()
# 	    print(serial_line), 

# 	    # If using Python 2.x use: print serial_line
# 	    # Do some other work on the data

# 	    #time.sleep(300) # sleep 5 minutes

# 	    # Loop restarts once the sleep is finished

# except KeyboardInterrupt:
# 	ser.close() # Only executes once the loop exits

