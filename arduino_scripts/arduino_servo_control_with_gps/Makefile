

# target: dependencies
#			action
# -------------------------------------------------- #


# variables
BOARD = arduino:avr:mega
PORT = /dev/ttyACM*
BAUD = 9600

#PYTHON_SCRIPT = ./custom_arduino-cli_read_serial_port.py


# build commands
b:
	arduino-cli compile -b ${BOARD} . && echo Build Complete

bu:
	arduino-cli compile -b ${BOARD} . && echo Build Complete && echo Uploading... && arduino-cli upload --verify -p ${PORT} -b ${BOARD} . && echo Upload Complete

# CTRL+C stops reading serial data, window closes.
bus:
	arduino-cli compile -b ${BOARD} . && echo Build Complete && echo Uploading... && arduino-cli upload --verify -p ${PORT} -b ${BOARD} . && echo Upload Complete && gnome-terminal -e 'bash -c "stty -F ${PORT} ${BAUD} raw -echo && cat ${PORT}"'

# CTRL+C stops reading serial data, window stays open. (usefull to view data that prints "too fast")
busp:
	arduino-cli compile -b ${BOARD} . && echo Build Complete && echo Uploading... && arduino-cli upload --verify -p ${PORT} -b ${BOARD} . && echo Upload Complete && gnome-terminal -e 'bash -c "trap bash SIGINT; stty -F ${PORT} ${BAUD} raw -echo && cat ${PORT}"'

# 
s:
	gnome-terminal -e 'bash -c "stty -F ${PORT} ${BAUD} raw -echo && cat ${PORT}"'

sp:
	gnome-terminal -e 'bash -c "trap bash SIGINT; stty -F ${PORT} ${BAUD} raw -echo && cat ${PORT}"'

c:
	rm -f ..*.*.*.* 


# NOTE:
# 
# To use a python2.7 script for serial monitoring
# instead of bash, replace the line under 'serial:'
# with:
#
#   	arduino-cli compile -b ${BOARD} . && echo Build Complete && echo Uploading... && arduino-cli upload --verify -p ${PORT} -b ${BOARD} . && echo Upload Complete && gnome-terminal -e 'bash -c " ./${PYTHON_SCRIPT} ${PORT} ${BAUD}"'
# 
# And save the following python script in the same directory as this Makefile (make sure it is executable with chmod +x):


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



# NOTE:
# 
# Here is a Sublime3 custom build system that will work 
# with this makefile:

# {
#     "shell_cmd": "make upload",
#     /* NOTE: Add file_regex here if desired */
#     "working_dir": "${folder:${project_path:${file_path}}}",
#     "selector": "source.makefile",
#     "syntax": "Packages/Makefile/Make Output.sublime-syntax",
#     "keyfiles": ["Makefile", "makefile"],

#     "variants":
#     [
#         {
#             "name": "build",
#             "shell_cmd": "make b"
#         },
#         {
#             "name": "build, upload",
#             "shell_cmd": "make bu"
#         },
#         {
#             "name": "build, upload, serial",
#             "shell_cmd": "make bus"
#         },
#         {
#             "name": "build, upload, serial (pause)",
#             "shell_cmd": "make busp"
#         }, 
#         {
#             "name": "serial monitor",
#             "shell_cmd": "make s"
#         },
#         {
#             "name": "serial monitor (pause)",
#             "shell_cmd": "make sp"
#         },
#         {
#             "name": "clean",
#             "shell_cmd": "make c"
#         },
#     ]
# }


