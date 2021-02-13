# jetnano_python
Jetson Nano Robot Control - Python,Modbus,HC-12


Need all those dependencies in the import section. Pyrealsense python wrapper might be the most difficult.

Run pijoy snd sync_server on the Raspberry Pi. Have to setup Bluetooth controller or any pygame controllers on the Pi.
Need to edit ip addresses to point to the Pi and it is hardcoded to look for my Bluetooth controller by address so that needs to be changed or omitted.

Run drivenano on the Jetson Nano. Uses modbus over wifi if its available, otherwise switches to the HC-12
