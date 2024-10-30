# microstrain_mv5_can_driver

See generic_can_driver for details

## Setup

1. Get a Microastrain MV5-AR J1939 IMU

2. Physically connect and power the sensor

3. Set up a CAN port on the host machine

   - For a PEAK CAN device:

       `sudo ip link set can0 up type can bitrate 250000 dbitrate 2000000 fd on fd-non-iso on `

   - For a Lawicel CANusb:

       `sudo slcand -o -c -f -s5 /dev/ttyUSB0 can0`
       
       `sudo ifconfig can0 up`

   - For a Virtual CAN network:
 
       `ip link add dev vcan0 type vcan`

       `ip link set up vcan0`

4. Modify `microstrain_mv5_can_driver/config/socketcan_params.yaml` for your interface (e.g. `can0`)

5. Modify `microstrain_mv5_can_driver/config/microstrain_mv5_can_params.yaml` for your device's id
   
   - This is the last two digits in the identifier of the sensor's CAN frame (e.g. E5) converted to 
     decimal

6. Launch `ros2 launch microstrain_mv5_can_driver microstrain_mv5_can.launch.py`


## Notes
 1. To connect phytools peak-can usb device run the following in host terminal:

``sudo ip link set can0 up type can bitrate 250000 dbitrate 2000000 fd on fd-non-iso on ``

3. Can address claim init output -> ``can0  18EEFFE5   [8]  00 00 20 47 00 91 00 20`` 
