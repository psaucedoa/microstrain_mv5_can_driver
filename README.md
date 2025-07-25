# microstrain_mv5_can_driver

Ros2 driver for the Microstrain MV5 J1939 CAN Driver. 
This includes a ros2 node and a ros2_control sensor interface

https://www.engr.colostate.edu/~jdaily/CyberBoat/Introduction%20to%20SAE%20J1939%20-%20CyberBoat%202022.pdf

## Setup

> make sure can-utils is installed (on bare-metal as well as your docker, just to make sure)

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

## Services

### Reset Attitude
Definition: Resets the attitude of the device through a PGN

Example: `put_example_service`

## Notes

1. Can address claim init output -> ``can0  18EEFFE5   [8]  00 00 20 47 00 91 00 20``
