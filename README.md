# microstrain_mv5_can_driver

## Lifecycle States
> Think of these states as configuring, activating, deactivating, ... the driver, not the device
### on_configure

### on_activate

### on_deactivate

### on_shutdown

### on_cleanup

## Errors


## Notes
 1. To connect phytools peak-can usb device run the following in host terminal:

``sudo ip link set can0 up type can bitrate 250000 dbitrate 2000000 fd on fd-non-iso on ``

3. Can address claim init output -> ``can0  18EEFFE5   [8]  00 00 20 47 00 91 00 20`` 