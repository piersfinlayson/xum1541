# Instructions for writing a non-zero serial number to your xum1541

First download and install [dfu-programmer](https://github.com/dfu-programmer/dfu-programmer/releases), into your `/usr/local/bin` directory.

Use [`scripts/serial.py`](https://raw.githubusercontent.com/piersfinlayson/xum1541/refs/heads/main/scripts/serial.py) to produce an eeprom-serial.hex file.  You will need to `pip install intelhex` first, if you don't have that package installed already.

```
pip install intelhex
scripts/serial.py 123
```

Get [`xum1541-ZOOMFLOPPY-v08.hex`](https://raw.githubusercontent.com/OpenCBM/OpenCBM/refs/heads/master/xum1541/xum1541-ZOOMFLOPPY-v08.hex) (or later) from the OpenCBM project.

Then run the following in order, where the order is important, as you cannot write the eeprom after flashig the firmware.

```
dfu-programmer atmega32u2 erase --force
dfu-programmer atmega32u2 flash --eeprom eeprom-serial.hex
dfu-programmer atmega32u2 flash  xum1541-ZOOMFLOPPY-v08.hex
dfu-programmer reset
```

If you are using usbipd to attach your xum1541 to WSL, you will need to re-bind the xum1541 after changing its serial number.
