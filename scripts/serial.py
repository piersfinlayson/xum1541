#!/bin/python3
import sys
from intelhex import IntelHex

def write_serial(serial_num):
    if not 0 <= serial_num <= 255:
        print("Error: Serial number must be a single byte (0-255)")
        sys.exit(1)
    
    # Invert the value as per the C code: Size = ~eeprom_read_byte()
    eeprom_value = (~serial_num) & 0xFF
    
    # Create new IntelHex object
    ih = IntelHex()
    
    # Write the inverted serial number
    ih[0] = eeprom_value
    
    # Write to file
    filename = "eeprom-serial.hex"
    ih.write_hex_file(filename)
    print(f"Wrote serial number {serial_num} (encoded as 0x{eeprom_value:02X}) to {filename}")

def main():
    if len(sys.argv) != 2:
        print("Usage: {} <serial_number>".format(sys.argv[0]))
        print("       serial_number: decimal number 0-255")
        print("Example: {} 123".format(sys.argv[0]))
        sys.exit(1)
    
    try:
        serial_num = int(sys.argv[1])
        write_serial(serial_num)
    except ValueError:
        print("Error: Serial number must be a decimal number")
        sys.exit(1)

if __name__ == "__main__":
    main()
