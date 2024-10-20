from smbus import SMBus
import time
import struct

def write_byte_data(bus, addr, value):
    try:
        bus.write_byte(addr, int(value) & 0xFF)
    except ValueError:
        print(f"Invalid value: {value}. Skipping this write.")

def get_float_input(prompt):
    while True:
        try:
            return float(input(prompt))
        except ValueError:
            print("Please enter a valid number.")

def write_sequence(addr=0x8):
    try:
        bus = SMBus(1)
        print(f"Connected to I2C device at address 0x{addr:02X}")
        
        while True:
            choice = input("Choose 1 (distance/angle) or 2 (long/lat) or q to quit: ").lower()
            
            if choice == 'q':
                break
            elif choice == '1':
                write_byte_data(bus, addr, 1)
                time.sleep(0.5)
                
                distance = get_float_input("Enter target distance: ")
                angle = get_float_input("Enter target angle: ")
                
                # Convert float to bytes and send
                distance_bytes = struct.pack('!f', distance)
                angle_bytes = struct.pack('!f', angle)
                
                for b in distance_bytes + angle_bytes:
                    write_byte_data(bus, addr, b)
                    time.sleep(0.1)
                
                print(f"Sent: Distance = {distance}, Angle = {angle}")
                
            elif choice == '2':
                write_byte_data(bus, addr, 2)
                time.sleep(0.5)
                
                longitude = get_float_input("Enter target longitude: ")
                latitude = get_float_input("Enter target latitude: ")
                
                # Convert float to bytes and send
                long_bytes = struct.pack('!f', longitude)
                lat_bytes = struct.pack('!f', latitude)
                
                for b in long_bytes + lat_bytes:
                    write_byte_data(bus, addr, b)
                    time.sleep(0.1)
                
                print(f"Sent: Longitude = {longitude}, Latitude = {latitude}")
            
            else:
                print("Invalid choice. Please enter '1', '2', or 'q'")
    
    except IOError as e:
        print(f"I2C Communication Error: {e}")
    except KeyboardInterrupt:
        print("\nProgram terminated by user")
    except Exception as e:
        print(f"Unexpected error: {e}")
    finally:
        if 'bus' in locals():
            bus.close()
            print("Bus closed")

if __name__ == '__main__':
    write_sequence()
