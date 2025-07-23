import serial

ser = serial.Serial('/dev/ttyTHS1', 115200)

def read_distance_only():
    while True:
        # Wait for valid frame header
        if ser.read() == b'\x59':
            if ser.read() == b'\x59':
                # Read remaining 7 bytes of the frame
                rest = ser.read(7)
                if len(rest) != 7:
                    continue  # incomplete frame, skip

                dist_l = rest[0]
                dist_h = rest[1]
                distance = dist_h * 256 + dist_l

                return distance
distance_cm = read_distance_only()
while True:
    print(f"Distance: {distance_cm} cm")
