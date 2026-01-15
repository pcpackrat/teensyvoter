import sys
import struct
import time
import socket

# Simple UDP Server to count packets per second
# Usage: python measure_bps.py <PORT>

def measure_pps(port=1667):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", port))
    print(f"Listening on port {port}...")
    
    count = 0
    start_time = time.time()
    
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            count += 1
            
            elapsed = time.time() - start_time
            if elapsed >= 1.0:
                print(f"[{time.ctime()}] Source: {addr[0]} | PPS: {count} packets/sec")
                count = 0
                start_time = time.time()
        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"Error: {e}")

if __name__ == "__main__":
    port = 1667
    if len(sys.argv) > 1:
        port = int(sys.argv[1])
    measure_pps(port)
