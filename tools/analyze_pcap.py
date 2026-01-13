import struct
import socket
import sys

def parse_pcap(filepath):
    print(f"Analyzing {filepath}...")
    
    unique_sources = {}
    
    try:
        with open(filepath, 'rb') as f:
            # Global Header
            global_header = f.read(24)
            if len(global_header) < 24:
                print("Error: File too short for global header")
                return

            # Magic Number check (d4 c3 b2 a1 or a1 b2 c3 d4)
            magic = global_header[0:4]
            # print(f"Magic: {magic.hex()}")

            packet_count = 0
            
            while True:
                # Packet Header (16 bytes)
                # ts_sec (4), ts_usec (4), incl_len (4), orig_len (4)
                pkt_header = f.read(16)
                if len(pkt_header) < 16:
                    break
                
                ts_sec, ts_usec, incl_len, orig_len = struct.unpack('<IIII', pkt_header)
                
                # Read Packet Data
                pkt_data = f.read(incl_len)
                if len(pkt_data) < incl_len:
                    print("Error: Incomplete packet data")
                    break
                
                packet_count += 1
                
                # Basic Parsing (Assuming Ethernet/IPv4)
                # Eth Header = 14 bytes
                # IP Header starts at 14. 
                # Check EthType (bytes 12-13). 0x0800 = IPv4
                
                if len(pkt_data) < 34: # Min size for Eth+IP
                    continue
                    
                eth_type = struct.unpack('>H', pkt_data[12:14])[0]
                if eth_type != 0x0800:
                    continue
                
                # IP Header
                ip_header_start = 14
                # Version/IHL byte
                ver_ihl = pkt_data[ip_header_start]
                ihl = (ver_ihl & 0x0F) * 4
                
                protocol = pkt_data[ip_header_start + 9]
                if protocol != 17: # UDP
                    continue
                    
                src_ip_bytes = pkt_data[ip_header_start+12 : ip_header_start+16]
                dst_ip_bytes = pkt_data[ip_header_start+16 : ip_header_start+20]
                
                src_ip = socket.inet_ntoa(src_ip_bytes)
                
                # UDP Header
                udp_header_start = ip_header_start + ihl
                src_port = struct.unpack('>H', pkt_data[udp_header_start:udp_header_start+2])[0]
                dst_port = struct.unpack('>H', pkt_data[udp_header_start+2:udp_header_start+4])[0]
                
                # Deep Inspection: Map IP -> Challenge -> Count
                # Voter Header starts at offset 0 of UDP Payload (after 8 byte UDP header)
                udp_payload = pkt_data[udp_header_start+8:]
                
                if dst_port == 1667 and len(udp_payload) >= 20: 
                         # Challenge is at offset 8 (after VTIME_SEC(4) + VTIME_NSEC(4))
                         # Length 10 bytes
                         challenge_bytes = udp_payload[8:18]
                         try:
                            challenge_str = challenge_bytes.decode('ascii')
                         except:
                            challenge_str = f"HEX:{challenge_bytes.hex()}"
                         
                         if src_ip not in unique_sources:
                             unique_sources[src_ip] = {}
                         
                         if challenge_str not in unique_sources[src_ip]:
                             unique_sources[src_ip][challenge_str] = 0
                         unique_sources[src_ip][challenge_str] += 1

            print(f"Total Packets Scanned: {packet_count}")
            print("\n--- Challenge ID Distribution per Source IP ---")
            if not unique_sources:
                print("No UDP packets found on port 1667.")
            else:
                for ip, challenges in unique_sources.items():
                    print(f"Source IP: {ip}")
                    for chal, count in challenges.items():
                        print(f"  - Challenge: {chal} | Count: {count}")
                
    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    print("--- Analyzing DUPLICATE Trace ---")
    parse_pcap(r"c:\Users\mikec\Documents\Projects\VOTER\TeensyVoter\traces\duplicate_trace.pcap")
