import struct
import sys
import socket

def parse_pcap(filename):
    with open(filename, 'rb') as f:
        # Global Header
        header = f.read(24)
        if len(header) < 24:
            print("Empty file")
            return
        
        # Magic (4), Major(2), Minor(2), Zone(4), SigFigs(4), SnapLen(4), Network(4)
        magic, major, minor, zone, sig, snap, network = struct.unpack('IHHIIII', header)
        print(f"PCAP Network Type: {network} (1=Ethernet, 101=Raw IP)")

        packet_count = 0
        voter_count = 0
        
        while True:
            # Packet Header
            header = f.read(16)
            if len(header) < 16:
                break
            
            # Timestamp Sec(4), Micro(4), InclLen(4), OrigLen(4)
            ts_sec, ts_usec, incl_len, orig_len = struct.unpack('IIII', header)
            
            # Read Data
            data = f.read(incl_len)
            
            # Simple Parser for Ethernet -> IP -> UDP
            # Ethernet Header = 14 bytes
            # IP Header = 20 bytes (usually)
            # UDP Header = 8 bytes
            
            offset = 0
            if network == 1: # Ethernet
                offset = 14
            elif network == 113: # Linux SLL (Cooked)
                offset = 16
            
            if len(data) < offset + 20 + 8:
                continue

            # Check IP Protocol (Byte 9 in IP Header)
            ip_proto = data[offset + 9]
            if ip_proto != 17: # UDP
                continue
            
            # Extract IPs
            src_ip = socket.inet_ntoa(data[offset+12:offset+16])
            dst_ip = socket.inet_ntoa(data[offset+16:offset+20])
                
            # UDP Dest Port (Bytes 2-3 in UDP Header)
            udp_offset = offset + 20 # Assuming 20 byte IP header (IHL=5)
            # Verify IHL
            ihl = (data[offset] & 0x0F) * 4
            udp_offset = offset + ihl
            
            src_port, dst_port, udp_len, udp_sum = struct.unpack('!HHHH', data[udp_offset:udp_offset+8])
            
            if dst_port == 1667 or src_port == 1667:
                voter_count += 1
                payload = data[udp_offset+8:]
                print(f"\nPacket #{packet_count+1} {src_ip}:{src_port} -> {dst_ip}:{dst_port} Len:{len(payload)}")
                
                # Parse Voter Packet
                # VTIME(8), Challenge(10), Digest(4), Type(2)
                if len(payload) >= 24:
                    v_sec, v_nsec = struct.unpack('!II', payload[0:8])
                    
                    # Challenge (Skipping print)
                    
                    digest = struct.unpack('!I', payload[18:22])[0]
                    p_type = struct.unpack('!H', payload[22:24])[0]
                    
                    print(f"  Voter Time: {v_sec}.{v_nsec}")
                    print(f"  Digest: 0x{digest:08X}")
                    print(f"  Type: {p_type} ({'ULAW' if p_type==1 else 'Unknown'})")
                    
                    if p_type == 1 and len(payload) >= 25:
                        rssi = payload[24]
                        print(f"  RSSI: {rssi}")
                        # Audio
                        audio = payload[25:]
                        print(f"  Audio Bytes: {len(audio)}")
                        # Check for silence (0x00, 0xFF, or 0x7F/0xFF in uLaw)
                        # uLaw silent is usually 0xFF
                        zeros = audio.count(0x00)
                        ffs = audio.count(0xFF)
                        print(f"  Content: Zeros={zeros}, FFs={ffs} (Silence?)")
                else:
                    print("  [Short Packet]")

            packet_count += 1
            
        print(f"\nTotal Packets: {packet_count}")
        print(f"Voter Packets: {voter_count}")

if __name__ == "__main__":
    import sys
    if len(sys.argv) > 1:
        parse_pcap(sys.argv[1])
    else:
        # Default path
        parse_pcap(r"c:\Users\mikec\Documents\Projects\VOTER\TeensyVoter\traces\trace.pcap")
