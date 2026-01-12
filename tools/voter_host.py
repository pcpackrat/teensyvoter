import socket
import struct
import time
import zlib
import binascii
import argparse

# Configuration Defaults (can be overridden by CLI args)
DEFAULT_PORT = 1667
DEFAULT_HOST_PWD = "bloodhound"

# Known Clients (Map Name -> Password) for verification
CLIENTS = {
    "teensyvoter": "teensyvoter",
    "Teensy_test": "teensyvoter", # For user's specific case
    "test": "test",
    "Chuck_Voter": "chuckpass", # Reference Voter
}

# Protocol Constants
PAYLOAD_AUTH = 0
PAYLOAD_ULAW = 1
PAYLOAD_GPS  = 2 # Assuming GPS type for Keepalive/Location updates

# Header Format (Big Endian)
#   u32 vtime_sec
#   u32 vtime_nsec
#   u8  challenge[10]
#   u32 digest
#   u16 type
HEADER_FMT = '>II10sIH'
HEADER_SIZE = struct.calcsize(HEADER_FMT)

def voter_crc32(challenge_bytes, password_bytes):
    """
    Simulate the Voter Protocol CRC32 logic.
    CRC32(Challenge + Password).
    The arguments should be bytes.
    Use standard zlib.crc32 which matches the STM32 hardware CRC usually, 
    but note Voter.c uses a software table. Python's zlib is standard Ethernet CRC32.
    """
    # Voter.c logic:
    # crc = 0xFFFFFFFF
    # for b in buf1: crc = tab[(crc ^ b) & 0xFF] ^ (crc >> 8)
    # for b in buf2: crc = tab[(crc ^ b) & 0xFF] ^ (crc >> 8)
    # return ~crc
    
    # In Python, zlib.crc32(data, start) does exactly this if chained.
    
    # Strip null terminators if present for C-string emulation
    def strip_null(b):
        if b'\x00' in b:
            return b[:b.index(b'\x00')]
        return b

    c_clean = strip_null(challenge_bytes)
    p_clean = strip_null(password_bytes)

    crc = zlib.crc32(c_clean)
    crc = zlib.crc32(p_clean, crc)
    return crc & 0xFFFFFFFF

def hex_dump(data):
    return binascii.hexlify(data).decode('ascii')

def main():
    parser = argparse.ArgumentParser(description="Voter Protocol Analyzer & Mock Host")
    parser.add_argument('--port', type=int, default=DEFAULT_PORT, help='UDP Port to listen on (Default: 1667)')
    parser.add_argument('--host-pwd', type=str, default=DEFAULT_HOST_PWD, help='Host Password (for signing replies)')
    args = parser.parse_args()

    print(f"[*] Voter Analyzer running on 0.0.0.0:{args.port}")
    print(f"[*] Host Password: '{args.host_pwd}'")
    print(f"[*] Known Clients: {list(CLIENTS.keys())}")
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.bind(('0.0.0.0', args.port))
    except Exception as e:
        print(f"[!] Bind Error: {e}")
        return

    # Server's Challenge (Constant for this session)
    server_challenge_str = "1234567890" # Fixed for consistency
    server_challenge_bytes = server_challenge_str.encode('ascii')

    while True:
        try:
            data, addr = sock.recvfrom(4096)
            timestamp = time.strftime('%H:%M:%S')

            if len(data) < HEADER_SIZE:
                print(f"[{timestamp}] [!] Short Packet from {addr} ({len(data)} bytes)")
                continue

            # Unpack Header
            vtime_sec, vtime_nsec, client_challenge_raw, rx_digest, ptype = struct.unpack(HEADER_FMT, data[:HEADER_SIZE])
            
            # Clean up Challenge String
            client_challenge_clean = client_challenge_raw.decode('ascii', errors='ignore').strip('\x00')
            
            ptype_str = "UNKNOWN"
            if ptype == PAYLOAD_AUTH: ptype_str = "AUTH"
            elif ptype == PAYLOAD_ULAW: ptype_str = "AUDIO"
            elif ptype == PAYLOAD_GPS:  ptype_str = "GPS/KEEPALIVE"

            print(f"----------------------------------------------------------------")
            print(f"[{timestamp}] RX from {addr[0]}:{addr[1]} | Type: {ptype_str} ({ptype}) | Len: {len(data)}")
            print(f"    Raw Challenge: {hex_dump(client_challenge_raw)} ('{client_challenge_clean}')")
            print(f"    Rx Digest:     {rx_digest:08X}")

            # Verify Digest against known clients - FORENSIC MODE
            match_found = False
            
            # Candidates for Password
            pwd_candidates = [(k, v) for k, v in CLIENTS.items()]
            pwd_candidates.append(("HOST_PWD", args.host_pwd))
            
            # Candidates for Challenge
            chall_candidates = [
                ("ServerChall", server_challenge_bytes),
                ("ClientChall", client_challenge_raw),
                ("ZeroChall", b'\x00'*10),
                ("EmptyChall", b''),
                ("SpaceChall", b' '*10),
            ]
            
            for client_name, client_pwd in pwd_candidates:
                pwd_bytes = client_pwd.encode('ascii')
                
                for chall_name, chall_bytes in chall_candidates:
                    # Check Normal Order: CRC32(Challenge, Pwd)
                    d1 = voter_crc32(chall_bytes, pwd_bytes)
                    if d1 == rx_digest:
                         print(f"    [+] FORENSIC MATCH: Client '{client_name}' using {chall_name} + Pwd (Normal Order)")
                         match_found = True
                         break
                    
                    # Check Swapped Order: CRC32(Pwd, Challenge)
                    d2 = voter_crc32(pwd_bytes, chall_bytes)
                    if d2 == rx_digest:
                         print(f"    [+] FORENSIC MATCH: Client '{client_name}' using Pwd + {chall_name} (Swapped Order)")
                         match_found = True
                         break
                
                if match_found: break

            if not match_found:
                print(f"    [!] DIGEST MISMATCH! Rx: {rx_digest:08X}")
                # Print expected values for the first client (helpful for debugging)
                ref_client = "Chuck_Voter"
                if ref_client in CLIENTS:
                     p = CLIENTS[ref_client].encode('ascii')
                     print(f"        Expected ({ref_client} + SvrChall): {voter_crc32(server_challenge_bytes, p):08X}")
                     print(f"        Expected ({ref_client} + Empty):    {voter_crc32(b'', p):08X}")
                     print(f"        Expected ({ref_client} + Zero):     {voter_crc32(b'\x00'*10, p):08X}")
                     print(f"        Expected ({ref_client} + CliChall): {voter_crc32(client_challenge_raw, p):08X}")

            # Handling Logic
            if ptype == PAYLOAD_AUTH:
                # Always Reply to Auth with Host Signed Challenge
                calc_digest = voter_crc32(client_challenge_raw, args.host_pwd.encode('ascii'))
                
                reply_header = struct.pack(HEADER_FMT,
                                           vtime_sec,
                                           vtime_nsec,
                                           server_challenge_bytes + b'\x00'*(10-len(server_challenge_bytes)),
                                           calc_digest,
                                           PAYLOAD_AUTH)
                
                sock.sendto(reply_header, addr)
                print(f"    >>> Sent AUTH Reply (Digest: {calc_digest:08X}) w/ Challenge '{server_challenge_str}'")

            elif ptype == PAYLOAD_ULAW:
                if len(data) > HEADER_SIZE:
                    rssi = data[HEADER_SIZE]
                    print(f"    RSSI: {rssi}")
            
            elif ptype == PAYLOAD_GPS:
                 # Logic to print GPS info if we knew the struct layout
                 print(f"    GPS/Keepalive Payload Present")

        except KeyboardInterrupt:
            break
        except Exception as e:
            print(f"[!] Error: {e}")
            import traceback
            traceback.print_exc()

if __name__ == "__main__":
    main()
