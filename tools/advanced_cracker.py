import zlib
import struct
import binascii

TARGET = 0xFCCA02CA

# Candidates
PASSWORDS = [
    b"chuckpass", b"ChuckPass", b"CHUCKPASS", b"chuckpass ", b" chuckpass",
    b"bloodhound", b"password", b"teensyvoter",
]

CHALLENGES = [
    b"1234567890",             # My Server Challenge
    b"433463776",              # Client Challenge
    b"433463776\x00",          # Client Challenge + Null
    b"\x34\x33\x33\x34\x36\x33\x37\x37\x36\x00", # Raw Bytes
    b"\x00"*10,                # Zeros
    b"\x00"*4,
    b" "*10,                   # Spaces
    b"",                       # Empty
]

def v_crc(a, b): return zlib.crc32(b, zlib.crc32(a)) & 0xFFFFFFFF
def v_crc_inv(a, b): return zlib.crc32(a, zlib.crc32(b)) & 0xFFFFFFFF

print(f"--- Cracking {TARGET:08X} ---")

for p in PASSWORDS:
    for c in CHALLENGES:
        # Generate Candidates
        res1 = v_crc(c, p)
        res2 = v_crc_inv(c, p)
        res3 = zlib.crc32(c) & 0xFFFFFFFF # Ignore pwd
        res4 = zlib.crc32(p) & 0xFFFFFFFF # Ignore chall
        
        results = [res1, res2, res3, res4]
        
        # Check against Target and Endian-Swapped Target
        for res in results:
            if res == TARGET:
                print(f"[!] MATCH: {res:08X} (Direct) | P:{p} C:{c}")
            
            # Check Byte Swap (Big Endian vs Little Endian issues)
            # 0xFCCA02CA <-> 0xCA02CAFC
            bytes_val = struct.pack('<I', res)
            swapped = struct.unpack('>I', bytes_val)[0]
            if swapped == TARGET:
                 print(f"[!] MATCH: {res:08X} (Swapped) -> {TARGET:08X} | P:{p} C:{c}")

print("--- Done ---")
