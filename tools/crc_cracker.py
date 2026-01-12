import zlib
import struct
import binascii

# Target Digest from User Log
TARGET = 0xFCCA02CA
TARGET_ZERO = 0x00000000

# Ingredients
PASSWORDS = [
    b"chuckpass", 
    b"chuckpass\x00", 
    b"bloodhound", 
    b"password", 
    b""
]

CHALLENGES = [
    b"1234567890",             # My Server Challenge
    b"433463776",              # Client Challenge (ASCII)
    b"433463776\x00",          # Client Challenge (ASCII + Null)
    b"\x34\x33\x33\x34\x36\x33\x37\x37\x36\x00", # Raw Bytes from Log
    b"\x00"*10,                # Zero Challenge
    b" "*10,                   # Space Challenge
    b"",                       # Empty Challenge
    b"80932935",               # Previous Teensy Challenge (Just in case)
]

def v_crc(a, b):
    # Standard Model: CRC(a) -> CRC(b, previous)
    return zlib.crc32(b, zlib.crc32(a)) & 0xFFFFFFFF

def v_crc_inv(a, b):
    # Inverted Model: CRC(b) -> CRC(a, previous)
    return zlib.crc32(a, zlib.crc32(b)) & 0xFFFFFFFF

def check(name, val):
    if val == TARGET:
        print(f"[!] MATCH FOUND for FCCA02CA: {name}")
    if val == TARGET_ZERO:
        print(f"[!] MATCH FOUND for 00000000: {name}")

print("--- Starting Brute Force ---")

for p in PASSWORDS:
    for c in CHALLENGES:
        # Permutation 1: Challenge + Password
        check(f"CRC({c} + {p})", v_crc(c, p))
        
        # Permutation 2: Password + Challenge
        check(f"CRC({p} + {c})", v_crc_inv(c, p))
        
        # Single CRC checks
        check(f"CRC({c})", zlib.crc32(c) & 0xFFFFFFFF)
        
        # Upper/Lower case variants of hex strings?
        
print("--- Done ---")
