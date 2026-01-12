Power-On & First-Boot Checklist for TeensyVoter

1. Physical wiring review
   - Verify Teensy 4.1 is seated and Audio Shield Rev D is plugged on top.
   - Confirm GPS GT-UT connections: TX->Teensy RX1(pin0), PPS->D2, VCC->3.3V, GND->GND.
   - Confirm ESP32 WROOM connections: MISO->12, MOSI->11, SCK->13, CS->10, READY->24, RESET->25, VCC->3.3V regulator, GND->GND.
   - RSSI divider output to `A14` and oscilloscope probe to `D3` (optional).

2. Power supplies
   - Use a dedicated 3.3V regulator for ESP32 capable of >=800mA. Place 0.1µF ceramic + 10–100µF bulk caps close to ESP module power pins.
   - Ensure Teensy ground and module grounds are common.

3. Pre-boot checks
   - Confirm USB cable is data-capable and firmly connected.
   - If powering ESP32 externally, ensure regulator is off before initial connection if you want to avoid spikes.

4. Upload firmware (already done)
   - If you need to re-upload: hold Teensy PROGRAM button while running:

```powershell
cd 'c:\Users\mikec\Documents\Projects\VOTER\TeensyVoter'
python -m platformio run -e teensy41 -t upload
```

5. Open serial monitor
   - Open PlatformIO monitor on the Teensy's COM port at 115200:

```powershell
python -m platformio device monitor -p COM9 -b 115200
```

6. Verify boot logs
   - Look for: "TeensyVoter Booting...", audio init message, "[GPS] Initializing GPS...", network init messages, and periodic sensor/status logs.
   - Confirm GPS messages / PPS detection within a minute (GPS may take time to get fix).

7. Basic runtime checks
   - Verify audio: if using speakers/headphones through Audio Shield, test audio path or check that `Audio` subsystem initialized.
   - Verify network: check `netMgr` reports connected or see logs showing UDP target set to configured host IP/port.
   - If using GPS: check for fix and time sync messages.

8. Troubleshooting
   - No serial output: ensure correct COM port and USB cable; try another port/cable.
   - ESP32 not connecting: check regulator voltage/current, ensure READY pin wiring and RESET wiring match module.
   - GPS no fix: move outdoors or ensure GPS antenna connected and powered.
   - Audio problems: confirm Audio Shield seated, volume set in sketch, and speakers/headphones wired correctly.

9. Safety
   - Do not feed >3.3V signals into Teensy GPIO pins.
   - Double-check regulator connections before powering heavy modules.

If you want, I can: (A) export these three docs as a ZIP, (B) generate a PNG of the SVG, or (C) produce a printable PDF checklist. Which would you like next?