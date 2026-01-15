# Debugging Plan: Jitter & Audio Quality
**Date:** 2026-01-12
**Objective:** Eliminate audio dropouts ("jitter") and optimize gain staging now that the filtering is fixed.

## 1. The "Test Tone" Isolation Test
We need to determine if the "jitter" is occurring in the **Analog Domain** (ADC reading bad values) or the **Network Domain** (Packets being dropped).

### Action:
1.  Open `src/main.cpp`.
2.  Find `bool g_testToneMode = false;` (likely near top or in `loop`).
3.  Set it to `true`.
4.  Compile and Upload.

### Explanation: Bandwidth vs. Packet Loss
**"The Network is Fast Enough"** is true for bandwidth (streaming Netflix).
But for Real-Time UDP (VoIP), what matters is **Consistency**.
- **Bandwidth**: Width of the highway (Teensy WiFi is huge).
- **Packet Loss**: Large potholes that swallow cars.
The trace shows your highway has huge potholes. 60% of cars are falling in.

### 2. Monitor PPS Live (New Tool)
1.  Run `python tools/measure_pps.py` on the server.
    *   (Usage: `python tools/measure_pps.py 1667`)
2.  Watch the PPS (Packets Per Second) count.
    *   **Target**: 50 PPS (Steady)
    *   **Current**: ~20 FPS (Jerky)

### 3. Immediate Fixes to Try
1.  **Reduce SPI Clock Speed**: Open `src/EspSpiDriver.cpp`. Change `SPISettings(8000000, ...)` to `SPISettings(2000000, ...)`. (Slower clock = cleaner signal).
2.  **Move Antenna**: If using an external antenna on the ESP32, verify it's connected. If PCB trace, ensure it's not shielded by the Teensy.

### 4. "Scrolling Sending Auth Request"?
If the serial monitor just scrolls "Sending Auth Request..." forever:
*   **Meaning**: The Teensy is **disconnected** from the server.
*   **Cause**: The handshake packets (Auth Request / Challenge) are being lost.
*   **Action**:
    1.  **Check IP**: Did the server IP change?
    2.  **Check Port**: Is Asterisk running? (Stop it: `service asterisk stop`).
    3.  **Check Password**: Ensure `clientPwd` matches `voter.conf`.
    4.  **Revert SPI**: If 2MHz broke it, try 4MHz or go back to 8MHz and check cables.

---

## 2. Re-Verification of Packet Loss
If the Test Tone stutters, we must confirm the packet loss rate.

### Action:
1.  Run the PCAP analysis script on the server:
    ```bash
    tcpdump -i any -n -u -w trace_tomorrow.pcap port 1667
    # (Talk into radio for 10 seconds)
    # Stop capture usually Ctrl-C
    python3 analyze_pcap.py trace_tomorrow.pcap
    ```

### Success Criteria:
*   **Expected Packet Rate**: ~50 packets per second.
*   **10 Seconds**: Should see ~500 packets from the Teensy IP.
*   **Fail Condition**: If we see <400 packets, we have >20% packet loss. This **will** sound like jitter.

---

## 3. Gain Staging Protocol
Once Jitter is gone, we tune for quality.

1.  **Input Gain (`rxGain`)**:
    *   Controls the SGTL5000 hardware amplifier.
    *   **Goal**: Strong signal, peaks around -6dB to -3dB. Never clipping (flat tops).
    *   **Current**: Set to `2` (Low). Increase only if audio is too quiet.
2.  **Filter Gain**:
    *   Currently **Unity (1.0x)**.
    *   If audio is muffled/quiet after filtering, we can uncomment the 1.5x boost in `DSPProcessor.cpp` but **must** ensure we clamp values to avoid `int16_t` overflow.

## Summary Checklist
- [ ] Run **Test Tone** text.
- [ ] Run **Packet Loss** check.
- [ ] If Network issue: Check ESP32/SPI.
- [ ] If Analog issue: Check Gain/Grounding.

## 4. Hardware Consideration: Removing Audio Shield?
**Proposition**: The SGTL5000 is fixed at 44.1kHz, forcing us to downsample to 8kHz, which is computationally expensive and error-prone (artifacts). Removing it would allow using the Teensy's Native Analog Input at exactly 8kHz.

### Pros:
1.  **Perfect Sample Rate**: Native 8000Hz sampling. Zero aliasing from downsampling. Zero CPU load for decimation.
2.  **Simplicity**: Less code, fewer I2C config issues. Matches original VOTER hardware design.

### Cons:
1.  **Lower Quality Input**: Native ADC is likely noisier than the SGTL5000's dedicated codec.
2.  **No Headphone Out**: We lose the ability to plug in headphones for debugging (The "Test Tone" becomes invisible).
3.  **Circuit Change**: Requires building a DC-bias voltage divider (Teensy ADC is 0-3.3V, Audio is AC +/- 1V). SGTL handles this internally.

**Reference Note**: The Voter2 (STM32) uses its internal **12-bit ADC**. Switching TeensyVoter to its native internal ADC would arguably make the project **more aligned** with the reference hardware standard than using the high-fidelity 16-bit SGTL5000.

**Decision Point**: Only if **Step 3 Gain Staging** fails to fix the quality should we strip the hardware. The Jitter is likely network anyway.
