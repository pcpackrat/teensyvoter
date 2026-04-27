# AllStarLink Voter Network Members

This document outlines the current members of the simulcast voting network.

## 1. Selma
*   **Role:** Master Node & Primary Transmitter
*   **Hardware:** Original hardware PIC controller (RTCM/Voter 1)
*   **Notes:** Acts as the network's voting system master. It exhibits a normal, calculated time difference from itself in Asterisk logs due to how Asterisk anchors the dynamic jitter buffer to the earliest valid timestamp within a vote polling cycle.

## 2. Universal City
*   **Role:** Receiver Node
*   **Hardware:** Original hardware PIC controller (RTCM/Voter 1)
*   **Notes:** Standard voting receiver.

## 3. Garden Ridge
*   **Role:** Receiver Node
*   **Hardware:** Original hardware PIC controller (RTCM/Voter 1)
*   **Notes:** Standard voting receiver.

## 4. Selma_john
*   **Role:** Receiver Node
*   **Hardware:** Original hardware PIC controller (RTCM/Voter 1)
*   **Notes:** Standard voting receiver.

## 5. Teensy_test (TeensyVoter)
*   **Role:** Experimental Receiver Node
*   **Hardware:** Teensy 4.1 Microcontroller + SGTL5000 Audio Shield
*   **Notes:** Custom firmware implementation. Matches the original hardware characteristics by applying a standard -180ms Delay Offset to GPS timestamps to accurately sit within Asterisk's Jitter Buffer window alongside the legacy PIC nodes.
