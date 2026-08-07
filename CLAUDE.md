# Claude Code instructions

Mirrors this project's GEMINI.md (same rules, for Claude Code sessions).

## Project rules

- **Don't run flash/upload commands** (e.g. `teensy_loader_cli`, Arduino IDE upload, `pio run -t upload`) or open a serial monitor **on your own initiative.** A serial debugger/monitor is kept attached to the device's serial port, so an upload from here can contend for the port and hang or corrupt the session.
- Default: build only to verify changes, and hand it back to the user to flash and report what they see.
- If the user explicitly asks you to flash/upload/monitor in their message, go ahead — this isn't a hard block. (`.claude/settings.json` also gates these commands behind an approval prompt as a backstop.)
- If build output shows an error, fix it and retry once.
