# Reverse-Tunnel Remote Access for TeensyVoter

## Context

Units get deployed at remote repeater sites where editing NAT/port-forwarding isn't practical. The requirement is remote reach to (a) the existing web config UI and (b) outbound log messages, for a fleet of units, without any site-side network config. Solution: the Teensy dials **out** persistently to a relay server the operator controls; the relay is a rendezvous/proxy point, addressed by device ID so multiple units can share one relay. SSH/telnet were ruled out earlier as unnecessary/unwise — scope is web UI + logs only.

Decisions already made with the user:
- **Fleet-scale**, not a single 1:1 pipe — relay needs a device-ID registry.
- **Relay-terminated trust model** (recommended default, not firmly locked in): the Teensy↔relay hop is authenticated+encrypted (PSK-based, forward-secret via ephemeral X25519), but the relay decrypts to route by device ID and proxy HTTP — it is not a blind pipe. This buys "any browser, pick a device from a dashboard" UX with no separate operator-side decrypt tool. Tradeoff: relay holds plaintext and could impersonate a device to an operator if compromised. If the user later wants true end-to-end blinding, the relay would need to stop holding the PSK and become a dumb byte-forwarder instead — noted here as a reversible decision, not a constraint baked into the wire format.
- **Relay is in scope** to build now, as a new component in this repo (`relay-server/`), not just specified.

No existing crypto/TLS is linked into the Teensy build (`platformio.ini` teensy41 env: only `NativeEthernet` + `TinyGPSPlus`) — this is new from scratch. The ESP32 co-processor path (`EspSpiDriver`, `esp32_firmware/Spirit/`) is being removed per the user's stated preference to keep everything on the Teensy; the tunnel does not depend on it, and cleaning it out first avoids tangling new plumbing with dead code in the same `main.cpp` loop region.

## Wire Protocol (Teensy ↔ Relay)

- **Transport**: one persistent outbound TCP connection, `EthernetClient.connect(relayHost, relayPort)` (`NativeEthernet`, standard Arduino `Client` API).
- **Frame**: `uint16 length | ciphertext (+16B Poly1305 tag)`. Decrypted plaintext's first byte is a channel ID: `0x00` HTTP proxy, `0x01` log, `0x02` ping/pong. Max plaintext payload 1024B (bounds per-frame decrypt/dispatch time, avoids segment-fragmentation complexity).
- **Handshake** (Monocypher primitives, one round trip):
  1. `ClientHello`: `deviceID(16B) | ephemeral_X25519_pub(32B) | timestamp(4B) | Blake2b-MAC(PSK, deviceID‖eph_pub‖timestamp)[:16]`
  2. Relay looks up PSK by `deviceID`, verifies MAC + timestamp window (±5min; Teensy has GPS time via `GPSManager`).
  3. `ServerHello`: relay ephemeral X25519 pub(32B) + `Blake2b-MAC(PSK, relay_eph_pub‖deviceID)[:16]` (mutual auth).
  4. `shared = X25519(eph_priv, peer_eph_pub)`; `session_root = Blake2b512(shared ‖ PSK)`; split into directional 32B subkeys via keyed Blake2b with context strings `"d2r"`/`"r2d"`.
  5. AEAD: XChaCha20-Poly1305, 24B nonce = 16 zero bytes + 8B big-endian monotonic counter per direction. Reconnect (fresh handshake) well before counter could realistically wrap.
- **Reconnect/backoff**: exponential 2s→60s cap with jitter. Ping every 30s idle; 90s no-pong forces reconnect (catches sockets silently dead through the site's NAT).

## Teensy-Side Firmware Changes

**New module**: `include/TunnelClient.h` / `src/TunnelClient.cpp` (matches existing `include/X.h`+`src/X.cpp` convention).
- `class TunnelManager`: `begin(NetworkManager*, ConfigManager*)`, `update()` (non-blocking, bounded — see contract below), `isConnected()`, `sendLog(const LogEntry&)`, `Client& getHttpClient()`.
- Internal `TunnelHttpClient : public Client` — implements `available()/read()/peek()/write()/connected()/stop()` against a ring buffer fed by decrypted channel-0 frames. This is the whole integration point: `WebServer` gets handed a `TunnelHttpClient&` and never knows a tunnel exists.
- State machine: `DISCONNECTED → TCP_CONNECTING → HANDSHAKING → CONNECTED → ERROR_BACKOFF`.
- **Bounded-time contract** (loop only has ~174ms audio slack, per `AudioMemory(60)`): never call blocking `connect()` inline — poll connection status across iterations while `TCP_CONNECTING`; process at most 1 inbound frame per `update()` call; never call `delay()`. Document this in the header.

**`TeensyWebServer` refactor** — mechanical, confirmed by reading the full file: every method in `include/TeensyWebServer.h` / `src/TeensyWebServer.cpp` (`handleRequest`, `readRequestLine`, all `handleX` route handlers, `sendHeader`/`sendHtmlHeader`/`sendHtmlFooter`/`sendRedirect`/`sendJsonResponse`, `handleXPost`, `handleExport`/`Import`/`ImportPost`, `readRequestBody` — ~15 total) takes `EthernetClient &client` but only ever calls generic `Client`/`Stream`/`Print` methods on it. Change every signature `EthernetClient &client` → `Client &client`. Zero logic changes. `_server` (`EthernetServer`) and the existing no-arg `handleClient()` stay as-is — implicit `EthernetClient`→`Client&` upcast keeps LAN-local access working identically. Add `void handleTunnelClient(Client &client);` calling `handleRequest(client)` when the tunnel's HTTP client has buffered bytes.

**`Logger` second sink**: add `void Logger::setTunnel(TunnelManager*)` + `TunnelManager* _tunnel` member. In `Logger::update()` (`src/Logger.cpp:82`), alongside the existing `_sendSyslog` loop, if `_tunnel && _tunnel->isConnected()`, call `_tunnel->sendLog(entry)`. Independent, best-effort sinks. Do **not** reuse the existing `delay(50)`-between-sends pattern for the tunnel path (`Logger.cpp:102`) — queue/drain at most one tunnel log frame per `update()`.

**`SysConfig` additions** (`include/ConfigManager.h:20-64`, append at end of the string block, bump `CONFIG_VERSION` 24→25 — same "whole struct resets to defaults" behavior as every prior bump, e.g. v23→v24 for `syslogHostname`):
```cpp
char     relayHost[64];   // hostname/IP, resolved like syslogHostname
uint16_t relayPort;
char     deviceID[17];    // 16 hex chars + NUL
uint8_t  devicePSK[32];   // Monocypher key material
bool     useTunnel;
```
Do this step **last** (see Staging), once the field set is proven stable, to avoid a second config-wiping version bump mid-protocol-iteration.

**Plumbing touchpoints** (mirror the existing `useSyslog`/`syslogHostname`/`syslogIP`/`syslogPort` pattern exactly, same files that pattern already touches):
- `src/TeensyWebServer.cpp`: new form fields + POST handling in `handleNetwork()`/`handleNetworkPost()` (relay host, port, device ID, PSK-as-hex, enable checkbox).
- `src/main.cpp` `printMenu()` / `handleSerialCLI()`: new menu entries mirroring the syslog block; PSK entry must not echo to serial (same care as `clientPwd`/`hostPwd`).
- `exportSettings()` / `parseConfigLine()` / `importSettings()` (`src/main.cpp`): add `relayHost=`/`relayPort=`/`deviceID=`/`useTunnel=` lines; PSK exported as hex, consistent with the existing plaintext-secret export precedent for `clientPwd`/`hostPwd`.

**Loop insertion** (`src/main.cpp`): add `tunnelMgr.update();` right after `netMgr.update();` (currently line 1553), in the slot vacated by removing the ESP32 `CFG_CMD_*`/`PARAM_*` block (currently lines 1556-1677). Add `if (networkReady && cfg.data.useTunnel) webServer.handleTunnelClient(tunnelMgr.getHttpClient());` right after the existing `webServer.handleClient();` (currently line 1700).

**`platformio.ini`**: vendor Monocypher (public-domain, 2 files, zero deps not even libc) into `lib/monocypher/monocypher.c` + `monocypher.h` — PlatformIO auto-discovers `lib/<name>/`, no `lib_deps` entry or build-time network fetch needed. Rejected mbedTLS as too heavy (brings unneeded X.509).

## Relay-Side Service (new: `relay-server/`)

**Language: Go** — single static binary (simple deploy on a VPS), goroutine-per-connection fits N persistent device sockets + M browser connections naturally, stdlib covers TLS/HTTP hijacking. Crypto via `golang.org/x/crypto/chacha20poly1305` (XChaCha), `crypto/ecdh` (X25519), `golang.org/x/crypto/blake2b` — vetted libraries kept interop-compatible with Monocypher's primitives on the device side, not hand-rolled twice.

**Structure**:
- `main.go` — wiring/startup.
- `device.go` — deviceID→PSK registry, loaded from `devices.json` (restart-required reload for v1).
- `tunnel.go` — device-facing TCP listener, handshake, per-device goroutine, `map[deviceID]*DeviceConn`.
- `proxy.go` — operator HTTP request → `http.Hijacker`-based raw byte pump to/from the device's channel-0 stream (hijack avoids double-parsing since firmware already speaks raw HTTP/1.1).
- `dashboard.go` — Go `embed`-bundled device list + log viewer page, no separate frontend build.
- `logstream.go` — per-device WebSocket (`gorilla/websocket`) pub/sub for channel-1 frames, `/ws/logs/{deviceID}`.
- `PROTOCOL.md` — single source of truth for the wire format; `TunnelClient.h` should reference it in a comment to prevent drift.
- `cmd/tunneltest/` — standalone Go CLI harness with a hardcoded PSK, used in Staging step 3 to validate handshake against a live board without needing the full dashboard built yet.

**Operator dashboard auth** — open question, not decided: v1 recommendation is HTTP Basic Auth over HTTPS with a single shared bcrypt-hashed operator password (adequate for single-operator use). If multi-operator/trust-tiers are ever needed, revisit with mTLS client certs, session-cookie login, or source-IP allowlisting.

**TLS** for the operator-facing HTTPS surface: `autocert` (Let's Encrypt) — a deployment-time concern (needs real DNS pointed at the relay), not code-blocking for this plan.

## Staging

0. Remove ESP32 path (`EspSpiDriver.h/.cpp`, `esp32_firmware/`, the `CFG_CMD_*`/`PARAM_*` block in `main.cpp`) — verify `pio run -e teensy41` still builds.
1. `Client&` refactor only, no tunnel code — verify build + physical-hardware regression (LAN-local UI unchanged).
2. Bare TCP framing, no crypto — `TunnelManager` talks to a throwaway listener; verify length-prefixed frame round-trip manually. Isolates framing/reconnect bugs from crypto bugs.
3. Add Monocypher handshake+AEAD on top of stage 2, verified against `relay-server/cmd/tunneltest/` with a hardcoded PSK — highest-risk stage (crypto interop).
4. Channel 1 (logs) end-to-end — relay forwards to stdout before any dashboard exists.
5. Channel 0 (HTTP proxy) end-to-end — relay's hijack-pump to a device, verified via direct `curl` against the relay's proxy endpoint (no dashboard yet), confirm identical HTML to LAN-local access.
6. Dashboard (device list, click-through, WS log viewer, auth) — pure Go, fastest-iterating stage, no firmware involvement.
7. `SysConfig` fields + all plumbing + `CONFIG_VERSION` bump — deliberately last.
8. Reconnect/backoff hardening + loop-timing verification — needs physical hardware with real audio signal exercised.

## Verification

**Verifiable in-session (build-only, no remote server touched)**: `pio run -e teensy41` after each firmware-touching stage (signature mismatches, struct layout, confirms Monocypher compiles for `arm-none-eabi`); `go build ./relay-server/...` + `go vet` for relay code.

**Not verifiable in-session**: real handshake interop between on-device Monocypher and the Go relay's crypto; actual relay deployment (DNS/TLS/outbound WAN dial); audio-timing regression under tunnel load. All need a physical Teensy 4.1 and a deployed relay.

**Out-of-session procedure for the user**:
1. `pio run -e teensy41 -t upload`.
2. Deploy `relay-server` (`go build && ./relay-server` with matching `devices.json`).
3. Confirm device shows connected within the backoff window of boot.
4. Browser → dashboard → device → confirm config UI matches LAN-local rendering; POST a harmless setting change, verify persistence.
5. Trigger a log event, confirm live-viewer delivery within seconds.
6. Unplug/replug Ethernet ~30s, confirm reconnect within backoff window.
7. During active RX/TX, watch the existing jitter-buffer debug prints while the tunnel proxies a request/streams logs — confirms no audio-timing regression.

## Critical Files
- `include/TeensyWebServer.h` / `src/TeensyWebServer.cpp` — `Client&` refactor, tunnel handoff.
- `include/Logger.h` / `src/Logger.cpp` — second log sink.
- `include/ConfigManager.h` — new `SysConfig` fields, `CONFIG_VERSION` bump.
- `src/main.cpp` — loop insertion (~line 1553, 1700), CLI menu, export/import, ESP32 removal.
- `include/NetworkDriver.h` / `include/NetworkManager.h` / `include/EthernetDriver.h` — unchanged, but confirms tunnel bypasses this UDP-only abstraction by design (uses `EthernetClient` TCP directly).
- New: `include/TunnelClient.h`, `src/TunnelClient.cpp`, `lib/monocypher/`, `relay-server/`.
