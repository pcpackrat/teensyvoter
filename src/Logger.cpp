#include "Logger.h"
#include <stdarg.h>

Logger logger;

Logger::Logger() {
    _net = nullptr;
    _cfg = nullptr;
    _head = 0;
    _tail = 0;
    for(int i=0; i<LOG_BUFFER_SIZE; i++) {
        _buffer[i].pending = false;
    }
    _syslogResolvedIP = IPAddress(0, 0, 0, 0);
    _lastResolutionTime = 0;
}

void Logger::begin(NetworkManager* net, ConfigManager* cfg) {
    _net = net;
    _cfg = cfg;
}

void Logger::log(LogLevel level, const char* tag, const char* fmt, ...) {
    // Format the message
    char buf[LOG_MESSAGE_MAX_LEN];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    // Add to circular buffer
    _buffer[_head].level = level;
    strncpy(_buffer[_head].tag, tag, sizeof(_buffer[_head].tag)-1);
    _buffer[_head].tag[sizeof(_buffer[_head].tag)-1] = '\0';
    strncpy(_buffer[_head].message, buf, sizeof(_buffer[_head].message)-1);
    _buffer[_head].message[sizeof(_buffer[_head].message)-1] = '\0';
    _buffer[_head].pending = true;

    // Advance head
    int nextHead = (_head + 1) % LOG_BUFFER_SIZE;
    if (nextHead == _tail) {
        // Buffer overflow - advance tail to drop oldest
        _tail = (_tail + 1) % LOG_BUFFER_SIZE;
    }
    _head = nextHead;

    // Also print to Serial for local debugging
    const char* lvlStr = "INFO";
    if (level == LOG_WARNING) lvlStr = "WARN";
    if (level == LOG_ERR) lvlStr = "ERROR";
    Serial.printf("[LOG][%s][%s] %s\r\n", lvlStr, tag, buf);
}

void Logger::info(const char* tag, const char* fmt, ...) {
    char buf[LOG_MESSAGE_MAX_LEN];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    log(LOG_INFO, tag, "%s", buf);
}

// Redefine helpers to handle variadic correctly without recursion issues
void Logger::warn(const char* tag, const char* fmt, ...) {
    char buf[LOG_MESSAGE_MAX_LEN];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    log(LOG_WARNING, tag, "%s", buf);
}

void Logger::error(const char* tag, const char* fmt, ...) {
    char buf[LOG_MESSAGE_MAX_LEN];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    log(LOG_ERR, tag, "%s", buf);
}

void Logger::update() {
    if (!_net || !_cfg) return;
    
    // Use a union to safely convert the raw 32-bit IP from EEPROM to an IPAddress object
    union {
        uint32_t raw;
        uint8_t bytes[4];
    } ipConv;
    ipConv.raw = _cfg->data.syslogIP;
    IPAddress targetIP(ipConv.bytes[0], ipConv.bytes[1], ipConv.bytes[2], ipConv.bytes[3]);
    
    if (targetIP == IPAddress(0, 0, 0, 0)) return;

    // Send at most one pending log per update() call (Phase 0 SPI reliability
    // fix). This used to drain the entire queue in one call with a delay(50)
    // between each entry - a burst of 3+ queued lines could block the main
    // loop, and therefore the time-critical audio send path, for 100ms+ in
    // a single call. Bounding to one entry keeps this call's worst case to
    // a single SPI transaction; a backlog just drains one entry per loop
    // iteration instead of all at once.
    if (_tail != _head) {
        if (_buffer[_tail].pending) {
            _sendSyslog(_buffer[_tail], targetIP);
            _buffer[_tail].pending = false;
        }
        _tail = (_tail + 1) % LOG_BUFFER_SIZE;
    }
}

void Logger::_sendSyslog(const LogEntry& entry, IPAddress targetIP) {
    if (!_net || !_cfg) return;

    int priority = (1 * 8) + (int)entry.level;
    char packet[256];
    snprintf(packet, sizeof(packet), "<%d>%s: %s", priority, entry.tag, entry.message);

    // CRITICAL DEBUG: Is this actually being called?
    Serial.printf("[Logger] Dispatching UDP to %d.%d.%d.%d:%d (%s)\r\n", 
                  targetIP[0], targetIP[1], targetIP[2], targetIP[3], 
                  _cfg->data.syslogPort, entry.tag);

    _net->sendPacketTo(targetIP, _cfg->data.syslogPort, (uint8_t*)packet, strlen(packet));
}
