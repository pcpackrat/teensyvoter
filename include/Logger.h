#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>
#include "NetworkManager.h"
#include "ConfigManager.h"

#define LOG_BUFFER_SIZE 32
#define LOG_MESSAGE_MAX_LEN 128

enum LogLevel {
    LOG_INFO = 6,     // Informational
    LOG_NOTICE = 5,   // Normal but significant
    LOG_WARNING = 4,  // Warning conditions
    LOG_ERR = 3       // Error conditions
};

struct LogEntry {
    LogLevel level;
    char tag[16];
    char message[LOG_MESSAGE_MAX_LEN];
    bool pending;
};

class Logger {
public:
    Logger();
    void begin(NetworkManager* net, ConfigManager* cfg);
    
    // Log a message (adds to buffer)
    void log(LogLevel level, const char* tag, const char* fmt, ...);
    
    // Helper shortcuts
    void info(const char* tag, const char* fmt, ...);
    void warn(const char* tag, const char* fmt, ...);
    void error(const char* tag, const char* fmt, ...);

    // Process the queue (call in main loop)
    void update();

private:
    NetworkManager* _net;
    ConfigManager* _cfg;
    
    LogEntry _buffer[LOG_BUFFER_SIZE];
    int _head; // Where to write next
    int _tail; // Where to read next for sending
    
    IPAddress _syslogResolvedIP;
    uint32_t _lastResolutionTime;
    
    void _sendSyslog(const LogEntry& entry, IPAddress targetIP);
    int _available();
};

extern Logger logger;

#endif
