#include "NetworkManager.h"

NetworkManager::NetworkManager() {
    _driver = nullptr;
}

void NetworkManager::begin(NetworkDriver* driver, uint8_t* mac_addr) {
    _driver = driver;
    _mac = mac_addr;
    
    if (_driver) {
        _driver->begin(_mac);
    }
}

void NetworkManager::update() {
    if (_driver) _driver->update();
}

void NetworkManager::setTarget(IPAddress ip, uint16_t port) {
    if (_driver) _driver->setTarget(ip, port);
}

void NetworkManager::sendPacket(const uint8_t* data, uint16_t length) {
    if (_driver) _driver->sendPacket(data, length);
}

int NetworkManager::parsePacket() {
    if (_driver) return _driver->parsePacket();
    return 0;
}

int NetworkManager::read(uint8_t* buffer, size_t maxLen) {
    if (_driver) return _driver->read(buffer, maxLen);
    return 0;
}

bool NetworkManager::isConnected() {
    if (_driver) return _driver->isConnected();
    return false;
}

IPAddress NetworkManager::getLocalIP() {
    if (_driver) return _driver->getLocalIP();
    return IPAddress(0,0,0,0);
}
