#include "NetworkManager.h"

NetworkManager::NetworkManager() { _driver = nullptr; }

void NetworkManager::begin(NetworkDriver *driver, uint8_t *mac_addr) {
  _driver = driver;
  _mac = mac_addr;

  // Note: Driver is already initialized by caller (main.cpp)
  // Don't call _driver->begin() again here
}

void NetworkManager::update() {
  if (_driver)
    _driver->update();
}

void NetworkManager::setTarget(IPAddress ip, uint16_t port) {
  if (_driver)
    _driver->setTarget(ip, port);
}

void NetworkManager::sendPacket(const uint8_t *data, uint16_t length) {
  if (_driver)
    _driver->sendPacket(data, length);
}

void NetworkManager::sendPacketTo(IPAddress ip, uint16_t port,
                                 const uint8_t *data, uint16_t length) {
  if (_driver)
    _driver->sendPacketTo(ip, port, data, length);
}

int NetworkManager::parsePacket() {
  if (_driver)
    return _driver->parsePacket();
  return 0;
}

int NetworkManager::read(uint8_t *buffer, size_t maxLen) {
  if (_driver)
    return _driver->read(buffer, maxLen);
  return 0;
}

IPAddress NetworkManager::resolveHostname(const char *hostname) {
  if (_driver)
    return _driver->resolveHostname(hostname);
  return IPAddress(0, 0, 0, 0);
}

bool NetworkManager::isConnected() {
  if (_driver)
    return _driver->isConnected();
  return false;
}

IPAddress NetworkManager::getLocalIP() {
  if (_driver)
    return _driver->getLocalIP();
  return IPAddress(0, 0, 0, 0);
}

IPAddress NetworkManager::getSubnetMask() {
  if (_driver)
    return _driver->getSubnetMask();
  return IPAddress(0, 0, 0, 0);
}

IPAddress NetworkManager::getGateway() {
  if (_driver)
    return _driver->getGateway();
  return IPAddress(0, 0, 0, 0);
}

IPAddress NetworkManager::getDNS() {
  if (_driver)
    return _driver->getDNS();
  return IPAddress(0, 0, 0, 0);
}

DriverType NetworkManager::getType() {
  if (_driver)
    return _driver->getType();
  return DRIVER_NONE;
}
