#include "Comms/SerialLink.h"
#include "Constants.h"

#include <windows.h>
#include <iostream>

HANDLE SerialLink::hSerial = INVALID_HANDLE_VALUE;

void SerialLink::init() {
    hSerial = CreateFileA(Constants::COM_PORT, GENERIC_READ | GENERIC_WRITE, 0, nullptr, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, nullptr);
    if (hSerial == INVALID_HANDLE_VALUE) throw std::runtime_error("Failed to open COM port");

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hSerial, &dcb);
    dcb.BaudRate = CBR_9600;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    SetCommState(hSerial, &dcb);

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    SetCommTimeouts(hSerial, &timeouts);
}

void SerialLink::send(Packet& packet) {
    packet.finalize();
    DWORD bytesWritten;
    WriteFile(hSerial, packet._data.data(), packet.length(), &bytesWritten, nullptr);
}

Packet SerialLink::read() {
    BYTE lengthByte;
    DWORD bytesRead;

    // Read first byte
    if (!ReadFile(hSerial, &lengthByte, 1, &bytesRead, nullptr) || bytesRead != 1)
        throw std::runtime_error("Failed to read packet length");

    std::vector<uint8_t> buffer(lengthByte);
    buffer[0] = lengthByte;

    // Read remaining bytes
    DWORD remaining = lengthByte - 1;
    if (!ReadFile(hSerial, buffer.data() + 1, remaining, &bytesRead, nullptr) || bytesRead != remaining)
        throw std::runtime_error("Failed to read packet payload");

    Packet packet(buffer);
    if (!packet.isValid())
        throw std::runtime_error("Invalid packet CRC");

    return packet;
}