#include "Comms/SerialLink.h"
#include "Constants.h"

#include <windows.h>
#include <iostream>

HANDLE SerialLink::hSerial = INVALID_HANDLE_VALUE;

void SerialLink::init() {
    hSerial = CreateFile(Constants::Comms::COM_PORT,
        GENERIC_READ | GENERIC_WRITE,
        0,          // No Sharing
        nullptr,    // No Security
        OPEN_EXISTING,
        0,
        nullptr);
    if (hSerial == INVALID_HANDLE_VALUE) {
        std::cout << "Error code: " << GetLastError() << "\n";
        throw std::runtime_error("Failed to open COM port.");
    }

    DCB dcb = {0};
    dcb.DCBlength = sizeof(dcb);
    GetCommState(hSerial, &dcb);
    dcb.BaudRate = Constants::Comms::BAUD_RATE;
    dcb.ByteSize = 8;
    dcb.Parity = NOPARITY;
    dcb.StopBits = ONESTOPBIT;
    SetCommState(hSerial, &dcb);

    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 20;
    timeouts.ReadTotalTimeoutMultiplier = 1;
    timeouts.ReadTotalTimeoutConstant = 50;
    timeouts.WriteTotalTimeoutMultiplier = 1; // Per-byte timeout
    timeouts.WriteTotalTimeoutConstant = 50;  // Constant timeout (ms)
    SetCommTimeouts(hSerial, &timeouts);

    //Clear both the TX and RX buffer
    PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR); //purge both tx and rx buffer
}

void SerialLink::send(Packet& packet) {
    packet.finalize();
    DWORD bytesWritten;
    WriteFile(hSerial, packet._data.data(), packet.length(), &bytesWritten, nullptr);
}

// run this function once in a new thread
// will loop forever, waiting for comm events to read
// pass in a mutex controlled vector of packets
void SerialLink::readWaitCommLoop(std::vector<Packet>*) {
    
}

Packet SerialLink::read() {
    BYTE lengthByte;
    DWORD bytesRead;

    //BOOL status;
    DWORD bytesRead;
    uint8_t receive_data_buffer[128] = { 0 }; // intialize the buffer 

    SetCommMask(hSerial, EV_RXCHAR);  // Set mask to listen for data received (EV_RXCHAR)

    DWORD EventMask;
    
    WaitCommEvent(hSerial, &EventMask, NULL);  // blocks until new data received

    std::vector<uint8_t> buffer;

    //Check received bitmask with EV_RXCHAR
    if (EventMask & EV_RXCHAR) {
        // Read first byte
        if (!ReadFile(hSerial, &lengthByte, 1, &bytesRead, nullptr) || bytesRead != 1)
            std::cout << "Failed to read packet length, error: " << GetLastError() << "\n";
        
        buffer.resize(lengthByte);
        buffer[0] = lengthByte;

        // Read remaining bytes
        DWORD remaining = lengthByte - 1;
        if (!ReadFile(hSerial, buffer.data() + 1, remaining, &bytesRead, nullptr) || bytesRead != remaining)
            std::cout << "Failed to read packet payload, error: " << GetLastError() << "\n";
    }

    Packet packet(buffer);
    if (!packet.isValid())
        std::cout << "Invalid packet CRC\n";

    return packet;
}