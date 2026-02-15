#ifndef WINUSBLINK_HPP
#define WINUSBLINK_HPP

#include "Comms/SerialLink.hpp"
#include "Constants.h"
#include <windows.h>
#include <hidsdi.h>  // need to link hid.lib library during the compilation process

class WinUSBLink : public SerialLink {
    private:
        static HANDLE _h_serial;

        static void _sendPacket(const Packet& packet) {
            DWORD bytesWritten;
            WriteFile(_h_serial, packet.data().data(), packet.length(), &bytesWritten, nullptr);
        }

        static Packet _receivePacket() {
            BYTE lengthByte;
            DWORD bytesRead;

            SetCommMask(_h_serial, EV_RXCHAR);  // Set mask to listen for data received (EV_RXCHAR)

            DWORD EventMask;
            
            WaitCommEvent(_h_serial, &EventMask, NULL);  // blocks until new data received

            std::vector<uint8_t> buffer;

            //Check received bitmask with EV_RXCHAR
            if (EventMask & EV_RXCHAR) {
                // Read first byte
                if (!ReadFile(_h_serial, &lengthByte, 1, &bytesRead, nullptr) || bytesRead != 1)
                    std::cout << "Failed to read packet length, error: " << GetLastError() << "\n";

                if (lengthByte == 0) {
                    std::cout << "Buffer Resizing Error Lengthbyte = 0\n";
                } else {
                    buffer.resize(lengthByte);
                    buffer[0] = lengthByte;

                    // Read remaining bytes
                    DWORD remaining = lengthByte - 1;
                    if (!ReadFile(_h_serial, buffer.data() + 1, remaining, &bytesRead, nullptr) || bytesRead != remaining)
                        std::cout << "Failed to read packet payload, error: " << GetLastError() << "\n";

                    Packet packet(buffer);
                    if (!packet.isValid())
                        std::cout << "Invalid packet CRC\n";

                    return packet;
                }
            }

            // if incorrect packet format recieved, return empty packet and purge input buffer 
            // Also check for if the USB device has been unplugged

            // try to reopen serial port, if disconnected this will fail:
            _h_serial = CreateFile(Constants::Comms::COM_PORT,
                GENERIC_READ | GENERIC_WRITE,
                0,          // No Sharing
                nullptr,    // No Security
                OPEN_EXISTING,
                0,
                nullptr);
            if (_h_serial == INVALID_HANDLE_VALUE) {
                std::clog << "Error code: " << GetLastError() << "\n";
                std::clog << "Failed to open COM port.\n";
            }

            DCB dcb = {0};
            dcb.DCBlength = sizeof(dcb);
            GetCommState(_h_serial, &dcb);
            dcb.BaudRate = Constants::Comms::BAUD_RATE;
            dcb.ByteSize = 8;
            dcb.Parity = NOPARITY;
            dcb.StopBits = ONESTOPBIT;
            SetCommState(_h_serial, &dcb);

            COMMTIMEOUTS timeouts = {0};
            timeouts.ReadIntervalTimeout = 20;
            timeouts.ReadTotalTimeoutMultiplier = 1;
            timeouts.ReadTotalTimeoutConstant = 50;
            timeouts.WriteTotalTimeoutMultiplier = 1; // Per-byte timeout
            timeouts.WriteTotalTimeoutConstant = 50;  // Constant timeout (ms)
            SetCommTimeouts(_h_serial, &timeouts);

            PurgeComm(_h_serial, PURGE_RXCLEAR); //purge rx buffer
            return Packet(Action::INVALID);
        }

        static void _initLink() {
            _h_serial = CreateFile(Constants::Comms::COM_PORT,
                GENERIC_READ | GENERIC_WRITE,
                0,          // No Sharing
                nullptr,    // No Security
                OPEN_EXISTING,
                0,
                nullptr);
            if (_h_serial == INVALID_HANDLE_VALUE) {
                std::cout << "Error code: " << GetLastError() << "\n";
                throw std::runtime_error("Failed to open COM port.");
            }

            DCB dcb = {0};
            dcb.DCBlength = sizeof(dcb);
            GetCommState(_h_serial, &dcb);
            dcb.BaudRate = Constants::Comms::BAUD_RATE;
            dcb.ByteSize = 8;
            dcb.Parity = NOPARITY;
            dcb.StopBits = ONESTOPBIT;
            SetCommState(_h_serial, &dcb);

            COMMTIMEOUTS timeouts = {0};
            timeouts.ReadIntervalTimeout = 20;
            timeouts.ReadTotalTimeoutMultiplier = 1;
            timeouts.ReadTotalTimeoutConstant = 50;
            timeouts.WriteTotalTimeoutMultiplier = 1; // Per-byte timeout
            timeouts.WriteTotalTimeoutConstant = 50;  // Constant timeout (ms)
            SetCommTimeouts(_h_serial, &timeouts);

            // Clear both the TX and RX buffer
            PurgeComm(_h_serial, PURGE_RXCLEAR | PURGE_TXCLEAR); //purge both tx and rx buffer
        }
};


#endif
