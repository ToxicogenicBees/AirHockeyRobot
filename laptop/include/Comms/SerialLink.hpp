#pragma once

#include "Comms/PacketBuffer.hpp"
#include "Comms/Packet.hpp"
#include "Types/Timer.hpp"
#include "Constants.hpp"

#include <windows.h>
#include <iostream>
#include <functional>
#include <stdint.h>
#include <vector>
#include <mutex>
#include <unordered_map>

class SerialLink {
    private:
        using processor = std::function<void(Packet&)>;

        // Link handler
        inline static HANDLE _h_serial;

        // Incoming bytes buffer
        inline static std::vector<uint8_t> _rx_buffer;

        // Buffered completed packets
        inline static PacketBuffer _receive_buffer;
        inline static PacketBuffer _send_buffer;
        inline static std::mutex _send_guard;

        // Packet handlers
        inline static std::unordered_map<Action, processor> _handlers;

        // Communication timer
        inline static Timer _timer;

        // Receive a packet from the link
        static Packet _receivePacket() {
            // Get COM status
            DWORD errors = 0;
            COMSTAT comStat = {0};
            if (!ClearCommError(_h_serial, &errors, &comStat)) {
                std::cerr << "ClearCommError failed, error: " << GetLastError() << "\n";
                return {Action::Invalid};
            }

            // Store all incoming bytes
            DWORD bytes_available = comStat.cbInQue;
            if (bytes_available) {
                std::vector<uint8_t> buffer(bytes_available);
                DWORD bytes_read = 0;
                if (!ReadFile(_h_serial, buffer.data(), bytes_available, &bytes_read, nullptr)) {
                    std::cerr << "Serial read failed, error: " << GetLastError() << "\n";
                    return {Action::Invalid};
                }
                _rx_buffer.insert(_rx_buffer.end(), buffer.begin(), buffer.begin() + bytes_read);
            }

            // Check if incoming bytes are long enough to form a valid packet
            if (!_rx_buffer.size() || _rx_buffer.size() < _rx_buffer[0])
                return {Action::Invalid};

            // Fetch packet
            Packet packet(_rx_buffer);

            // Validate packet
            if (!packet.isValid()) {
                _rx_buffer.clear();
                return {Action::Invalid};
            }
            
            // Return packet
            _rx_buffer.erase(_rx_buffer.begin(), _rx_buffer.begin() + packet.length());
            return packet;
        }

    public:
        static void init() {
            // Initialize serial link
            _h_serial = CreateFile(Constants::Comms::COM_PORT,
                GENERIC_READ | GENERIC_WRITE,
                0,          // No Sharing
                nullptr,    // No Security
                OPEN_EXISTING,
                0,
                nullptr);
            if (_h_serial == INVALID_HANDLE_VALUE) {
                std::cerr << "Error code: " << GetLastError() << "\n";
                throw std::runtime_error("Failed to open COM port.");
            }

            DCB dcb;
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
            Sleep(50);
            PurgeComm(_h_serial, PURGE_RXCLEAR | PURGE_TXCLEAR); //purge both tx and rx buffer

            // Reset timer
            _timer.reset();
        }

        static void registerHandler(Action action, processor handler) {
            _handlers[action] = handler;
        }

        static void buffer(const Packet& packet) {
            {
                std::lock_guard<std::mutex> guard(_send_guard);
                _send_buffer.insert(packet);
            }
        }

        static void process(bool forceDumpBuffer = false) {
            // Fetch incoming packet
            bool receivedTermination = forceDumpBuffer;
            Packet packet;

            while (!receivedTermination && _timer.delta<std::chrono::seconds>() < Constants::Comms::TIMEOUT) {
                packet = _receivePacket();

                if (packet.action() != Action::Invalid) {
                    // Store in buffer
                    _receive_buffer.insert(packet);

                    // If this was the last packet
                    if (packet.action() == Action::Terminate) {
                        // Process all packets
                        for (auto& p : _receive_buffer) {
                            if (p) {
                                auto callback = _handlers.find(p->action());
                                if (callback != _handlers.end()) {
                                    p->resetRead();
                                    callback->second(*p);
                                }
                            }
                        }
                        _receive_buffer.clear();
                        receivedTermination = true;
                    }
                }
            }

            if (receivedTermination) {
                {
                    std::lock_guard<std::mutex> guard(_send_guard);

                    // Send buffered packets
                    for (auto& p : _send_buffer) {
                        if (p) {
                            p->finalize();

                            DWORD bytesWritten;
                            WriteFile(_h_serial, p->data(), p->length(), &bytesWritten, nullptr);
                        }
                    }
                    _send_buffer.clear();
                }
                
                // Send sentinal packet
                Packet terminate(Action::Terminate);
                terminate.finalize();
                DWORD bytesWritten;
                WriteFile(_h_serial, terminate.data(), terminate.length(), &bytesWritten, nullptr);

                // Reset timer
                _timer.reset();
            }

            else if (_timer.delta() >= Constants::Comms::TIMEOUT) {
                // Reset incoming buffer
                _rx_buffer.clear();

                // Send sentinal packet
                Packet terminate(Action::Terminate);
                terminate.finalize();
                DWORD bytesWritten;
                WriteFile(_h_serial, terminate.data(), terminate.length(), &bytesWritten, nullptr);

                // Reset timer
                _timer.reset();
            }
        }
};
