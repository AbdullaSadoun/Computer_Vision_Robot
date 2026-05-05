/*
 * File:   serial.h
 * Author: Terraneo Federico
 * Distributed under the Boost Software License, Version 1.0.
 *
 * Created on September 10, 2009, 10:21 AM
 */

#ifndef _SERIAL_H
#define _SERIAL_H

#include <windows.h>
#include <string>
#include <stdexcept>

class Serial
{
public:
    Serial(const std::string &port, unsigned int baud_rate = 9600)
    {
        hSerial = CreateFileA(port.c_str(),
                GENERIC_READ | GENERIC_WRITE,
                0,
                NULL,
                OPEN_EXISTING,
                FILE_ATTRIBUTE_NORMAL,
                NULL);
        if(hSerial==INVALID_HANDLE_VALUE){
            if(GetLastError()==ERROR_FILE_NOT_FOUND){
                throw std::runtime_error("Serial port not found");
            }
            throw std::runtime_error("Failed to open serial port");
        }

        DCB dcbSerialParams = {0};
        dcbSerialParams.DCBlength=sizeof(dcbSerialParams);
        if (!GetCommState(hSerial, &dcbSerialParams)) {
            CloseHandle(hSerial);
            throw std::runtime_error("Failed to get serial port state");
        }
        dcbSerialParams.BaudRate=baud_rate;
        dcbSerialParams.ByteSize=8;
        dcbSerialParams.StopBits=ONESTOPBIT;
        dcbSerialParams.Parity=NOPARITY;
        if(!SetCommState(hSerial, &dcbSerialParams)){
            CloseHandle(hSerial);
            throw std::runtime_error("Failed to set serial port state");
        }

        COMMTIMEOUTS timeouts={0};
        timeouts.ReadIntervalTimeout=50;
        timeouts.ReadTotalTimeoutConstant=50;
        timeouts.ReadTotalTimeoutMultiplier=10;
        timeouts.WriteTotalTimeoutConstant=50;
        timeouts.WriteTotalTimeoutMultiplier=10;
        if(!SetCommTimeouts(hSerial, &timeouts)){
            CloseHandle(hSerial);
            throw std::runtime_error("Failed to set serial port timeouts");
        }
    }

    ~Serial()
    {
        if (hSerial != INVALID_HANDLE_VALUE) {
            CloseHandle(hSerial);
        }
    }

    int write(const char *buffer, unsigned int nbChar)
    {
        DWORD bytesSend;
        if(!WriteFile(hSerial, (void *)buffer, nbChar, &bytesSend, 0)){
            ClearCommError(hSerial, &errors, &status);
            return -1;
        }
        else return bytesSend;
    }

    bool is_open() const {
        return hSerial != INVALID_HANDLE_VALUE;
    }

private:
    HANDLE hSerial;
    COMSTAT status;
    DWORD errors;
};

#endif /* _SERIAL_H */
