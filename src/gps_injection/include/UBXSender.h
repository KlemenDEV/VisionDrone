#pragma once

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>

#include <stdio.h>
#include <string.h>

#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()

#include <chrono>
#include <thread>

#include "ubx.h"
#include "GPSData.h"

using namespace std;

class UBXSender {

private:
    int serial_port;

public:
    UBXSender();
    void sendData(GPSData *data);

    void close();

};
