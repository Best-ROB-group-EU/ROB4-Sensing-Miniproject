#include <stdio.h>
#include <stdlib.h>

#include "rplidar.h"
#include <unistd.h>

#include <signal.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

using namespace rp::standalone::rplidar;

bool ctrl_c_pressed;
void ctrlc(int)
{
    ctrl_c_pressed = true;
}

static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

int main()
{
    _u32 baudrate = 115200;
    const char * usb_path = "/dev/ttyUSB0";
    u_result response;

    // Variable to hold device info response (???)
    rplidar_response_device_info_t device_info;

    // Instantiate driver
    RPlidarDriver * drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

    // Connect to LIDAR
    drv->connect(usb_path, baudrate);

    // Test connection
    response = drv->getDeviceInfo(device_info);
    if(IS_OK(response))
    {
        printf("Firmware version: %d.%02d\n"
               "Hardware revision: %d\n",
               device_info.firmware_version>>8,
               device_info.firmware_version & 0xFF,
               (int)device_info.hardware_version);
    }

    // Stop if ctrl-c is pressed
    signal(SIGINT, ctrlc);

    // Start scanning
    drv->startMotor();
    drv->startScan(0,1);

    // Eternal (torment!) loop for getting data
    while(1)
    {
        rplidar_response_measurement_node_hq_t nodes[8192];
        size_t count = _countof(nodes);

        response = drv->grabScanDataHq(nodes, count);
        if (IS_OK(response)) {
            drv->ascendScanData(nodes, count);
            for (int pos = 0; pos < (int)count ; ++pos) {
                printf("%s theta: %03.2f Dist: %08.2f Q: %d \n",
                       (nodes[pos].flag & RPLIDAR_RESP_MEASUREMENT_SYNCBIT) ?"S ":"  ",
                       (nodes[pos].angle_z_q14 * 90.f / (1 << 14)),
                       nodes[pos].dist_mm_q2/4.0f,
                       nodes[pos].quality);
            }
        }
        if (ctrl_c_pressed){
            break;
        }
    }

    // End my misery
    drv->stop();
    drv->stopMotor();
    RPlidarDriver::DisposeDriver(drv);
    drv = NULL;
    return 0;
}