#ifndef _gamecube_h
#define _gamecube_h

#include <stdint.h>

#define CONSOLE_PROBE (0)
#define CONSOLE_PROBE_ORIGIN (1)
#define CONSOLE_POLL (2)
#define CONSOLE_POLL_RUMBLE (3)
#define CONSOLE_INVALID (4)

typedef struct {
    // 8 bytes of datareport that we get from the controller
    uint8_t raw8[8];
    uint16_t raw16[0];
    uint32_t raw32[0];

    struct{
        uint8_t buttons0;
        union{
            uint8_t buttons1;
            uint8_t dpad : 4;
        };
    };

    struct {
        // first data byte (bitfields are sorted in LSB order)
        uint8_t a : 1;
        uint8_t b : 1;
        uint8_t x : 1;
        uint8_t y : 1;
        uint8_t start : 1;
        uint8_t origin : 1; // Indicates if GetOrigin(0x41) was called (LOW)
        uint8_t errlatch : 1;
        uint8_t errstat : 1;

        // second data byte
        uint8_t dleft : 1;
        uint8_t dright : 1;
        uint8_t ddown : 1;
        uint8_t dup : 1;
        uint8_t z : 1;
        uint8_t r : 1;
        uint8_t l : 1;
        uint8_t high1 : 1;

        // 3rd-8th data byte
        uint8_t xAxis;
        uint8_t yAxis;
        uint8_t cxAxis;
        uint8_t cyAxis;
        uint8_t left;
        uint8_t right;
    };
} Gamecube_Report;

const uint8_t console_commands[4][4] {
    {0x00, 0x80, 0x00, 0x00}, // PROBE
    {0x41, 0x01, 0x80, 0x00}, // PROBE_ORIGIN
    {0x40, 0x03, 0x00, 0x80}, // POLL
    {0x40, 0x03, 0x01, 0x80}, // POLL_RUMBLE
};

typedef struct {
    Gamecube_Report gc_data;

    uint8_t lx_calib;
    uint8_t ly_calib;
    uint8_t cx_calib;
    uint8_t cy_calib;
    uint8_t l_calib;
    uint8_t r_calib;
} Gamecube_Controller;

#endif