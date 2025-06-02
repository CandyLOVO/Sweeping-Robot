#pragma once

#include "REG.h"
#include "wit_c_sdk.h"
#include "stdint.h"

struct STime {

    unsigned char ucYear;
    unsigned char ucMonth;
    unsigned char ucDay;
    unsigned char ucHour;
    unsigned char ucMinute;
    unsigned char ucSecond;
    unsigned short usMiliSecond;
};
struct SAcc {

    short a[3];
    short T;
};
struct SGyro {

    short w[3];
    short T;
};
struct SAngle {

    short Angle[3];
    short T;
};
struct SMag {

    short h[3];
    short T;
};

struct SDStatus {
    short sDStatus[4];
};

struct SPress {
    long lPressure;
    long lAltitude;
};

struct SLonLat {

    long lLon;
    long lLat;
};

struct SGPSV {
    short sGPSHeight;
    short sGPSYaw;
    long lGPSVelocity;
};
struct SQ {
    short q[4];
};
