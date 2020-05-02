#ifndef __SIGMA5ABS_H
#define __SIGMA5ABS_H


typedef struct {
    unsigned int magic : 5;
    unsigned int indexed : 1;
    unsigned int magic2 : 10;
    unsigned int slowcounter : 8;
    unsigned int fastcounter : 26;
    unsigned int magic3 : 6;
    unsigned int position : 24;
    unsigned int z : 1;
    unsigned int hall_a : 1;
    unsigned int hall_b : 1;
    unsigned int hall_c : 1;
    unsigned int offset : 9;
    unsigned int turns : 3;
    unsigned int crc : 16;
} sigma5abs_serial_data_t;





#endif

