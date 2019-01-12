#include <stdio.h>
#include "util.h"

int getInt(char* buf)
{
    int data = 0;
    data += buf[0] << 8;
    buf++;
    data += buf[1];
    return data;
}
unsigned int getUInt(char* buf)
{
    int data = 0;
    data += buf[0] << 8;
    buf++;
    data += buf[1];
    return data;
}
int setUchar(unsigned int buf,unsigned char * data)
{
    data[0] = buf & 0xFF;
    data[1] = buf >> 8 & 0xFF;
    return 0;
}

