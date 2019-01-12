#include "msp_protocol.h"

#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <stdbool.h>
#include <stdlib.h>
#include "port_config.h"
#include "../util/util.h"
#include "drone.h"

int fd;                             // ファイルディスクリプタ
int init()
{
    unsigned char msg[] = "serial port open...\n";
    struct termios tio;                 // シリアル通信設定
    int baudRate = B115200;
    int i;
    int ret;
    int size;

    fd = open(SERIAL_PORT, O_RDWR );     // デバイスをオープンする
    if (fd < 0) {
        printf("open error\n");
        return -1;
    }
        printf("open \n");
    tio.c_cflag += CREAD;               // 受信有効
    tio.c_cflag += CLOCAL;              // ローカルライン（モデム制御なし）
    tio.c_cflag += CS8;                 // データビット:8bit
    tio.c_cflag += 0;                   // ストップビット:1bit
    tio.c_cflag += 0;                   // パリティ:None
    cfsetispeed( &tio, baudRate );
    cfsetospeed( &tio, baudRate );

    cfmakeraw(&tio);                    // RAWモード

    tcsetattr( fd, TCSANOW, &tio );     // デバイスに設定を行う

    ioctl(fd, TCSETS, &tio);            // ポートの設定を有効にする
    printf("fd %d\n",fd);
    return 0;
}
int send_drone(const char *message,int len)
{
    int ret;
    for(int i = 0; i< len; i++){
        printf("message[%d] %X\n",i,message[i]);
    }
    ret = write(fd, message, len);
    if(ret <= 0){
        printf("send error %d\n",ret);
        return -1;
    }
    return 0;
}
int getPreamblePoint(char *pointa,int len)
{
    for(int i = 0; i < len -1; i++){
        if(*pointa == (unsigned char)PREAMBLE0)
        {
            if(*(pointa +1) == (unsigned char)PREAMBLE1)
            {
                pointa = pointa + 2;
                return i + 2;
            }
        }
        pointa++;
    }
    return 0;
}
int read_drone(char *buf, int size)
{
    int count = 0;
    char str[100];
    char *pointa = &str[0];
    while(1){
        int len = 0;
        len = read(fd,pointa,size);
        if (0 < len) {
            int pos = 0;
            if( 0 < (pos = getPreamblePoint(str,len+ count)))
            {
                for(int i = 0; i < size; i++) 
                {
                    buf[i] = str[i + pos];
                }
                return 0;
            }
            pointa = (pointa + len);
        }
        else
        {
            return -1;
        }
        count += len;
        if(count >= sizeof(str)) return -1;
    }
    return -1;
}
int release()
{
    if (fd < 0) {
        printf("open error\n");
        return 1;
    }
    close(fd);                              // デバイスのクローズ
    return 0;
}
char* setCRC(char *pointa)
{
    char crc;
    crc = *(pointa -1) ^ *(pointa -2);
    *pointa = crc;
    pointa++;
    return pointa;
}
char* setSize(char *pointa, int size)
{
    *pointa= size;
    pointa++;   
    return pointa;
}
char* setStream(char *pointa, bool isInput)
{
    if(isInput)
    *pointa= (unsigned char)INPUT_MODE;
    else
    *pointa= (unsigned char)OUTPUT_MODE;
    pointa++;   
    return pointa;
}
char* setPreamble(char *pointa)
{
    *pointa = (unsigned char)PREAMBLE0;
    pointa++;
    *pointa = (unsigned char)PREAMBLE1;
    pointa++;
    return pointa;
}
int set_data(char *data,unsigned char command,int len)
{
    char str[100];
    char *pointa = &str[0];
    pointa = setPreamble(pointa);
    pointa = setStream(pointa,false);
    pointa = setSize(pointa,len);
    *pointa= (unsigned char)command;
    pointa++;   
    char crc = *(pointa -1) ^ *(pointa -2);
    for(int i = 0; i < len; i++)
    {
    *pointa= data[i];
    crc = crc ^ *pointa;
    pointa++;   
    }
    *pointa = crc;
    send_drone(str,6 + len);    
}

int get_data(char *data,unsigned char command,int len)
{
    char str[100];
    char *pointa = &str[0];
    pointa = setPreamble(pointa);
    pointa = setStream(pointa,true);
    pointa = setSize(pointa,0);
    *pointa= (unsigned char)command;
    pointa++;   
    pointa = setCRC(pointa);
    send_drone(str,6);
    char buf[len];
    read_drone(buf,len);
    int size = 0;
    char commandIn = 0;
    char crcbit = 0;
    if(buf[0] == (unsigned char)OUTPUT_MODE)
    {
        size = buf[1];
        commandIn = buf[2];
        if(commandIn != command) return -1;
        crcbit =  buf[3];
        char bufint[2];
        for(int i = 4; i< size +4; i++)
        {
            data[i -4] = buf[i]; 
        }
        return size;
    }
    return -1;
}
int get_status()
{
    int len = 18+6;
    int size = 0;
    char buf[len];
    if(0 < (size = get_data(buf,(unsigned char)MSP_RAW_IMU,18+6)))
    {
        char bufint[2];
        for(int i = 0; i< size /2; i++)
        {
            bufint[0] = buf[i * 2];
            bufint[1] = buf[i + 1];
            int accx = 0;
            accx = getInt(bufint);
            printf("getInt16 %d\n",accx);
        }
    }
}
int get_attitude()
{
    int len = 6+6;
    int size = 0;
    char buf[len];
    if(0 < (size = get_data(buf,(unsigned char)MSP_ATTITUDE,len)))
    {
        char bufint[2];
        for(int i = 0; i< size /2; i++)
        {
            bufint[0] = buf[i * 2];
            bufint[1] = buf[i + 1];
            int accx = 0;
            accx = getInt(bufint);
            printf("getInt16 %d\n",accx);
        }
    }
}
int get_rc()
{
    int len = 6+16*2;
    int size = 0;
    char buf[len];
    if(0 < (size = get_data(buf,(unsigned char)MSP_RC,len)))
    {
        unsigned char bufint[2];
        for(int i = 0; i< size /2; i++)
        {
            bufint[0] = buf[i * 2];
            bufint[1] = buf[i + 1];
            unsigned int accx = 0;
            accx = getUInt(bufint);
            printf("getRC %d : %u\n",i,accx);
        }
    }

}

int set_rc(unsigned int *rcData,int len)
{
        printf("len : %d\n",len);    
    unsigned char data[len];
    for(int i = 0; i < len /2; i++)
    {
        unsigned char buf[2];
        setUchar(*(rcData + i),buf);
        for(int k = 0; k < 2; k++)
        {
            data[i*2 + k] = buf[k];
        }

    }
    for(int j = 0; j <len; j++)
    {
        printf("setRC %d : %x\n",j,data[j]);
    }
    set_data(data,(unsigned char)MSP_SET_RAW_RC,len);
}