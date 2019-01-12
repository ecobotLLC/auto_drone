#pragma once
int init();
int send_drone(const char *message,int len);
int read_drone(char *buf,int len);
int get_status();
int get_attitude();
int get_rc();
int set_rc(unsigned int *rcData,int len);
int release();

