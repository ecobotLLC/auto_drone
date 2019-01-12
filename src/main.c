
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <signal.h>
#include "util/util.h"
#include "io/drone.h"

void SigHandler(int p_signame)
{
  exit(0);
  return;
}
void SetSignal(int p_signame)
{
  if (signal(p_signame, SigHandler) == SIG_ERR) {
    /* シグナル設定エラー  */
    printf("シグナルの設定が出来ませんでした。終了します\n");
    exit(1);
  }

  return;
}
int main(int argc, char *argv[])
{
 SetSignal(SIGINT);
    unsigned char buf[1];
char message[1];
int len = 1;
message[0] = (char)'#';
printf("start \n");
init();
printf("start send_drone\n");
char c;
unsigned int rcData[16];
for(int i =0; i < sizeof(rcData)/sizeof(unsigned int); i++)
{
    rcData[i] = 1500;
}
rcData[0] = 1500;
rcData[1] = 1500;
rcData[2] = 2000;
rcData[3] = 1000;


int count = 0;
while(1){
set_rc(rcData,sizeof(rcData)/sizeof(unsigned int));
count++;
if(count > 10) break;
}
// rcData[0] = 1500;
// rcData[1] = 1500;
// rcData[2] = 1500;
// rcData[3] = 1500;
// set_rc(rcData);
  while(1) {
//get_status();
//get_attitude();
get_rc();
break;
  }
release();
printf("done\n");
    return 0;
}