//---------------------- REFERENCE.C ----------------------------

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <rtai_shm.h>
#include <rtai_mbx.h>
#include "parameters.h"

 
// SETS THE REFERENCE SIGNAL

static MBX *mbx ;
static RT_TASK* asyncTask;
#define CPUMAP 0x1
int main (int argc, char ** argv)

{
	if(!(asyncTask = rt_task_init_schmod(nam2num("U_AIRBAG"), 1, 0, 0, SCHED_FIFO, CPUMAP))){
		printf("failed creating rt task\n");
		exit(-1);
	}
	
	mbx = rt_typed_named_mbx_init( MBX1_ID, sizeof(int), FIFO_Q) ; //****
	int s=0;
	rt_mbx_receive(mbx, &s, sizeof(int));
	if(s == 1) {
		printf("AIRBAG ATTIVATO \n");
}
	printf("Mortooooo\n");	
  

    rt_named_mbx_delete(mbx) ;
	rt_task_delete(asyncTask);
    return 0;

}
