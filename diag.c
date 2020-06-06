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
	if(!(asyncTask = rt_task_init_schmod(nam2num("RT_ASYNC"), 1, 0, 0, SCHED_FIFO, CPUMAP))){
		printf("failed creating rt task\n");
		exit(-1);
	}
	
    shm_diag *s;
	int i=0;
	int j=0;
	int richiesta=0;
	mbx = rt_typed_named_mbx_init( MBX_DIAG_ID, sizeof(shm_diag), FIFO_Q) ; //****
	s = malloc(sizeof( shm_diag));
	for(j=0;j<7;j++){
	printf("Inserisci richiesta : ");
	scanf("%d",&richiesta);
	//richiesta=1;
	if(richiesta == 1) {
		rt_mbx_send(mbx,&richiesta,sizeof(int));
		printf("Sono arrivato qua\n");
		rt_mbx_receive(mbx, s, sizeof(shm_diag));
			printf("Ricevo\n");
		for(i=0;i<NUM_OF_WHEELS;i++)
			printf("%d %d %d %d %lld %lld %lld %lld \n",s->d[i].actuator, s->d[i].block, s->d[i].buff, s->d[i].curr_avg,
			s->d[i].exc_acquire, s->d[i].exc_actuator, s->d[i].exc_control, s->d[i].exc_filter);
}
	printf("Escooooooo\n");	
}
	printf("Esco\n");	
  

    rt_named_mbx_delete(mbx) ;
	rt_task_delete(asyncTask);
    return 0;

}
