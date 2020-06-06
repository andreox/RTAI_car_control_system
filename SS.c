#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <rtai_lxrt.h>
#include <rtai_shm.h>
#include <rtai_sem.h>
#include <rtai_msg.h>
#include <rtai_mbx.h>
#include <sys/io.h>
#include <signal.h>
#include <stdbool.h>
#include "parameters.h"
#define CPUMAP 0x1


static int keep_on_running = 1;
static RT_TASK *sporadic_server;
static RT_TASK *sporadic_task ;
static RT_TASK *airbag_task ;

static pthread_t sporadic_thread ;
static pthread_t airbag_thread ;

diagnostica * diag; 
shm_diag * s_diag;
SporadicServer * SS;

static RTIME sampl_interv;
SEM* mutex ;
static MBX *mbx ;
static MBX *mbx1 ;
int* reference;
static void endme(int dummy) {keep_on_running = 0;}

bool attivo;
int t;
	
static void *sporadic_loop( void*par ) {

	int i = 0;
	int richiesta =0;
	int index=0;
	int coda=0;
	int head = 0;
	int var_index=0;
	if (!(sporadic_task = rt_task_init_schmod(nam2num("SPORADIC"), 1, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT SPORADIC TASK\n");
		exit(1);
	}


	rt_make_hard_real_time();
	
	rt_printk("SS ATTIVATO\n");
	
	while(keep_on_running){
			rt_mbx_receive(mbx, &richiesta, sizeof(int));
			rt_printk(" Tempo : %d, capacità: %d , RA %d : \n", t, SS->capacity,SS->RA[index]);
			
			for ( i = coda ; i != index && SS->RT[i] <= t; i = (i+1)%5 ){
				if(SS->capacity <3) {
					SS->capacity += SS->RA[i]; 
					SS->RA[i]=0;
					rt_printk("Sommo varia %d \n", SS->capacity);
					}
			}
			
			if( SS->capacity >0) {
			rt_printk(" Capacità attuale %d al tempo %d: \n", SS->capacity ,t);
			SS->RT[index] = t + TS;
			SS->capacity --;
			SS->RA[index] = 1;
			index = (index+1);
			var_index = (var_index) +1; // ci serve per la gestione circolare
			if(var_index>=5) coda++;
			index = index%5;
			if(richiesta==1) {
			
			rt_sem_wait( mutex );
		
			// SE task diag manda un messaggio, svegliati
			for(i=0;i<NUM_OF_WHEELS;i++) {
				//rt_printk("%d %d %d %d\n",diag[i].actuator, diag[i].block, diag[i].buff, diag[i].curr_avg);
				s_diag->d[i]=diag[i];
				rt_printk("%d %d %d %d %lld \n",s_diag->d[i].actuator, s_diag->d[i].block,s_diag->d[i].buff,s_diag->d[i].curr_avg,s_diag->d[i].exc_acquire);
			}
			richiesta=0;
			
			rt_mbx_send(mbx,s_diag,sizeof(shm_diag));
		
			rt_sem_signal(mutex) ;
			
			rt_sleep(10000000);
			//rt_busy_sleep(nano2count(TICK_TIME));
		}
	}
	t++;
}

	rt_task_delete(sporadic_task);

}

static void *airbag_loop( void*par ) {

	int i = 0;
	int richiesta =0;
	int stop=1;
	if (!(airbag_task = rt_task_init_schmod(nam2num("AIRBAG"), 1, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT AIRBAG TASK\n");
		exit(1);
	}


	rt_make_hard_real_time();
	
	while(stop){
			
			rt_mbx_receive(mbx1, &richiesta, sizeof(int));
			
			if(richiesta==1) {
			rt_mbx_send_if(mbx1,&richiesta,sizeof(int));
			*reference = -2;
			stop=0;		
		}
		
	}
	rt_printk("MUOIO \n");
	rt_task_delete(airbag_task);

}

int main( int argc, char** argv) {

	

	printf("THE SPORADIC SERVERR HAS STARTED\n");

	
	signal(SIGINT, endme) ;
	

	if (!(sporadic_server = rt_task_init_schmod(nam2num("SERVER"), 1, 0, 0, SCHED_FIFO, 0xF))) {
		printf("CANNOT INIT MAIN SERVER TASK\n");
		exit(1);
	}

	diag = rtai_malloc(DIAG_SHM, sizeof( diagnostica)*NUM_OF_WHEELS);
	mbx = rt_typed_named_mbx_init( MBX_DIAG_ID, sizeof(shm_diag), FIFO_Q) ; //****
	mbx1 = rt_typed_named_mbx_init( MBX1_ID, sizeof(int), FIFO_Q) ; //****
	s_diag = malloc(sizeof( shm_diag));
	mutex = rt_typed_named_sem_init( MUTEX1, 1, BIN_SEM | FIFO_Q) ;
	reference = rtai_malloc(REFSENS, sizeof(int));
	SS = malloc(sizeof(SporadicServer));
	
	SS->capacity=3; // provare con altre capacità
	t=0;
	pthread_create( &sporadic_thread, NULL, sporadic_loop, 0);
	pthread_create( &airbag_thread, NULL, airbag_loop, 0);
	int n = 0;
	attivo = false;
	while ( keep_on_running ){
		if(n==30){ // da provare con tempo più lento o più veloce
		t++;
		n=0;
		}
		rt_sleep(10000000);
		n++;
		
	} 

	rt_shm_free(DIAG_SHM);
	rt_named_mbx_delete(mbx) ;
	rt_named_mbx_delete(mbx1) ;
	//free(my_diag);
	rt_sem_delete(mutex) ;
	rt_task_delete(sporadic_server);

	return 0 ;

}


// WHILE(1) SE RICEVO MESSAGGIO DA DIAG CON VALORE INDICE DELLA STRUTTURA IN MEMORIA CONDIVISA ( QUINDI DIVERSO DA -1 )
// ALLORA ACCEDIAMO ALLA ZONA CRITICA, LEGGIAMO LA STRUTTURA E PASSIAMO I VALORI DELLA STRUTTURA A DIAG, CHE STAMPERA A VIDEO I VALORI RICEVUTI
