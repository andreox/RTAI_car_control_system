//------------------- CONTROLLER.C ---------------------- 

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
#include "parameters.h"
#define CPUMAP 0x1


static RT_TASK *main_Task;
static RT_TASK *read_Task;
static RT_TASK *filter_Task;
static RT_TASK *control_Task;
static RT_TASK *write_Task;

static int count_productions = 0 ;

static MBX *mbx ;
static MBX *mbx1 ;

static int keep_on_running = 1;

static pthread_t read_thread;
static pthread_t filter_thread;
static pthread_t control_thread;
static pthread_t write_thread;
static RTIME sampl_interv;

int block_if_other_wheel_is_blocked ;

static void endme(int dummy) {keep_on_running = 0;}

int* sensor;
int* actuator;
int* reference;
diagnostica *diag; 

int buffer[BUF_SIZE];
int head = 0;
int tail = 0;

SEM* space_avail;
SEM* meas_avail;
SEM* mutex ;
SEM* mutex_cons ;
SEM* mutex_prod ;

//semafori per la diagnostica
SEM* diag_avail ;
SEM* space1_avail ;
SEM* scritt_avail;

static RTIME exc_acquire[3];
static RTIME exc_filter[3];
static RTIME exc_control[3];
static RTIME exc_actuator[3];

//Acquire mette BUF_SIZE valori di velocità nel buffer tramite sensor in zona critica
static void * acquire_loop(void * par) {

	if (!(read_Task = rt_task_init_schmod(nam2num("READER"), 1, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT SENSOR TASK\n");
		exit(1);
	}
	rt_get_exectime(read_Task, exc_acquire);
	RTIME acq = exc_acquire[0];
	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(read_Task, expected, sampl_interv);
	rt_make_hard_real_time();

	int curr_buff = 0 ;
	int crash = 1;
	int prev_sen = 0;
	int index = 0;
	while (keep_on_running)
	{
		// DATA ACQUISITION FROM PLANT
		rt_sem_wait(space_avail);
		(head > 0 ) ? (index=(head-1) % BUF_SIZE) : (index = head);
		prev_sen=buffer[index];
		buffer[head] = (sensor[0]); //Velocità memorizzata in sensor viene messa nel buffer
		curr_buff = buffer[head];
		
		if( (prev_sen-curr_buff) >= 30) {
			 rt_mbx_send_if(mbx1, &crash, sizeof(int)) ; rt_printk("Valore ruota 1 : %d \n",curr_buff);
			 }
		
		head = (head+1) % BUF_SIZE; //Gestito come un BUFFER CIRCOLARE

		rt_sem_signal(meas_avail);
				
		rt_sem_wait(mutex) ;
		
		rt_get_exectime(read_Task, exc_acquire);
		acq = exc_acquire[0] - acq;		 
		diag[0].buff = curr_buff ;
		diag[0].exc_acquire = acq;
		rt_sem_signal(mutex) ;

		rt_task_wait_period();
	}
	rt_task_delete(read_Task);
	return 0;
}

//Filter calcola la media dei valori contenuti nel buffer inseriti da Acquire in zona critica
static void * filter_loop(void * par) {

	if (!(filter_Task = rt_task_init_schmod(nam2num("FILTER"), 3, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT FILTER TASK\n");
		exit(1);
	}
	
	rt_get_exectime(filter_Task, exc_filter);
	RTIME flt = exc_filter[0];
	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(filter_Task, expected, sampl_interv);
	rt_make_hard_real_time();

	int cnt = BUF_SIZE;
	unsigned int sum = 0;
	unsigned int avg = 0;
	while (keep_on_running)
	{
		// FILTERING (average) ZONA CRITICA Inizio 
		rt_sem_wait(meas_avail);

		sum += buffer[tail];
		tail = (tail+1) % BUF_SIZE; //buffer è un BUFFER CIRCOLARE

		rt_sem_signal(space_avail); //Zona Critica Fine
		
		cnt--;

		if (cnt == 0) { //Quando ho finito di leggere il buffer, calcolo la media dei valori presenti in esso
			cnt = BUF_SIZE;
			avg = sum/BUF_SIZE;
			sum = 0;
			// sends the average measure to the controller
			rt_send(control_Task, avg);	//Il filtro invia il messaggio avg al controllore	
		}
					
			rt_sem_wait(mutex) ;
			
			rt_get_exectime(filter_Task, exc_filter);
			flt = exc_filter[0] - flt;	
		
			diag[0].curr_avg = avg ;
			
			diag[0].exc_filter = flt;
			
		    rt_sem_signal(mutex) ;

		rt_task_wait_period();
	}
	rt_task_delete(filter_Task);
	return 0;
}

//Setta il valore di control_action in base allo stato dell'impianto
static void * control_loop(void * par) {
	unsigned int prev_sensor=0;             //to store previous sensor readings to detect skids
	unsigned int plant_state = 0;           //speed received from the plant
	int error = 0;                          //error to use to calculate the control action
	unsigned int control_action = 0;        //control action to be sent to the actuator
	unsigned int ANTI_SKID_ON = 1;		//to activate the ANTI SKID
	unsigned int CONTROL_PERIOD_MULTIPLIER = 1;	//to configure the control period
	int block = 0; 				// to check if the wheel is blocked

	int block_if_other_wheel_is_blocked ;

	if (!(control_Task = rt_task_init_schmod(nam2num("CNTRL"), 5, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT CONTROL TASK\n");
		exit(1);
	}
	
	rt_get_exectime(control_Task, exc_control);
	RTIME clt = exc_control[0];
	
	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(control_Task, expected, 
		CONTROL_PERIOD_MULTIPLIER*BUF_SIZE*sampl_interv);
	rt_make_hard_real_time();
     
    //serve 1 altro task, con 1 altro thread che prende in mem condivisa con l'altro controllore il messaggio e lo invia a control_loop. 
	while (keep_on_running)
	{    

		block_if_other_wheel_is_blocked = 0  ;
		// receiving the average plant state from the filter
		rt_receive(0, &plant_state); //Controllore riceve messaggio avg
		if(!rt_mbx_receive_if(mbx, &block_if_other_wheel_is_blocked, sizeof(int)))
			rt_printk("L'altra ruota sta slittando, ha attivato l'Anti-Skid, blocco anche questa (RUOTA 0)\n"); //****


		// evaluating if the wheel is blocked
        if(prev_sensor==(sensor[0])) block = 1;
        else block = 0; 
		// computation of the control law
		error = (*reference) - plant_state;
		if (error > 0) control_action = 1; //setto il comportamento del controllore
		else if (error < 0) control_action = 2;
                else control_action = 3;
		
		if (ANTI_SKID_ON) {
		 //se ho attivato l'anti slittamento
			if (((*reference)==0) && (plant_state!=0) && (block!=1)) {

				control_action = 4; //brake only when no skid is detected.
				/* INVIA MESSAGGIO BLOCCANTE ALL'ALTRA RUOTA */
				int blocca = 1 ; //*** VAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA RRRRRRRRIPOSTO A 0
				rt_printk("INVIO MESSAGGIO SONO IL CONTROLLER 0 %d\n",blocca);
				rt_mbx_send(mbx, &blocca, sizeof(int)) ; //****

			}

		} 
		
		
		else if ( block_if_other_wheel_is_blocked ) control_action = 4 ; //****
		else if ((*reference) == 0) control_action = 4;
		
		if ((*reference) == -5) {
			control_action = 5;
			//*reference = 0;
			}
 
                prev_sensor=(sensor[0]);  
		// sending the control action to the actuator
		rt_send_if(write_Task, control_action); //Controllore invia il messaggio control_action all'attuatore
		
		
				
		rt_sem_wait(mutex) ;
		
		rt_get_exectime(control_Task, exc_control);
		clt = exc_control[0] - clt;
			
		diag[0].block = block ;
		
		diag[0].exc_control = clt;
		
		rt_sem_signal(mutex) ;

        rt_task_wait_period();
	}
	rt_task_delete(control_Task);
	return 0;
}


//Attuatore, in base al valore di control action, aggiorna *actuator
static void * actuator_loop(void * par) {

	if (!(write_Task = rt_task_init_schmod(nam2num("WRITE"), 7, 0, 0, SCHED_FIFO, CPUMAP))) {
		printf("CANNOT INIT ACTUATOR TASK\n");
		exit(1);
	}
	
	rt_get_exectime(write_Task, exc_actuator);
	RTIME act = exc_actuator[0];
	
	RTIME expected = rt_get_time() + sampl_interv;
	rt_task_make_periodic(write_Task, expected, BUF_SIZE*sampl_interv);
	rt_make_hard_real_time();

	unsigned int control_action = 0;
	int cntr = 0;

	while (keep_on_running)

	{		
		// receiving the control action from the controller
		rt_receive(0, &control_action); //Attuatore riceve messaggio control_action dal Controllore
		
		switch (control_action) {
			case 1: cntr = 1; break;
			case 2:	cntr = -1; break;
			case 3:	cntr = 0; break;
            case 4: cntr = -2; break;
            case 5: cntr = -3; break;
			default: cntr = 0;
		}
		
		(actuator[0]) = cntr;
				
		rt_sem_wait(mutex) ;
		
		rt_get_exectime(write_Task, exc_actuator);
		act = exc_actuator[0] - act;
		 
		diag[0].block = cntr ;
		
		diag[0].exc_actuator = act;
		
		rt_sem_signal(mutex) ;
			


		rt_task_wait_period();
	}
	rt_printk("sto morendo\n");
	rt_task_delete(write_Task);
	return 0;
}

int main(void)
{
	printf("The controller is STARTED!\n");
 	signal(SIGINT, endme);

	if (!(main_Task = rt_task_init_schmod(nam2num("MAINTSK"), 0, 0, 0, SCHED_FIFO, 0xF))) {
		printf("CANNOT INIT MAIN TASK\n");
		exit(1);
	}

	//attach to data shared with the controller
	diag = rtai_malloc(DIAG_SHM, sizeof(diagnostica)*NUM_OF_WHEELS);
	sensor = rtai_malloc(SEN_SHM, NUM_OF_WHEELS*sizeof(int));
	actuator = rtai_malloc(ACT_SHM, NUM_OF_WHEELS*sizeof(int));
	reference = rtai_malloc(REFSENS, sizeof(int));

	(*reference) = 110;


	space_avail = rt_typed_sem_init(SPACE_SEM, BUF_SIZE, CNT_SEM | PRIO_Q);
	meas_avail = rt_typed_sem_init(MEAS_SEM, 0, CNT_SEM | PRIO_Q);
	mutex = rt_typed_named_sem_init( MUTEX1, 1, BIN_SEM | FIFO_Q) ;
	sampl_interv = nano2count(CNTRL_TIME);
	mbx = rt_typed_named_mbx_init( MBX_ID, sizeof(int), FIFO_Q) ; //****
	mbx1 = rt_typed_named_mbx_init( MBX1_ID, sizeof(int), FIFO_Q) ; //****
	
	// CONTROL THREADS 
	pthread_create(&read_thread, NULL, acquire_loop, 0);
	pthread_create(&filter_thread, NULL, filter_loop, 0);
	pthread_create(&control_thread, NULL, control_loop, 0);
	pthread_create(&write_thread, NULL, actuator_loop, 0);


	while (keep_on_running) {
		printf("Control: %d %d",(*actuator), block_if_other_wheel_is_blocked );
		printf("\n");
		rt_sleep(10000000);
	}

	rt_shm_free(SEN_SHM);
	rt_shm_free(ACT_SHM);
	rt_shm_free(REFSENS);
	rt_shm_free(DIAG_SHM);

	rt_named_mbx_delete(mbx) ; 
	rt_named_mbx_delete(mbx1) ; 
	rt_sem_delete(meas_avail);
	rt_sem_delete(space_avail);	
	rt_sem_delete(mutex) ;
	rt_task_delete(main_Task);
 	printf("The controller is STOPPED\n");
	return 0;
}




