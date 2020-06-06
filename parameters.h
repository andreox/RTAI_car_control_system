//---------------- PARAMETERS.H ----------------------- 

#define TICK_TIME 100000000
#define CNTRL_TIME 50000000

#define TASK_PRIORITY 1

#define STACK_SIZE 10000

#define BUF_SIZE 10
#define DIAG_SIZE 5

#define MUTEX1  "MyMute1"
#define MUTEX2  "MyMute2"

#define MAILBOX_SIZE 4096

#define SEN_SHM 121111
#define ACT_SHM 112112
#define DIAG_SHM 122222

#define REFSENS 111213
#define MBX_ID "MyBlock"
#define MBX1_ID "MyBloc1"
#define MBX_DIAG_ID "MyDiagg"


#define SPACE_SEM 1234444
#define MEAS_SEM 1234445

#define DIAG_SEM 1234567
#define AV_SEM 1234568

#define NUM_OF_WHEELS 2


#define SEN_SHM1 121111
#define ACT_SHM1 112112

#define SPACE_SEM1 1234441
#define MEAS_SEM1 1234442

#define TS 7 // da provare con altri TS

#define MS 1000000

typedef struct{

	int block;
	int curr_avg;
	int buff;
	int actuator;
	
	RTIME exc_acquire;
	RTIME exc_filter;
	RTIME exc_control;
	RTIME exc_actuator;
}diagnostica;

typedef struct{
	
	diagnostica d[2];
	
	int stato;	
	
}shm_diag;

typedef struct{
	
	int RT[5];
	int RA[5];
	int capacity;
}SporadicServer;
