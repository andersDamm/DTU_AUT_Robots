/*
* An example SMR program.
*
*/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>

#include <sys/ioctl.h>
#include "rhd.h"
#include "componentserver.h"
#include "xmlio.h"

struct xml_in *xmldata;
struct xml_in *xmllaser;
struct {
  double x, y, z, omega, phi, kappa, code,id,crc;
} gmk;
double visionpar[10];
double laserpar[10];

void serverconnect(componentservertype *s);
void xml_proc(struct xml_in *x);
void xml_proca(struct xml_in *x);

typedef struct { int time;
	double x,y,theta;
} poseTimeLog_t;


componentservertype lmssrv,camsrv;

symTableElement *
getinputref (const char *sym_name, symTableElement * tab)
{
  int i;
  for (i=0; i< getSymbolTableSize('r'); i++)
   if (strcmp (tab[i].name,sym_name) == 0)
	 return &tab[i];
 return 0;
}

symTableElement *
getoutputref (const char *sym_name, symTableElement * tab)
{
  int i;
  for (i=0; i< getSymbolTableSize('w'); i++)
   if (strcmp (tab[i].name,sym_name) == 0)
	 return &tab[i];
 return 0;
}

/*****************************************
* odometry
*/

#define IS_SIMULATION 0 //1=simulation, 0=real world

#if IS_SIMULATION==1
    #define WHEEL_SEPARATION 0.267	/* m 0.252 */
#else
    #define WHEEL_SEPARATION 0.28986 /* m 0.252 */
#endif

#define WHEEL_DIAMETER   0.067
#define LENGTH_OF_ROBOT 0.26    /*meters from center of wheels to end of front IR sensors */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT	24902
#define AJAX 0.5 /* m/s² */
#define SIZE_OF_ARRAY 10000
#define CONVERSION_FACTOR_ACC 0.01
#define K_FOR_STRAIGHT_DIRECTION_CONTROL 1
#define K_FOR_ACCELERATING_DIRECTION_CONTROL 0.001
#define K_FOR_FOLLOWLINE 0.1
//SIM: [0.1 0.001 0.001]   RW: [0.1 0 0.001]
#define KP_FOR_FOLLOWLINE 0.1
#define KI_FOR_FOLLOWLINE 0.00
#define KD_FOR_FOLLOWLINE 0.001

#define KP_FOR_FOLLOWWALL 0.5
#define KI_FOR_FOLLOWWALL 0
#define KD_FOR_FOLLOWWALL 0
#define NUMBER_OF_IRSENSORS 8
#define CRITICAL_IR_VALUE 0.5
#define OBSTACLE_DIST 0.20
#define CRITICAL_BLACK_VALUE 0.8
#define CRITICAL_FLOOR_VALUE 0.2
#define CRIT_NR_BLACK_LINE 6
#define DONT_CARE 0

//begin gateOnTheLoose Definitions :
#define K_FOR_DRIVE_TOWARDS_CENTER_OF_GATE 0.05
#define IS_LEFT_SIDE_BLACK 2.5
#define LASERSCANNER_ZONES 9
#define DESIRED_DIST_TO_FRONT_OBJECT 5
//end gateOnTheLoose Definitions

//Dist to first box sntants and variables:
#define START_POS_TO_BOX_MAX_DIST 2.0
#define LASER8VALUES 100
int laserOld8;
double laser8Values[LASER8VALUES];
int drivePastBoxCounter;
double meanLaser8;
//end, Dist to first box sntants and variables:

/* 	DB_STOPCOND
* 	fwd: 				0=stop by dist	1=stop by wall detection	
*						2=stop by line black line detection		3=stop by right IR sensor
* 	followLineCenter: 	0=stopline		1=dist 		2=object in front(ir)	3=object in front (laser)	4=object to the left
*	follow_wall: 		0=hole in wall 	1=object on the other side, 2 for 0 but laser, 3 for 1 but laser
*
*/
/* DB_OtherCond
*  int follow_wall(int side, double dist, double speed, int condition, int time);  Side: 0 is left, 1 is right
*
*
*/




typedef struct{ //input signals
		int left_enc,right_enc; // encoderticks
		// parameters
		double w;	// wheel separation
		double cr,cl;   // meters per encodertick
		//output signals
		double right_pos,left_pos, y_pos, x_pos, theta, theta_ref;
		// internal variables
		int left_enc_old, right_enc_old;
  } odotype;

  void reset_odo(odotype *p);
  void update_odo(odotype *p);
  int log_data_to_file(poseTimeLog_t * poseTimeLog_out, int size);



/********************************************
* Motion control
*/

typedef struct{//input
	int cmd;
	int curcmd;
	double speedcmd;
	double dist;
	double angle;
	double start_angle;
  	double left_pos,right_pos;
	// parameters
  	double w;
	//output
  	double motorspeed_l,motorspeed_r;
  	int finished;
	// internal variables
  	double startpos;
  	//errors
  	double error_current, error_old, error_sum;
  	//stop condition
  	int stop_condition;
	double laser_old; //The old laservalue of laserpar[0]
}motiontype;

enum {
	mot_stop=1,mot_move,mot_turn, mot_turnr,mot_followLineCenter,
	mot_followRightLine,mot_followLeftLine,mot_follow_wall_left,
	mot_follow_wall_right, mot_reverse,
	mot_detect_line,mot_followWhiteLine,
	mot_followLineCenterTwoGatePolesDetected,
	mot_reverseTillBlackLine, mot_driveTowardsCenterOfGate,
	mot_turnTowardsCenterOfGate
};

void update_motcon(motiontype *p);

/********************************************
* Mission control
*/
int fwd(double dist, double speed, int condition, int time);
int turn(double angle, double speed,int time);
int turnr(double radius, double angle, double speed, int time);
int followLineCenter(double dist, double speed, int condition, int time);
int followWhiteLine(double dist, double speed, int time);
double center_of_gravity(int* input, int size, char color);  // Finding the line with centre of gravity algorithm
int follow_wall(int side, double dist, double speed, int condition, int time);
int followRightLine(double dist, double speed, int time);
int followLeftLine(double dist, double speed, int time);


void getTransformedIRData(double* output); // Calibfunction - Calibrates in relation to black_mean.
/********************************************
* Sensor functions and variables
*/
int minIntensity();             // Minimum intensity function
int maxIntensity();
double centerOfGravity(char color);  // Finding the line with center of gravity algorithm
char stopLineNazi();

double leftMostNegSlope();
double leftMostPosSlope();

// Returns a number from 0 to 7, which indicates the position
// at which the right most negative slope begins.
double rightMostNegSlope();
double minDistFrontIR();         // Finds the shortest distance to an object in front, measured by the IR sensor, in cm
double* getDistIR(double* dist);             // Returns the distance all IR's measure, in an array[5], measured in cm.

double Ka_IR[5] = {1523.280675968216, 1523.280675968216, 1478.278602346147, 1515.870801518367, 1515.870801518367};
double Kb_IR[5] = {93.590281110006, 93.590281110006, 94.007320920384, 92.744874767269, 92.744874767269};
double Ka_IR_sim[5] = {1634.64672091309,1634.64672091309, 1634.64672091309,1634.64672091309,1634.64672091309};
double Kb_IR_sim[5] = {73.5153115510393, 73.5153115510393, 73.5153115510393, 73.5153115510393, 73.5153115510393};


// Returns a number from 0 to 7, which indicates the position
// at which the right most positive slope begins.
double rightMostPosSlope();
char stopLine();
char detectLine();

//Initialisation of functions used in "gateOnTheLoose":
int reverseTillBlackLine(double dist, double speed, int time);
int turnTowardsCenterOfGate(double speed, int time);
int driveTowardsCenterOfGate(double speed, int time);
int hasFrontObjectBeenReached(double dist, double speed, int time);
int followLineCenterTwoGatePolesDetected(double dist, double speed, int time);
double centerOfGateWithLaserScanner();
double distanceToCenterOfGate();
int leftMostPillar();
int rightMostPillar();
double trueTillDistToFrontObjectIsReached(double dist);
int numberOfGatePolesDetected; //Used in the case mot_followLineCenterTwoGatePolesDetected
//End of initialisation of functions used in "gateOnTheLoose"


typedef struct{
  int state,oldstate;
  int time;
}smtype;

void sm_update(smtype *p);

// SMR input/output data

symTableElement *  inputtable,*outputtable;
symTableElement *lenc,*renc,*linesensor,*irsensor, *speedl,*speedr,*resetmotorr,*resetmotorl;

odotype odo;
smtype mission;
motiontype mot;

enum {	ms_init,ms_fwd,ms_turn,ms_turnr,ms_followLineCenter,
		ms_followRightLine,ms_followLeftLine,ms_follow_wall,
		ms_PushNDrive_SIM, ms_PushNDrive_RW,ms_end,ms_wall_gate_SIM,
		ms_last_box_RW,ms_followWhiteLine_SIM, ms_followWhiteLine_RW,
		ms_distanceToBox_SIM,ms_distanceToBox_RW,ms_gateOnTheLoose_SIM, 
		ms_gateOnTheLoose_RW, ms_last_box_SIM,ms_wall_gate_RW,ms_whiteLineTest,
		ms_DistBoxL
};

int main()
{
  poseTimeLog_t poseTimeLog_a[SIZE_OF_ARRAY];
  int counter = 0;
  int running,n=0,arg,time=0,laser8sum;
  double dist=0,angle=0;
  double x,x_ref=0,distance_Box = 0; // Variables used for "distance to box"
  FILE *distance_f;
  double IR_dist[5];
  int i;
  
  /* Establish connection to robot sensors and actuators.
  */
  if (rhdConnect('w',"localhost",ROBOTPORT)!='w'){
	printf("Can't connect to rhd \n");
	exit(EXIT_FAILURE);
  }

printf("connected to robot \n");
if ((inputtable=getSymbolTable('r'))== NULL){
   printf("Can't connect to rhd \n");
   exit(EXIT_FAILURE);
}
if ((outputtable=getSymbolTable('w'))== NULL){
   printf("Can't connect to rhd \n");
   exit(EXIT_FAILURE);
}
	  // connect to robot I/O variables
lenc=getinputref("encl",inputtable);
renc=getinputref("encr",inputtable);
linesensor=getinputref("linesensor",inputtable);
irsensor=getinputref("irsensor",inputtable);

speedl=getoutputref("speedl",outputtable);
speedr=getoutputref("speedr",outputtable);
resetmotorr=getoutputref("resetmotorr",outputtable);
resetmotorl=getoutputref("resetmotorl",outputtable);
	// **************************************************
//  Camera server code initialization
//

/* Create endpoint */
lmssrv.port=24919;
strcpy(lmssrv.host,"127.0.0.1");
strcpy(lmssrv.name,"laserserver");
lmssrv.status=1;
camsrv.port=24920;
strcpy(camsrv.host,"127.0.0.1");
camsrv.config=1;
strcpy(camsrv.name,"cameraserver");
camsrv.status=1;

if (camsrv.config) {
  int errno = 0;
  camsrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if ( camsrv.sockfd < 0 )
  {
	perror(strerror(errno));
	fprintf(stderr," Can not make  socket\n");
	exit(errno);
}

serverconnect(&camsrv);

xmldata=xml_in_init(4096,32);
printf(" camera server xml initialized \n");

}


// **************************************************
//  LMS server code initialization
//

/* Create endpoint */
lmssrv.config=1;
if (lmssrv.config) {
  char buf[256];
  int errno = 0,len;
  lmssrv.sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if ( lmssrv.sockfd < 0 )
  {
	perror(strerror(errno));
	fprintf(stderr," Can not make  socket\n");
	exit(errno);
}

serverconnect(&lmssrv);
if (lmssrv.connected){
	xmllaser=xml_in_init(4096,32);
	printf(" laserserver xml initialized \n");
	len=sprintf(buf,"scanpush cmd='zoneobst'\n");
	send(lmssrv.sockfd,buf,len,0);
}

}


  /*
  /  Read sensors and zero our position.
  */
  rhdSync();

  odo.w= WHEEL_SEPARATION;
  odo.cr=DELTA_M;
  odo.cl=odo.cr;
  odo.left_enc=lenc->data[0];
  odo.right_enc=renc->data[0];
  reset_odo(&odo);
  printf("position: %f, %f\n", odo.left_pos, odo.right_pos);
  mot.w=odo.w;
  running=1;
  mission.state=ms_init;
  mission.oldstate=-1;

if (IS_SIMULATION==0){
	odo.w = 0.28986;
	odo.cr = odo.cr * 1.0048;
	odo.cl = odo.cr;
}

/*
* 	Check simulation / real world
*/
if(IS_SIMULATION){
    printf("Shit I'm caught in a simulation!\n");
	for(n=0;n<5;n++){
		Ka_IR[n] = Ka_IR_sim[n];
		Kb_IR[n] = Kb_IR_sim[n];
	}
    n=0;
}
else{
	printf("REAL WORLD!!!!!!!!!!\n");
}
  /*
   * Run loop
  */

  while (running){
	if (lmssrv.config && lmssrv.status && lmssrv.connected){
	  while ( (xml_in_fd(xmllaser,lmssrv.sockfd) >0))
		xml_proca(xmllaser);
	}

	if (camsrv.config && camsrv.status && camsrv.connected){
	  while ( (xml_in_fd(xmldata,camsrv.sockfd) >0))
		xml_proc(xmldata);
	}


	rhdSync();
	odo.left_enc=lenc->data[0];
	odo.right_enc=renc->data[0];
	update_odo(&odo);



  /****************************************
  / mission statemachine
  */
sm_update(&mission);
switch (mission.state) {
	case ms_init:
		n=0; dist=1;angle= -90.0/180*M_PI;
        if(IS_SIMULATION){
            mission.state=ms_distanceToBox_SIM;
            printf("Beginning the ms %d in the sim!\n", mission.state);
        } else{
            mission.state=ms_distanceToBox_RW;
        }
	break;

	case ms_fwd:  //stop_condition: 0=stop by dist, 1=stop by wall detection, 2=stop by line black line detection
		if (fwd(dist,0.3,0,mission.time)){  mission.state=ms_turn;} // Square min ven
	break;

	case ms_turn:
		if (turn(angle,0.3,mission.time)){
			n=n+1;
			if (n==4)
				mission.state=ms_end;
			else
				mission.state=ms_fwd;
		}
	break;

	case ms_turnr:
		if(turnr(1,angle,0.6,mission.time)){
			n=n+1;
			if(n == 4)
				mission.state=ms_end;
			else
				mission.state=ms_whiteLineTest;
		}
	break;

	case ms_followLineCenter: // Cond: 0 for stopline, 1 for dist, 2 for object in front
		if (followLineCenter(dist,0.1,0,mission.time)) mission.state = ms_end; 
    break;

    case ms_follow_wall: //Side = 0 = left   Side = 1 = right
        if(follow_wall(1,0.30,0.2,3,mission.time)) mission.state = ms_end;
    break;

    case ms_followRightLine:
        if (followRightLine(2,0.3,mission.time)) mission.state = ms_followLineCenter;
    break;

	case ms_followLeftLine:
		if (followLeftLine(2,0.3,mission.time)) mission.state = ms_end;
	break;

    case ms_whiteLineTest:
        if (followWhiteLine(2,0.2,mission.time)) mission.state = ms_end;
    break;
    
    case ms_end:
        mot.cmd=mot_stop;
        running=0;
        // Output the data
        if(log_data_to_file(poseTimeLog_a, counter)){
            printf("No data logged\n");
        }
    break;

	case ms_DistBoxL:
		if (n == 0) {
			if(fwd(1.0,0.3,5,mission.time)){ mission.time = -1; n = 1; }
		}
		else if (n == 1) {
			if (fwd(0.4, -0.3, 0, mission.time)) { mission.time = -1; n = 2; }
		}
		else if (n==2){
			laser8sum = 0;
			for (i = LASER8VALUES - 1; i >= 0; i--) {laser8sum += laser8Values[i];}
			distance_Box = laser8sum/(LASER8VALUES);
			distance_f = fopen("Distance_to_box", "w");
			fprintf(distance_f, "x-distance is: %f \n", distance_Box);
			fclose(distance_f);
			printf("\ndistance = %f\n\n", distance_Box);
			mission.time = -1; n = 3;
		}
		else if (n == 3) {
			if (followLeftLine(2.0, 0.3, mission.time))		 { mission.time = -1; n = 4; }
		}
		else if (n == 4) {
			if (followLineCenter(1.0, 0.3, 0, mission.time)) { mission.time = -1; n = 5; }
		}
		else if (n == 5) {
			if (turn(-45.0 / 180.0*M_PI,0.3, mission.time))		 { mission.time = -1; n = 6; }
		}
		else if (n == 6) {
			if (followLineCenter(0.5, 0.3, 0, mission.time)) { mission.time = -1; n = 7; }
		}
		else if (n == 7) {
			if (followLineCenter(0.5, 0.3, 1, mission.time)) { mission.time = -1; n = 8; }
		}
		else if (n == 8) {
			mission.time = ms_PushNDrive_RW;
		}
		break;

    case ms_distanceToBox_RW:
    if(n==0){
      if(followRightLine(1.3,0.2,mission.time)){
	mission.time=-1; n=1;
      }
    }
    else if(n==1){
        if(followLineCenter(0.4,0.2,1,mission.time)){
            mission.time=-1; n=2;
        }
    }
    else if(n==2){
      x=(odo.left_pos+odo.right_pos)/2 + getDistIR(IR_dist)[2]+ LENGTH_OF_ROBOT-0.49;  // Calibrate odo sim
      distance_Box = x-x_ref-0.22;
      mission.time=-1; n=3;
      distance_f = fopen("Distance_to_box","w");
      fprintf(distance_f,"x-distance is: %f \n",distance_Box);
      fclose(distance_f);
		printf("\ndistance = %f\n\n",distance_Box);
      }
    else if(n==3){
      if(turn(90.0/180*M_PI,0.3,mission.time)){
      mission.time=-1; n=4;
      }
    }
    else if(n==4){	//stop_condition: 0=stop by dist, 1=stop by wall detection, 2=stop by line black line detection
      if(fwd(0,0.3,2,mission.time)){
	mission.time=-1;n=5;
      }
    }
    else if(n==5){
      if(fwd(0.3,0.4,0,mission.time)){
	mission.time=-1;n=6;
      }
    }
    else if(n==6){
      if(fwd(0,0.4,2,mission.time)){
	mission.time=-1; n= 7;
      }
    }
    else if(n==7){
      if(fwd(0.2,0.4,0,mission.time)){
	mission.time=-1; n= 8;
      }
    }
    else if(n==8){
      if(turn(-85.0/180*M_PI,0.3,mission.time)){
	  mission.time =-1; n=9;
      }
    }
    else if(n==9){
      n=0; mission.time=-1;
        mission.state=ms_PushNDrive_RW;      
    }
    break;

    
    case ms_distanceToBox_SIM:
    if(n==0){
      if(followRightLine(1,0.3,mission.time)){
	mission.time=-1; n++;
      }
    }
    else if(n==1){
        if(followLineCenter(0.5,0.3,1,mission.time)){
	        mission.time=-1; n++;
        }
    }
    else if(n==2){
      x=(odo.left_pos+odo.right_pos)/2 + getDistIR(IR_dist)[2]+ LENGTH_OF_ROBOT-0.49;  // Calibrate odo sim
      distance_Box = x-x_ref;
      mission.time=-1; n++;
      distance_f = fopen("Distance_to_box","w");
      fprintf(distance_f,"x-distance is: %f \n",distance_Box);
      fclose(distance_f);
		printf("\ndistance = %f\n\n",distance_Box);
      }
    else if(n==3){
      if(turn(90.0/180*M_PI,0.3,mission.time)){
      mission.time=-1; n++;
      }
    }
    else if(n==4){	//stop_condition: 0=stop by dist, 1=stop by wall detection, 2=stop by line black line detection
      if(fwd(0,0.3,2,mission.time)){
	mission.time=-1;n++;
      }
    }
    else if(n==5){
      if(fwd(0.3,0.4,0,mission.time)){
	mission.time=-1;n++;
      }
    }
    else if(n==6){
      if(fwd(0,0.4,2,mission.time)){
	mission.time=-1; n++;
      }
    }
    else if(n==7){
      if(fwd(0.2,0.4,0,mission.time)){
	mission.time=-1; n++;
      }
    }
    else if(n==8){
      if(turn(-90.0/180*M_PI,0.3,mission.time)){
	  mission.time =-1; n++;
      }
    }
    else if(n==9){
      n=0; mission.time=-1;
        mission.state=ms_PushNDrive_SIM;      
    }
    break;

    
  case ms_followWhiteLine_RW:  // White line task
    if(n==0){	 // Cond: 0 for stopline, 1 for dist, 2 for object in front
      if(followLineCenter(4,0.3,0,mission.time)){
        mission.time=-1; n=1;
      }
    }
    else if(n==1){	//stop_condition: 0=stop by dist, 1=stop by wall detection, 2=stop by line black line detection
      if(fwd(0.4,0.3,0,mission.time)){
	mission.time = -1; n=2;
      }
    }
    else if(n==2){
      if(turn(90.0/180*M_PI, 0.3,mission.time)){
	mission.time=-1; n=3;
      }
    }
    else if(n==3){
    if(followWhiteLine(2.5,0.2,mission.time)){
      mission.time =-1; n=4;
      }
    }
    else if(n==4){
      if(fwd(0.35,0.3,0,mission.time)){
	mission.time=-1; n=5;
      }
    }
    else if(n==5){
      if(turn(-90.0/180*M_PI, 0.3, mission.time)){
	mission.time=-1; n=6;
      }
    }
    else if(n== 6){
      mission.state = ms_last_box_RW;
		n=0;
    }
    break;
    
    case ms_followWhiteLine_SIM:  // White line task
    if(n==0){	 // Cond: 0 for stopline, 1 for dist, 2 for object in front
      if(followLineCenter(4,0.3,0,mission.time)){
        mission.time=-1; n=1;
      }
    }
    else if(n==1){	//stop_condition: 0=stop by dist, 1=stop by wall detection, 2=stop by line black line detection
      if(fwd(0.4,0.3,0,mission.time)){
	mission.time = -1; n=2;
      }
    }
    else if(n==2){
      if(turn(90.0/180*M_PI, 0.3,mission.time)){
	mission.time=-1; n=3;
      }
    }
    else if(n==3){
    if(followWhiteLine(2.5,0.2,mission.time)){
      mission.time =-1; n=4;
      }
    }
    else if(n==4){
      if(fwd(0.35,0.3,0,mission.time)){
	mission.time=-1; n=5;
      }
    }
    else if(n==5){
      if(turn(-90.0/180*M_PI, 0.3, mission.time)){
	mission.time=-1; n=6;
      }
    }
    else if(n== 6){
      mission.state = ms_last_box_SIM;
		n=0;
    }
    break;

  case ms_PushNDrive_SIM:        // Push box and gate
		if(n==0){ // Cond: 0 for stopline, 1 for dist, 2 for object in front
//		    printf("Min IR dist = %f\n",minDistFrontIR());
	  		if(followLineCenter(0.2, 0.3, 2, mission.time)){
				mission.time=-1; n=1;
	  		}
		}else if(n==1){	//stop_condition: 0=stop by dist, 1=stop by wall detection, 2=stop by line black line detection
		  	if(fwd(0.50,0.3,0,mission.time)){
				mission.time=-1; n=2;
		  	}
		}else if(n==2){
		  	if(fwd(0.95,-0.3,0,mission.time)){
				mission.time=-1; n = 3;
		  	}
		}else if(n==3){
		  	if(turn(-90.0/180*M_PI,0.3,mission.time)){
				mission.time=-1; n = 4;
		  	}
		}else if(n==4){                                      // Drive till line found
		  	if(fwd(0,0.2,2,mission.time)){
				mission.time=-1; n = 5;
		  	}
		}else if(n==5){
		  	if(turnr(0.2,90.0/180*M_PI,0.3,mission.time)){
				mission.time=-1; n = 6;
		  	}
		}else if(n==6){ // Cond: 0 for stopline, 1 for dist, 2 for object in front
		  	if(followLineCenter(1, 0.2, 0, mission.time)){
				mission.time=-1; n = 7;
		  	}
		}
	 	else if(n==7){	//stop_condition: 0=stop by dist, 1=stop by wall detection, 2=stop by line black line detection
		  	if(fwd(0.2,0.2,0,mission.time)){
				mission.time = -1; n = 8;
		  	}
		}
		else if(n==8){
		  	if(turn(90.0/180*M_PI,0.3,mission.time)){
				mission.time = -1; n = 9;
		  	}
		}
		else if(n==9){  // Cond: 0 for stopline, 1 for dist, 2 for object in front
		  	if(followLineCenter(0.9,0.3,0,mission.time)){
				mission.time=-1; n = 11;
		  	}
		}
	 	else if(n==11){
		  	if(fwd(0.2,0.3,0,mission.time)){
				mission.time=-1; n = 12;
		  	}
	 	}
	 	else if(n==12){
		  	if(followRightLine(0.4,0.3,mission.time)){
				mission.time=-1; n = 13;
		  	}
	 	}
	 	else if(n==13){  // Cond: 0 for stopline, 1 for dist, 2 for object in front
		 	if(followLineCenter(5,0.3,0,mission.time)){
		   		mission.time =-1; n= 14;
			}
        } else if(n==14){
            if(fwd(0.12,0.1,0,mission.time)){
                mission.time = -1; n=15;
            }
        }else if(n == 15){
		    n=0;
		    mission.state=ms_gateOnTheLoose_SIM;
	  	}
  	break;

  case ms_PushNDrive_RW:
	if(n==0){
	    if(followLineCenter(0.2,0.2, 1, mission.time)){
		mission.time=-1; n=1;
	  }
	}
	else if(n==1){ // Cond: 0 for stopline, 1 for dist, 2 for object in front, 3 for obj with laser
        if(followLineCenter(4,0.2, 3, mission.time)){
		  mission.time=-1; n=2;
        }
	}else if(n==2){//stop_condition: 0=stop by dist, 1=stop by wall detection, 2=stop by line black line detection
	  if(fwd(0.55,0.2,0,mission.time)){
		mission.time=-1; n=3;
	  }
	}else if(n==3){
	  if(fwd(0.85,-0.2,0,mission.time)){
		mission.time=-1; n = 4;
	  }
	}else if(n==4){
	  if(turn(-90.0/180*M_PI,0.15,mission.time)){
		mission.time=-1; n = 5;
	  }
	}else if(n==5){                                      // Drive till line found
	  if(fwd(0,0.2,2,mission.time)){
		mission.time=-1; n = 6;
	  }
	}else if(n==6){
	  if(turnr(0.2,90.0/180*M_PI,0.1,mission.time)){
		mission.time=-1; n = 7;
	  }
	}else if(n==7){ // Cond: 0 for stopline, 1 for dist, 2 for object in front
	  if(followLineCenter(1,0.2,0,mission.time)){
		mission.time=-1; n = 8;
	  }
	}else if(n==8){
	  if(fwd(0.15,0.2,0,mission.time)){
	mission.time = -1; n = 9;
	  }
	}else if(n==9){
	  if(turn(90.0/180*M_PI,0.1,mission.time)){
	mission.time = -1; n = 10;
	  }
	}else if(n==10){ // Cond: 0 for stopline, 1 for dist, 2 for object in front
	  if(followLineCenter(2,0.1,0,mission.time)){
		mission.time=-1; n = 11;
	  }
	}else if(n==11){//stop_condition: 0=stop by dist, 1=stop by wall detection, 2=stop by line black line detection
	  if(fwd(0.20,0.2,0,mission.time)){
		mission.time=-1; n = 12;
	  }
	}else if(n==12){
	  if(followRightLine(0.4,0.2,mission.time)){
		mission.time=-1; n = 13;
	  }
	}else if(n==13){// Cond: 0 for stopline, 1 for dist, 2 for object in front
	   if(followLineCenter(5,0.2,0,mission.time)){
	       mission.time =-1; n= 14;
	   }
    }else if(n == 14){
	   mission.state=ms_gateOnTheLoose_RW;
	   n=0;
	}
    break;

  case ms_gateOnTheLoose_SIM:
	  if (n == 0) {
		  if (followLineCenter(0.75,0.2,4,mission.time)) { mission.time = -1; n = 1; }
	  }
	  else if (n == 1) {
		  if (followLineCenter(0.7,0.2,1,mission.time)) { mission.time = -1; n = 2; }
	  }
	  else if (n == 2) {
		  if (turn(90.0/180*M_PI,0.15,mission.time)) { mission.time = -1; n = 3; }
	  }
	  else if (n == 3) {
		  if (fwd(0.25,0.2,1,mission.time)) { mission.time = -1; n = 4; }
	  }
	  else if (n == 4) {
		  if (turn(-90.0/180*M_PI,0.15,mission.time)) { mission.time = -1; n = 5; }
	  }
	  else if (n == 5) {
		  if (fwd(0.15, 0.2,2, mission.time)) { mission.time = -1; n = 9; }
	  }
	  else if (n == 6) {
		  if (turn(90.0 / 180 * M_PI, 0.3, mission.time)) { mission.time = -1; n = 7; }
	  }
	  else if (n == 7) {
		  if (follow_wall(0, 15, 0.3,0, mission.time)) { mission.time = -1; n = 8; } 
	  }
	  else if (n == 8) {
		  if (fwd(50,0.3, 2, mission.time)) { mission.time = -1; n = 9; } 
	  }
	  else if (n == 9) {
		  mission.state = ms_wall_gate_SIM;
			n=0;
	  }
	  break;

	case ms_gateOnTheLoose_RW:
		
	  if (n == 0) {
		  if (followLineCenter(0.75,0.2,4,mission.time)) { mission.time = -1; n = 1; }
	  }
	  else if (n == 1) {
		  if (followLineCenter(0.75,0.2,1,mission.time)) { mission.time = -1; n = 2; }
	  }
	  else if (n == 2) {
		  if (turn(90.0/180*M_PI,0.15,mission.time)) { mission.time = -1; n = 3; }
	  }
	  else if (n == 3) {
		  if (fwd(0.15,0.2,1,mission.time)) { mission.time = -1; n = 4; }
	  }
	  else if (n == 4) {
		  if (turn(-90.0/180*M_PI,0.15,mission.time)) { mission.time = -1; n = 5; }
	  }
	  else if (n == 5) { //Drives till bl
		  if (fwd(0.15, 0.2,2, mission.time)) { mission.time = -1; n = 9; }
	  }
	  else if (n == 6) {
		  if (turn(90.0 / 180 * M_PI, 0.3, mission.time)) { mission.time = -1; n = 7; }
	  }
	  else if (n == 7) {
		  if (follow_wall(0, 15, 0.3,0, mission.time)) { mission.time = -1; n = 8; } 
	  }
	  else if (n == 8) {
		  if (fwd(50,0.3, 2, mission.time)) { mission.time = -1; n = 9; } 
	  }
	  else if (n == 9) {
		  mission.state = ms_wall_gate_RW;
			n=0;
	  }
	  break;

    case ms_wall_gate_SIM:
        if(n==0){ //Turn the same way as the line
            if(turnr(0.2,90.0/180*M_PI,0.3,mission.time)){
                mission.time =-1; n++;
            }
        } else if(n==1){  //Follow the line through the first gate, 1=distance
            if(followLineCenter(0.3, 0.2, 1, mission.time)){
                mission.time =-1; n++;
            }
        } else if(n==2){  // Turn to wall
            if(turnr(0.2,100.0/180*M_PI,0.3,mission.time)){
                mission.time =-1; n++;
            }
        } else if(n==3){ //Follow wall till inside gate, s=0=left, cond=1
            if(follow_wall(0, 0.35, 0.2, 0, mission.time)){ // Stopcon: 0 for hole in wall, 1 for object on the other side
                mission.time =-1; n++;
            } 
        }else if(n==4){ //Follow wall till inside gate, s=0=left, cond=1
            if(fwd(0.4, 0.2, 0, mission.time)){ // Stopcon: 0 for hole in wall, 1 for object on the other side
                mission.time =-1; n++;
            } 
        }else if(n==5){ //Follow wall till inside gate, s=0=left, cond=1
            if(turn(90.0 / 180 * M_PI, 0.3, mission.time)){ // Stopcon: 0 for hole in wall, 1 for object on the other side
                mission.time =-1; n++;
            } 
        } else if(n==6){
            if(fwd(0, 0.2, 2, mission.time)){  // Cond: 0 dist, 1 wall IR detect, 2 bl detect
                mission.time =-1; n++;
            }
        } else if(n==7){  
            if(turnr(0.25,90.0/180*M_PI,0.3,mission.time)){
                mission.time =-1; n++;
            }
        } else if(n==8){
            if(followLineCenter(4, 0.2, 0, mission.time)){ // Cond: 0 for stopline, 1 for dist, 2 for object in front
                mission.time =-1; n++;
            }
        } else if(n==9){
            if(turnr(0.2,90.0/180*M_PI,0.3,mission.time)){
                mission.time =-1; n++;
            }
        } else if(n==10){
            if(followLineCenter(4, 0.2, 0, mission.time)){
                mission.time =-1; n++;
            }
        } else if(n>10){
            mission.state=ms_followWhiteLine_SIM;
			n=0;
        }
    break;
    
    case ms_wall_gate_RW:
        if(n==0){ //Turn the same way as the line
            if(turnr(0.2,90.0/180*M_PI,0.1,mission.time)){
                mission.time =-1; n=1;
            }
        } else if(n==1){  //Follow the line through the first gate, 1=distance
            if(followLineCenter(0.25, 0.2, 1, mission.time)){
                mission.time =-1; n=2;
            }
        } else if(n==2){  // Turn to wall
            if(turnr(0.2,100.0/180*M_PI,0.1,mission.time)){
                mission.time =-1; n=3;
            }
        } else if(n==3){ //Follow wall till beside gate, s=0=left, cond=1
            if(follow_wall(0, 0.45, 0.3, 2, mission.time)){ // Stopcon: 2 for hole in wall, 3 for object on the other side - laser
                mission.time =-1; n=4;
            } 
		} else if(n==4){
			if(fwd(0.5, 0.2, 0, mission.time)){
				mission.time = -1; n=5;
			}
		} else if(n==5){
			if(turn(90.0*M_PI/180, 0.1, mission.time)){
				mission.time = -1; n=6;
			}
        } else if(n==6){
            if(fwd(0, 0.2, 2, mission.time)){  // Cond: 0 dist, 1 wall IR detect, 2 bl detect
                mission.time =-1; n=7;
            }
        } else if(n==7){  
            if(turnr(0.25,100.0/180*M_PI,0.1,mission.time)){
                mission.time =-1; n=8;
            }
        } else if(n==8){
            if(followLineCenter(4, 0.2, 0, mission.time)){ // Cond: 0 for stopline, 1 for dist, 2 for object in front
                mission.time =-1; n=9;
            }
		} else if(n==9){
			if(fwd(0.15, 0.2, 0, mission.time)){
				mission.time =-1; n=10;
			}
        } else if(n==10){
            if(turn(90.0/180*M_PI,0.2,mission.time)){
                mission.time =-1; n=11;
            }
        } else if(n==11){
            if(followLineCenter(4, 0.2, 0, mission.time)){
                mission.time =-1; n=12;
            }
        } else if(n>11){
            mission.state=ms_followWhiteLine_RW;
            n=0;
        }
    break;

  case ms_last_box_RW:	
  	if(n==0){	//follow black line until walldetection 2
  		if(followLineCenter(0.2, 0.1,2, mission.time)){
  			mission.time = -1;
  			n=1;
  		}	
  	}			//turn 90 degrees CCW
  	else if(n==1){
		if(turn(90.0*M_PI/180.0, 0.3,mission.time)){
			mission.time = -1;
			n++;
		}

	}
	else if(n==2){	//drive until right IR sensor detects more than 0.7m
		if(fwd(0.70,0.2,4,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==3){	
		if(fwd(0.55,0.2,0,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==4){ 
		if(turn(-90.0/180*M_PI, 0.3,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==5){ //Drive till beside box
//	printf("5, th %f\t the_r: %f\n", odo.theta, odo.theta_ref);
		if(fwd(0.40,0.2,0,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==6){
//	printf("6, th %f\t the_r: %f\n", odo.theta, odo.theta_ref);
		if(turn(-179.0/180*M_PI, 0.3,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==7){
//	printf("7, th %f\t the_r: %f\n", odo.theta, odo.theta_ref);
		if(fwd(0.5,0.2,0,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==8){
//	printf("8, th %f\t the_r: %f\n", odo.theta, odo.theta_ref);
		if(turn(90.0*M_PI/180, 0.2,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==9){
		if(fwd(0.35,0.2,2,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==10){
		if(fwd(0.2,0.2,0,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==11){
		if(turn(90.0*M_PI/180, 0.2,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==12){
		if(followLineCenter(0.2, 0.2,2, mission.time)){
  			mission.time = -1;
  			n++;
  		}
	}
	else if(n==13){
	    mission.state=ms_end;
	}
	break;


case ms_last_box_SIM:	
  	if(n==0){	//follow black line until walldetection 2
  		if(followLineCenter(0.2, 0.1,2, mission.time)){
  			mission.time = -1;
  			n++;
  		}
  	}			//turn 90 degrees CCW
  	else if(n==1){
		if(turn(90.0*M_PI/180, 0.3,mission.time)){
			mission.time = -1;
			n++;
		}

	}
	else if(n==2){
		if(fwd(0.70,0.2,3,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==3){
		if(fwd(0.40,0.2,0,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==4){
		if(turn(-90.0*M_PI/180, 0.3,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==5){
		if(fwd(0.35,0.2,0,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==6){
		if(turn(-180.0*M_PI/180.0, 0.3,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==7){
		if(fwd(0.35,0.2,0,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==8){
		if(turn(90.0*M_PI/180, 0.2,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==9){
		if(fwd(0.35,0.2,2,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==10){
		if(fwd(0.2,0.2,0,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==11){
		if(turn(90.0*M_PI/180, 0.2,mission.time)){
			mission.time = -1;
			n++;
		}
	}
	else if(n==12){
		if(followLineCenter(0.2, 0.2,2, mission.time)){
  			mission.time = -1;
  			n++;
  		}
	}
    else if(n==13){
	    mission.state=ms_end;
	}
	break;
}
  /**********************************
	* Log the data
	*/
	if (counter<SIZE_OF_ARRAY) {
	 poseTimeLog_a[counter].time=counter;
	 poseTimeLog_a[counter].x = odo.x_pos;
	 poseTimeLog_a[counter].y = odo.y_pos;
	 poseTimeLog_a[counter].theta = odo.theta;
	 counter++;
   }

/*  end of mission  */

   mot.left_pos=odo.left_pos;
   mot.right_pos=odo.right_pos;
   update_motcon(&mot);
   speedl->data[0]=100*mot.motorspeed_l;
   speedl->updated=1;
   speedr->data[0]=100*mot.motorspeed_r;
   speedr->updated=1;

   if (time  % 100 ==0)
//		printf("n = %d \n", n);
	time++;
/* stop if keyboard is activated
*
*/
  ioctl(0, FIONREAD, &arg);
  if (arg!=0)  running=0;

}/* end of main control loop */
  speedl->data[0]=0;
  speedl->updated=1;
  speedr->data[0]=0;
  speedr->updated=1;
  rhdSync();
  rhdDisconnect();
  exit(0);
}


/*
* Routines to convert encoder values to positions.
* Encoder steps have to be converted to meters, and
* roll-over has to be detected and corrected.
*/


void reset_odo(odotype * p)
{
	p->right_pos = p->left_pos = p->x_pos = p->y_pos = p->theta = 0.0;
	p->right_enc_old = p->right_enc;
	p->left_enc_old = p->left_enc;
	p->theta_ref = 0;
}

void update_odo(odotype *p)
{
	int delta_l, delta_r;
	double delta_u, delta_theta;

	delta_r = p->right_enc - p->right_enc_old;
	if (delta_r > 0x8000) delta_r -= 0x10000;
	else if (delta_r < -0x8000) delta_r += 0x10000;
	p->right_enc_old = p->right_enc;
	p->right_pos += delta_r * p->cr;
	//printf("delta_l = %d ",delta_l);
	delta_l = p->left_enc - p->left_enc_old;
	
	if (delta_l > 0x8000) delta_l -= 0x10000;
	else if (delta_l < -0x8000) delta_l += 0x10000;
	p->left_enc_old = p->left_enc;
	p->left_pos += delta_l * p->cl;

	delta_u = ((double)delta_r * p->cr + (double)delta_l * p->cl)/2;
	delta_theta = ((double)delta_r * p->cr - (double)delta_l * p->cl)/(double)p->w;
	//printf("delta_theta = %f\t delta_r = %d, delta_l = %d\n",delta_theta,delta_r,delta_l);
	p->theta += delta_theta;
	p->x_pos += delta_u*cos(p->theta);
	p->y_pos -= delta_u*sin(p->theta);

}


void update_motcon(motiontype *p){
	double d, d_angle,pid;
    double IR_dist[5];
    double delta_l, delta_r;
	if (p->cmd !=0){
		p->finished=0;
		p->error_sum=0;
		p->error_old=0;
		switch (p->cmd){
			case mot_stop:
				p->curcmd=mot_stop;
			break;

			case mot_move:
			    p->startpos=(p->left_pos+p->right_pos)/2;
			    if(p->speedcmd < 0){
				p->curcmd=mot_reverse;
			    }
			    else{
				p->curcmd=mot_move;
			    }
			break;

			case mot_turn:
			    if (p->angle > 0)
				p->startpos=p->right_pos;
			    else
				p->startpos=p->left_pos;
			    p->curcmd=mot_turn;
			break;

			case mot_turnr:
			    if (p->angle > 0)
				p->startpos=p->right_pos;
			    else
				p->startpos=p->left_pos;
						    p->curcmd=mot_turnr;
			break;

			case mot_followLineCenter:
			    p->startpos=(p->left_pos+p->right_pos)/2;
			    p->curcmd=mot_followLineCenter;
			break;

			case mot_followWhiteLine:
			    p->startpos=(p->left_pos+p->right_pos)/2;
			    p->curcmd=mot_followWhiteLine;
			break;

			case mot_follow_wall_left:
			    p->curcmd=mot_follow_wall_left;
			break;

			case mot_follow_wall_right:
			    p->curcmd=mot_follow_wall_right;
			break;

			case mot_followRightLine:
			    p->curcmd=mot_followRightLine;
			    p->startpos=(p->left_pos+p->right_pos)/2;
			break;

			case mot_followLeftLine:
			    p->curcmd=mot_followLeftLine;
			    p->startpos=(p->left_pos+p->right_pos)/2;
			break;

			case mot_driveTowardsCenterOfGate:
				p->curcmd = mot_driveTowardsCenterOfGate;
				break;

			case mot_reverseTillBlackLine:
				p->curcmd = mot_reverseTillBlackLine;
				break;

			case mot_followLineCenterTwoGatePolesDetected:
				p->curcmd = mot_followLineCenterTwoGatePolesDetected;
				numberOfGatePolesDetected = 0;
				p->laser_old=0;
				break;
   		}
   		p->cmd=0;
 	}

	switch (p->curcmd){
		case mot_stop:
			p->motorspeed_l=0;
			p->motorspeed_r=0;
		break;

		//stop_condition: 0=stop by dist, 1=stop by wall detection, 2=stop by line black line detection
		case mot_move:
			d=((p->motorspeed_l+p->motorspeed_r)/2)*((p->motorspeed_l+p->motorspeed_r)/2)/(2*(AJAX));
			if(p->stop_condition==0){
				if ((p->right_pos+p->left_pos)/2- p->startpos >= p->dist){
					p->finished=1;
					p->motorspeed_l=0;
					p->motorspeed_r=0;
	  			}
	  			else if((p->right_pos+p->left_pos)/2- p->startpos > p->dist - d){ 	// Deacceleration
					p->motorspeed_l -= AJAX*CONVERSION_FACTOR_ACC - K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
					p->motorspeed_r -= AJAX*CONVERSION_FACTOR_ACC + K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
		  		}
		  		else if(p->motorspeed_l<p->speedcmd){                           			// Acceleration
					p->motorspeed_l += AJAX*CONVERSION_FACTOR_ACC - K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
					p->motorspeed_r += AJAX*CONVERSION_FACTOR_ACC + K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
		  		}
		  		else {
					p->motorspeed_l=p->speedcmd - K_FOR_STRAIGHT_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
					p->motorspeed_r=p->speedcmd + K_FOR_STRAIGHT_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
		  		}
			}
			else if(p->stop_condition==1){
				if (minDistFrontIR()<=p->dist && minDistFrontIR() > 0){
					p->finished=1;
					p->motorspeed_l=0;
					p->motorspeed_r=0;
	  			}
	  			else if(minDistFrontIR() < p->dist + d){ 	// Deacceleration
					p->motorspeed_l -= AJAX*CONVERSION_FACTOR_ACC - K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
					p->motorspeed_r -= AJAX*CONVERSION_FACTOR_ACC + K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
		  		}
		  		else if(p->motorspeed_l<p->speedcmd){                           			// Acceleration
					p->motorspeed_l += AJAX*CONVERSION_FACTOR_ACC - K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
					p->motorspeed_r += AJAX*CONVERSION_FACTOR_ACC + K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
		  		}
		  		else {
					p->motorspeed_l=p->speedcmd - K_FOR_STRAIGHT_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
					p->motorspeed_r=p->speedcmd + K_FOR_STRAIGHT_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
		  		}
			}
			else if(p->stop_condition==2 && !detectLine() ){
				p->motorspeed_l=p->speedcmd;
				p->motorspeed_r=p->speedcmd;
			}
			else if(p->stop_condition==3 && getDistIR(IR_dist)[4] < p->dist && getDistIR(IR_dist)[4] > 0){
				p->motorspeed_l=p->speedcmd;
				p->motorspeed_r=p->speedcmd;
			}
			else if(p->stop_condition==4 && (laserpar[8] < p->dist || (laserpar[8] == 0.0))){
				p->motorspeed_l=p->speedcmd;
				p->motorspeed_r=p->speedcmd;
			}
			else if (p->stop_condition==5) {
				if (laserpar[8] > START_POS_TO_BOX_MAX_DIST && laserOld8 < START_POS_TO_BOX_MAX_DIST && laserOld8 != 0) {//Hits outer edge of box
					p->finished = 1;
					p->motorspeed_l = 0;
					p->motorspeed_r = 0;
					laserOld8 = 0;
				}
				else if (laserpar[8] < START_POS_TO_BOX_MAX_DIST && laserpar[8] != 0 && laserOld8 > START_POS_TO_BOX_MAX_DIST)//Hits first edge of box
				{
					p->motorspeed_l = p->speedcmd - K_FOR_STRAIGHT_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
					p->motorspeed_r = p->speedcmd + K_FOR_STRAIGHT_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
				}
				else if (laserpar[8] > START_POS_TO_BOX_MAX_DIST && laserpar[8] != 0 && laserOld8 > START_POS_TO_BOX_MAX_DIST) {//Both are before the edge
					p->motorspeed_l = p->speedcmd - K_FOR_STRAIGHT_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
					p->motorspeed_r = p->speedcmd + K_FOR_STRAIGHT_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
				}
				else if (laserpar[8] < START_POS_TO_BOX_MAX_DIST && laserpar[8] != 0 && laserOld8 < START_POS_TO_BOX_MAX_DIST && laserOld8 != 0) {//at the box.
					p->motorspeed_l = p->speedcmd - K_FOR_STRAIGHT_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
					p->motorspeed_r = p->speedcmd + K_FOR_STRAIGHT_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
					if (drivePastBoxCounter == LASER8VALUES - 1) drivePastBoxCounter = 0;
					laser8Values[drivePastBoxCounter] = laserpar[8];
					drivePastBoxCounter++;
				}
				laserOld8 = laserpar[8];
			}
			else{
				p->finished=1;
				p->motorspeed_l=0;
				p->motorspeed_r=0;
			}
		break;

		case mot_reverse:
			d=((p->motorspeed_l+p->motorspeed_r)/2)*((p->motorspeed_l+p->motorspeed_r)/2)/(2*(AJAX));
		 	if ((p->right_pos+p->left_pos)/2- p->startpos <= -(p->dist)){
				p->finished=1;
				p->motorspeed_l=0;
				p->motorspeed_r=0;
		  	}
		  	else if((p->right_pos+p->left_pos)/2- p->startpos < -(p->dist-d)){ // Deacceleration
				p->motorspeed_l += AJAX*CONVERSION_FACTOR_ACC - K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
				p->motorspeed_r += AJAX*CONVERSION_FACTOR_ACC + K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
		  	}
		  	else if(p->motorspeed_l>p->speedcmd){                           // Acceleration
				p->motorspeed_l -= AJAX*CONVERSION_FACTOR_ACC - K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
				p->motorspeed_r -= AJAX*CONVERSION_FACTOR_ACC + K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
		  	}
		  	else {
				p->motorspeed_l=p->speedcmd - K_FOR_STRAIGHT_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
				p->motorspeed_r=p->speedcmd + K_FOR_STRAIGHT_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
		  	}
		break;
    
		case mot_detect_line:
			if(!(detectLine())){
				p->motorspeed_l=p->speedcmd;
				p->motorspeed_r=p->speedcmd;
			}
			else{
				p->finished=1;
				p->motorspeed_l=0;
				p->motorspeed_r=0;
			}
		break;

		case mot_turn:
			if (p->angle>0){ 		//Turn left
				d_angle=p->motorspeed_r*p->motorspeed_r/(AJAX*p->w);
				if(p->right_pos-p->startpos >= p->angle*p->w/2){
			  		p->motorspeed_l=0;
			  		p->motorspeed_r=0;
			  		p->finished=1;
				}
				else if(p->right_pos-p->startpos > p->angle*p->w/2-d_angle*p->w/2){
			  		p->motorspeed_l += AJAX*CONVERSION_FACTOR_ACC;
			  		p->motorspeed_r -= AJAX*CONVERSION_FACTOR_ACC;
				}
				else if(p->motorspeed_r < p->speedcmd){
			  		p->motorspeed_l -= AJAX*CONVERSION_FACTOR_ACC;
			  		p->motorspeed_r += AJAX*CONVERSION_FACTOR_ACC;
				}
				else{									//(p->right_pos-p->startpos < p->angle*p->w/2)
			  		p->motorspeed_r=p->speedcmd;
			  		p->motorspeed_l=-(p->speedcmd);
				}
	  		}
	  		else { 				//Turn right
				d_angle=p->motorspeed_l*p->motorspeed_l/(AJAX*p->w);
				if(p->left_pos - p->startpos >= - p->angle*p->w/2){
			  		p->motorspeed_l=0;
			  		p->motorspeed_r=0;
			  		p->finished=1;
				}
				else if(p->left_pos-p->startpos > -p->angle*p->w/2-d_angle*p->w/2){
			  		p->motorspeed_r += AJAX*CONVERSION_FACTOR_ACC;
			  		p->motorspeed_l -= AJAX*CONVERSION_FACTOR_ACC;
				}
				else if(p->motorspeed_l < p->speedcmd){
			  		p->motorspeed_r -= AJAX*CONVERSION_FACTOR_ACC;
			  		p->motorspeed_l += AJAX*CONVERSION_FACTOR_ACC;
				}
				else{			//(p->left_pos-p->startpos < - p->angle*p->w/2)
			  		p->motorspeed_l=p->speedcmd;
			  		p->motorspeed_r=-(p->speedcmd);
				}
		  	}
		break;

		case mot_turnr:
			if (p->angle>0){ 		//Turn left
	 			delta_r = p->dist/(p->dist-p->w/2);
		 		delta_l = p->dist/(p->dist+p->w/2);
		 		
		 		if(p->right_pos-p->startpos >= p->angle*(p->dist+p->w/2)){
			   		p->motorspeed_l=0;
			   		p->motorspeed_r=0;
			   		p->finished=1;
			 	}
			 	else if (p->right_pos-p->startpos < p->angle*(p->dist+p->w/2)){
			   		p->motorspeed_l =  p->speedcmd*delta_l;
			   		p->motorspeed_r =  p->speedcmd*delta_r;
			 	}
		   	}
		   	else {
			 	delta_r = p->dist/(p->dist+p->w/2);
			 	delta_l = p->dist/(p->dist-p->w/2);

			 	if(p->left_pos-p->startpos >= -(p->angle*(p->dist+p->w/2))){
			   		p->motorspeed_l=0;
			   		p->motorspeed_r=0;
			   		p->finished=1;
			 	}
			 	else if (p->left_pos-p->startpos < -( p->angle*(p->dist+p->w/2))){
			   		p->motorspeed_l =  p->speedcmd*delta_l;
			   		p->motorspeed_r =  p->speedcmd*delta_r;
			 	}
		   	}
		break;

		case mot_followLineCenter:
			//stop_condition: 0=stop by stop line detection, 1=stop by defined distance p->dist, 2=stop by IR obstacle detection, 3=stop by LASER
			p->error_old = p->error_current;
			if(IS_SIMULATION==1){
			    p->error_current = centerOfGravity(0)-3.5;
			}
			else{
		        p->error_current = minIntensity()-3.5;
			}
			p->error_sum += p->error_current;
			pid = KP_FOR_FOLLOWLINE*p->error_current+KI_FOR_FOLLOWLINE*p->error_sum+KD_FOR_FOLLOWLINE*(p->error_current-p->error_old);
			if(	p->stop_condition==0 && !stopLine()){
				p->motorspeed_l = p->speedcmd - pid;
				p->motorspeed_r = p->speedcmd + pid;
			}
			else if(p->stop_condition==1 && (p->left_pos+p->right_pos)/2 - p->startpos < p->dist){
				p->motorspeed_l = p->speedcmd - pid;
				p->motorspeed_r = p->speedcmd + pid;
			}
			else if(p->stop_condition==2 && (minDistFrontIR() > p->dist || minDistFrontIR()<0)){
				p->motorspeed_l = p->speedcmd - pid;
				p->motorspeed_r = p->speedcmd + pid;
			}
            else if(p->stop_condition==3 && laserpar[4] > 0.2){
                p->motorspeed_l = p->speedcmd - pid;
				p->motorspeed_r = p->speedcmd + pid;
            }
			else if(p->stop_condition==4 && (laserpar[0] > p->dist || laserpar[0]==0)){		//left laser detects obstacle
                p->motorspeed_l = p->speedcmd - pid;
				p->motorspeed_r = p->speedcmd + pid;
            }
			else{
				p->motorspeed_l = 0;
				p->motorspeed_r = 0;
				p->finished = 1;
			}
		break;

		//must not be used with stopLine() as stopcondition
		case mot_followRightLine:
		    if (p->right_pos - p->startpos < p->dist) {
		        p->motorspeed_l = p->speedcmd - K_FOR_FOLLOWLINE*(rightMostPosSlope() - 2.5);
		        p->motorspeed_r = p->speedcmd  + K_FOR_FOLLOWLINE*(rightMostPosSlope() - 2.5);
			}
			else{
				p->motorspeed_l = 0;
				p->motorspeed_r = 0;
				p->finished = 1;
			}
		break;
		
		case mot_followWhiteLine:

		  if(!stopLineNazi()){
			//(p->left_pos+p->left_pos)/2 - p->startpos < p->dist
			    p->motorspeed_l = p->speedcmd - K_FOR_FOLLOWLINE*(maxIntensity() - 3.5);
		        p->motorspeed_r = p->speedcmd + K_FOR_FOLLOWLINE*(maxIntensity() - 3.5);
		  }
		  else {
		    p->motorspeed_l = 0;
		    p->motorspeed_r = 0;
		    p->finished = 1;
		  }
		break;

		case mot_follow_wall_left:                      // 0 is the leftmost IR sensor
			p->error_old = p->error_current;
            if (p->stop_condition < 2){ p->error_current = getDistIR(IR_dist)[0] - p->dist; 
                } else { p->error_current = laserpar[0] - p->dist;}
			p->error_sum += p->error_current;
			pid = KP_FOR_FOLLOWWALL*p->error_current+KI_FOR_FOLLOWWALL*p->error_sum+KD_FOR_FOLLOWWALL*(p->error_current-p->error_old);

			if(p->stop_condition==0 && getDistIR(IR_dist)[0] < 0.70){  // Stopcon: 0 for hole in wall, 1 for object on the other side, 2 for 0 but laser		
		        p->motorspeed_l=p->speedcmd - pid;
		        p->motorspeed_r=p->speedcmd + pid;
			} else if(p->stop_condition==1 && (getDistIR(IR_dist)[4] > 0.20 || getDistIR(IR_dist)[4] < 0)){  // stopcon: 0 for hole in wall, 1 for object on the other side, 2 for 0 but laser
			    if(pid > p->speedcmd){                      // Speedlimit
			        pid = p->speedcmd;
			    }
		        p->motorspeed_l=p->speedcmd - pid;
		        p->motorspeed_r=p->speedcmd + pid;
			} else if(p->stop_condition==2 && (laserpar[0] < 1.5*p->dist || laserpar[0] == 0)){   // stopcon: 0 for hole in wall, 1 for object on the other side, 2 for 0 but laser
                p->motorspeed_l=p->speedcmd - pid;
                p->motorspeed_r=p->speedcmd + pid;
            } else if(p->stop_condition==3 && (laserpar[8] > 0.30 || laserpar[0] == 0)){
                if(pid > p->speedcmd){                      // Speedlimit
                    pid = p->speedcmd;
                }
                p->motorspeed_l=p->speedcmd - pid;
                p->motorspeed_r=p->speedcmd + pid;
            } else{
                p->motorspeed_l = 0;
                p->motorspeed_r = 0;
                p->finished = 1;
            }
		break;

		//follows wall using the right IR sensor or laser until no wall is detected anymore
		case mot_follow_wall_right:   
			p->error_old = p->error_current;
            if (p->stop_condition < 2){ p->error_current = getDistIR(IR_dist)[4] - p->dist;
                } else { p->error_current = laserpar[8] - p->dist;}
			p->error_sum += p->error_current;
			pid = KP_FOR_FOLLOWWALL*p->error_current+KI_FOR_FOLLOWWALL*p->error_sum+KD_FOR_FOLLOWWALL*(p->error_current-p->error_old);

			if(p->stop_condition==0 && getDistIR(IR_dist)[4] < 0.70){                     // stopcon: 0 for hole in wall, 1 for object on the other side
		        p->motorspeed_l=p->speedcmd + pid;
		        p->motorspeed_r=p->speedcmd - pid;
			} 
			else if(p->stop_condition==1 && (getDistIR(IR_dist)[0] > 0.20 || getDistIR(IR_dist)[0] < 0)){     // stopcon: 0 for hole in wall, 1 for object on the other side
			    if(pid > p->speedcmd){                      //Speedlimit
			        pid = p->speedcmd;
			    }
		        p->motorspeed_l=p->speedcmd + pid;
		        p->motorspeed_r=p->speedcmd - pid;
            } else if(p->stop_condition==2 && (laserpar[8] < 1.5*p->dist || laserpar[0] == 0)){ // stopcon: 0 for hole in wall, 1 for object on the other side, 2 for 0 but laser
                p->motorspeed_l=p->speedcmd + pid;
                p->motorspeed_r=p->speedcmd - pid;
            } else if(p->stop_condition==3 && (laserpar[0] > 0.30 || laserpar[0] == 0)){
                if(pid > p->speedcmd){                      // Speedlimit
                    pid = p->speedcmd;
                }
                p->motorspeed_l=p->speedcmd + pid;
                p->motorspeed_r=p->speedcmd - pid;
			} else{
			        p->motorspeed_l = 0;
			        p->motorspeed_r = 0;
			        p->finished = 1;
			}
		break;

		//follows black line on the left side until a stop line is detected
		//must not be used with stopLine() as stopcondition
		case mot_followLeftLine:
			if (p->right_pos - p->startpos < p->dist) {
				p->motorspeed_l = p->speedcmd - KP_FOR_FOLLOWLINE*(leftMostNegSlope() - 4.5);
				p->motorspeed_r = p->speedcmd + KP_FOR_FOLLOWLINE*(leftMostNegSlope() - 4.5);
			}
			else {
				p->motorspeed_l = 0;
				p->motorspeed_r = 0;
				p->finished = 1;
			}
			break;

		//states used in gateOnTheLoose:
		case mot_followLineCenterTwoGatePolesDetected:
			if (numberOfGatePolesDetected == 2 && laserpar[0]>0.75 && p->laser_old<0.75) { //second gate pole detection
				p->motorspeed_l = 0;
				p->motorspeed_r = 0;
				p->finished = 1;
			} else if (p->laser_old >= 0.75 && laserpar[0] < 0.75 && laserpar[0] != 0) { //first gate pole detection
				numberOfGatePolesDetected++;
				p->motorspeed_l = p->speedcmd - K_FOR_FOLLOWLINE*(minIntensity() - 3.5);
				p->motorspeed_r = p->speedcmd + K_FOR_FOLLOWLINE*(minIntensity() - 3.5);

			} else {
				p->motorspeed_l = p->speedcmd - K_FOR_FOLLOWLINE*(minIntensity() - 3.5);
				p->motorspeed_r = p->speedcmd + K_FOR_FOLLOWLINE*(minIntensity() - 3.5);
			}
			if (laserpar[0]!=0) p->laser_old = laserpar[0];
			
			break;
			
		case mot_reverseTillBlackLine:
			d = ((p->motorspeed_l + p->motorspeed_r) / 2)*((p->motorspeed_l + p->motorspeed_r) / 2) / (2 * (AJAX));
			double input[NUMBER_OF_IRSENSORS];
			getTransformedIRData(input);
			if ((p->right_pos + p->left_pos) / 2 - p->startpos <= -(p->dist)) {
				p->finished = 1;
				p->motorspeed_l = 0;
				p->motorspeed_r = 0;
			}
			else if ((p->right_pos + p->left_pos) / 2 - p->startpos < -(p->dist - d) && (!(input[0]>CRITICAL_BLACK_VALUE) || !(input[7]>CRITICAL_BLACK_VALUE))) { // Deacceleration
				p->motorspeed_l += AJAX*CONVERSION_FACTOR_ACC - K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
				p->motorspeed_r += AJAX*CONVERSION_FACTOR_ACC + K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
			}
			else if (p->motorspeed_l>p->speedcmd && (!(input[0]>CRITICAL_BLACK_VALUE) || !(input[7]>CRITICAL_BLACK_VALUE))) {                           // Acceleration
				p->motorspeed_l -= AJAX*CONVERSION_FACTOR_ACC - K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
				p->motorspeed_r -= AJAX*CONVERSION_FACTOR_ACC + K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
			}
			else if ((p->right_pos + p->left_pos) / 2 - p->startpos < -(p->dist - d) && ((input[0] > CRITICAL_BLACK_VALUE) || (input[7] > CRITICAL_BLACK_VALUE))) {
				if (input[0] > CRITICAL_BLACK_VALUE && !(input[7] > CRITICAL_BLACK_VALUE)) {
					p->motorspeed_r = 0;
					p->motorspeed_l += AJAX*CONVERSION_FACTOR_ACC - K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
				}
				else if (!(input[0] > CRITICAL_BLACK_VALUE) && input[7] > CRITICAL_BLACK_VALUE) {
					p->motorspeed_r += AJAX*CONVERSION_FACTOR_ACC + K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
					p->motorspeed_l = 0;
				}
			}
			else if (input[0] > CRITICAL_BLACK_VALUE && input[7] > CRITICAL_BLACK_VALUE) {
				p->finished = 1;
				p->motorspeed_l = 0;
				p->motorspeed_r = 0;
			}
			else {
				p->motorspeed_l = p->speedcmd - K_FOR_STRAIGHT_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
				p->motorspeed_r = p->speedcmd + K_FOR_STRAIGHT_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
			}
			break;

		case mot_turnTowardsCenterOfGate:
			if (centerOfGateWithLaserScanner() != 0) {//turn robot if centerOfGateWithLaserScanner() is anything other then 0:
				p->motorspeed_l -= p->speedcmd + AJAX*CONVERSION_FACTOR_ACC*centerOfGateWithLaserScanner();
				p->motorspeed_l += p->speedcmd + AJAX*CONVERSION_FACTOR_ACC*centerOfGateWithLaserScanner();
			}
			else {
				p->motorspeed_l = 0;
				p->motorspeed_r = 0;
				p->finished = 1;
			}
			break;

		case mot_driveTowardsCenterOfGate:
			if (!(laserpar[0] < 30 && laserpar[0] != 0 && laserpar[LASERSCANNER_ZONES - 1] < 30 && laserpar[LASERSCANNER_ZONES - 1] != 0)) {
				p->motorspeed_l = p->speedcmd - K_FOR_DRIVE_TOWARDS_CENTER_OF_GATE*(centerOfGateWithLaserScanner());
				p->motorspeed_r = p->speedcmd + K_FOR_DRIVE_TOWARDS_CENTER_OF_GATE*(centerOfGateWithLaserScanner());
			}
			else {
				p->motorspeed_l = 0;
				p->motorspeed_r = 0;
				p->finished = 1;
			}
			break;
			
		
		//End of states used in gateOnTheLoose:
	}
}

// stop condition: 0 for dist, 1 for wall detection, 2 for 
int fwd(double dist, double speed, int condition,int time){        
	if (time==0){
		mot.cmd=mot_move;
		mot.speedcmd=speed;
		mot.dist=dist;
        mot.stop_condition = condition;
		return 0;
	}
	else
		return mot.finished;
}

int turn(double angle, double speed,int time){
 	if (time==0){
		mot.cmd=mot_turn;
		mot.speedcmd=speed;
		mot.angle=angle;
		mot.start_angle=odo.theta;
		odo.theta_ref=odo.theta_ref + angle; //Update the desired angle
		return 0;
	}
	else
		return mot.finished;
}
int turnr(double radius, double angle, double speed, int time){
	if(time == 0){
   		mot.cmd = mot_turnr;
   		mot.speedcmd = speed;
   		mot.dist = radius;
   		mot.angle = angle;
		odo.theta_ref=odo.theta_ref + angle; //Update the desired angle
   	return 0;
 	}
 	else
   		return mot.finished;
}
int followLineCenter(double dist, double speed,int condition, int time){   // linesensor input???
  if(time == 0){
    mot.cmd = mot_followLineCenter;
    mot.speedcmd = speed;
    mot.dist = dist;
    mot.stop_condition = condition;
    return 0;
    } else { 
		odo.theta_ref=odo.theta; 
		return mot.finished;
	}
}
int followWhiteLine(double dist, double speed, int time){   // linesensor input???
	if(time == 0){
		mot.cmd = mot_followWhiteLine;
		mot.speedcmd = speed;
		mot.dist = dist;
		return 0;
	}
	else {
        odo.theta_ref=odo.theta; 
		return mot.finished;
}
}
int follow_wall(int side, double dist, double speed, int condition, int time){  //Side = 0 = left   Side = 1 = right
  if(time == 0){
	mot.speedcmd = speed;
	mot.dist = dist;
	if (side == 0){
	  mot.cmd = mot_follow_wall_left;
	} else if (side == 1){
	  mot.cmd = mot_follow_wall_right;
	}
    mot.stop_condition = condition;                 // Cond: 0 for hole in wall, 1 for object on opposite side
	return 0;
  }
  else{
	odo.theta_ref=odo.theta;
	return mot.finished;
	}
}
int followRightLine(double dist, double speed, int time){   // linesensor input???
  if(time == 0){
	 mot.cmd = mot_followRightLine;
	 mot.speedcmd = speed;
	 mot.dist = dist;
	 return 0;
 }
 else{ 
	odo.theta_ref=odo.theta;
	return mot.finished;
	}
}

int followLeftLine(double dist, double speed, int time){   // linesensor input???
	if(time == 0){
		mot.cmd = mot_followLeftLine;
		mot.speedcmd = speed;
		mot.dist = dist;
		return 0;
	} else { 
	odo.theta_ref=odo.theta;
	return mot.finished;
	}
}

void sm_update(smtype *p){
	if (p->state!=p->oldstate){
		p->time=0;
		p->oldstate=p->state;
	}
	else {
		p->time++;
	}
}

int log_data_to_file(poseTimeLog_t * poseTimeLog_out, int size){
  // Takes array of custom log-type and the length of this, writes to the file: Log.dat
  FILE *outFile;
  int i;
  double input[NUMBER_OF_IRSENSORS];
	getTransformedIRData(input);
  printf("\nStarting logging of data\n");
  outFile = fopen("Log.dat", "w");
  if (outFile == NULL) {
	fprintf(stderr, "ERROR: couldn't open %s for writing\n", "Log.dat");
	return 1;
	}
	for(i = 0;i < size; i++) {
		fprintf(outFile,"%i,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",poseTimeLog_out[i].time, 
poseTimeLog_out[i].x, poseTimeLog_out[i].y, poseTimeLog_out[i].theta,
input[7],input[6],input[5],input[4],input[3],input[2],input[1],input[0]);
	}
	fclose(outFile);
	printf("Ended logging of data\n\n");
	return 0;
}

//transforms raw IR data to a number between 0 and 1
void getTransformedIRData(double* output) {
	int i;
	double black_mean[] = { 45.4906,46.1698,46.0286,46.3113,46.0849,46.2453,46.1321,48.5189 };
	double scale[] = { 0.0365,0.0313,0.0297,0.0272,0.084,0.0287,0.0312,0.0367 };
	if(IS_SIMULATION==1){
	   for(i=0;i<NUMBER_OF_IRSENSORS;i++){
	   black_mean[i] = 85;
	   scale[i] = 0.00588;
	   }
	}
	for (i = 0; i < NUMBER_OF_IRSENSORS; i++) {
		output[i] = 1 - scale[i] * (linesensor->data[i] - black_mean[i]);
	}
}

//finds IR sensor with minimum intensity
int minIntensity(){
  int i, index = 0;
  int min;
  min = (int)linesensor->data[0];
  for(i = 1; i < NUMBER_OF_IRSENSORS; i++){
	if(linesensor->data[i]< min){
	  min = linesensor->data[i];
	  index = i;
	}
  }
  return index;
}
 int maxIntensity(){
  int i, index = 0;
  int max;
  max = (int)linesensor->data[0];
  for(i = 1; i < NUMBER_OF_IRSENSORS; i++){
    if(linesensor->data[i]> max){
      max = linesensor->data[i];
      index = i;
    }
  }
return index;
}

//finds black line with IR sensors using the center of gravity algorithm
//color: 0=black, 1=white
double centerOfGravity(char color){
  // Input is raw data from linesensors. Between each photoLED exist one "i";
	double sumI = 0, sumXI=0;
	int i;
	double input[NUMBER_OF_IRSENSORS];
	getTransformedIRData(input);
	if(color==0){

	}
	/*for(i = 0; i< NUMBER_OF_IRSENSORS; i++){
		input[i] = color==0 ? 1-input[i] : input[i];    // 0 is black, everything else is white.
	}*/
	for(i=0; i < NUMBER_OF_IRSENSORS; i++){
		sumI += input[i];
		sumXI += i*input[i];
	}
	return sumXI/sumI;
  }

//We follow the convention of the x-axis given as the numbers of the IR-sensors 0 to 7. The LEFT-most IR-sensor is no. 7 and the RIGHT-most IR-sensor is no. 0.!!!!!!!
double minDistFrontIR(){
	int i;
	double dist[5];
	double min_dist;

	getDistIR(dist);
	min_dist = dist[1];
	for (i=1; i < 3; i++){
		if(min_dist >  dist[i]){
			min_dist = dist[i];
		}
	}
	return min_dist;
}

double* getDistIR(double* dist){
	int i;
	for(i=0; i<5; i++){
		dist[i] = Ka_IR[i]/(irsensor->data[i] - Kb_IR[i])/100.0;
	}
	return dist;
}
//finds the rightmost slope of the IR data
double rightMostNegSlope() {
	// Returns a number from 0 to 7, which indicates the position at which the right most positive slope begins.
	int i;
	double input[NUMBER_OF_IRSENSORS], a;
	getTransformedIRData(input);
	for (i = 1; i<NUMBER_OF_IRSENSORS; i++) {//right most negative slope.
		if (input[i]<CRITICAL_IR_VALUE && input[i - 1] >= CRITICAL_IR_VALUE) {
			a = input[i] - input[i - 1];
			return (CRITICAL_IR_VALUE - (double)input[i-1]) / (double)a + (double)(i-1);
		}
	}
	return -1;
}

double rightMostPosSlope(){
	// Returns a number from 0 to 7, which indicates the position at which the right most negative slope begins.
	int i;
	double input[NUMBER_OF_IRSENSORS], a;
	getTransformedIRData(input);
	for(i=1;i<NUMBER_OF_IRSENSORS;i++){//right most negative slope.
	  if (input[i]>CRITICAL_IR_VALUE && input[i-1] <= CRITICAL_IR_VALUE) {
		a = input[i]-input[i-1];
		return (CRITICAL_IR_VALUE -(double)input[i-1])/(double)a+(double)(i-1);
	  }
	}
return -1;
}

//finds the left most slope of the IR data:
double leftMostNegSlope() {
	// Returns a number from 0 to 7, which indicates the position at which the right most positive slope begins.
	int i;
	double input[NUMBER_OF_IRSENSORS], a;
	getTransformedIRData(input);
	for (i = NUMBER_OF_IRSENSORS-1; i>=1; i--) {//right most negative slope.
		if (input[i]<CRITICAL_IR_VALUE && input[i - 1] >= CRITICAL_IR_VALUE) {
			a = input[i] - input[i-1];
			return (CRITICAL_IR_VALUE - (double)input[i-1]) / (double)a + (double)(i-1);
		}
	}
	return -1;
}
double leftMostPosSlope(){
	// Returns a number from 0 to 7, which indicates the position at which the right most negative slope begins.
	int i;
	double input[NUMBER_OF_IRSENSORS], a;
	getTransformedIRData(input);
	for(i=NUMBER_OF_IRSENSORS-1;i>=1;i++){//right most negative slope.
	  if (input[i]>CRITICAL_IR_VALUE && input[i-1] <= CRITICAL_IR_VALUE) {
		a = input[i]-input[i-1];
		return (CRITICAL_IR_VALUE -(double)input[i-1])/(double)a+(double)(i-1);
	  }
	}
return -1;
}


//checks if the IR sensors detect a stop line.
//return values: 1=line detected, 0=no line detected
char stopLine(){
	int i,count=0;
	double input[NUMBER_OF_IRSENSORS];
	getTransformedIRData(input);
	for(i=0;i<NUMBER_OF_IRSENSORS;i++){
		if(input[i]>CRITICAL_BLACK_VALUE){
			count++;
		}
	}
	return count>CRIT_NR_BLACK_LINE ? 1 : 0;
}

char stopLineNazi(){
	int i,count=0;
	double input[NUMBER_OF_IRSENSORS];
	getTransformedIRData(input);
	for(i=0;i<NUMBER_OF_IRSENSORS;i++){
		if(input[i]>=0.4){
			count++;
		}
	}
	return count>=NUMBER_OF_IRSENSORS ? 1 : 0;
}

char detectLine(){
	int i,count=0;
	double input[NUMBER_OF_IRSENSORS];
	getTransformedIRData(input);
	for(i=0;i<NUMBER_OF_IRSENSORS;i++){
		if(input[i]>CRITICAL_BLACK_VALUE){
			count++;
		}
	}
	return count>2 ? 1 : 0;
}


//Functions used in gateOnTheLoose:
//Calculation functions:
int followLineCenterTwoGatePolesDetected(double dist, double speed, int time) {
	if (time == 0) {
		mot.cmd = mot_followLineCenterTwoGatePolesDetected;
		mot.speedcmd = speed;
		mot.dist = dist;
		return 0;
	} else {
	odo.theta_ref=odo.theta;
	return mot.finished;
	}
}

int reverseTillBlackLine(double dist, double speed, int time) {
	if (time == 0) {
		mot.cmd = mot_reverseTillBlackLine;
		mot.speedcmd = speed;
		mot.dist = dist;
		return 0;
	}
	else {
		odo.theta_ref=odo.theta;
		return mot.finished;
	}
}

int turnTowardsCenterOfGate(double speed, int time) {
	if (time == 0) {
		mot.cmd = mot_turnTowardsCenterOfGate;
		mot.speedcmd = speed;
		return 0;
	} else {
		odo.theta_ref=odo.theta;
		return mot.finished;
	}
}
int driveTowardsCenterOfGate(double speed, int time) {
	if (time == 0) {
		mot.cmd = mot_driveTowardsCenterOfGate;
		mot.speedcmd = speed;
		return 0;
	}
	else
		return mot.finished;
}
double centerOfGateWithLaserScanner() {
	int l, r;
	l = leftMostPillar();
	r = rightMostPillar();
	return 8 - (l + r);
	//Returning 0 means it's perfectly aligned. 
	//If the number is negative the robot is turned too far to the left in relation to the center of the gate.
	//If the number is positive the robot is turned too far to the right in relation to the center of the gate.
}
double distanceToCenterOfGate() {
	int l, r;
	l = leftMostPillar();
	r = rightMostPillar();

	//The highter the distance, the higher is the returnvalue:
	return (r - l);
}
int leftMostPillar() {
	int i;
	for (i = 0; i <= LASERSCANNER_ZONES - 1; i++) {
		if (laserpar[i] < 50 && laserpar[i] != 0) return i;
	}
	return -1;
}
int rightMostPillar() {
	int i;
	for (i = LASERSCANNER_ZONES - 1; i >= 0; i++) {
		if (laserpar[i] < 50 && laserpar[i] != 0) return i;
	}
	return -1;
}
int isLeftSideBlack() {
	double input[NUMBER_OF_IRSENSORS];
	getTransformedIRData(input);
	int i, blackness = 0;
	double halfNoIRSensors = NUMBER_OF_IRSENSORS / 2;


	for (i = NUMBER_OF_IRSENSORS - 1; i >= NUMBER_OF_IRSENSORS - (int)halfNoIRSensors; i--) {
		blackness += input[i];
	}
	if (blackness > IS_LEFT_SIDE_BLACK) return 1;
	return 0;
}



//End of functions used in gateOnTheLoose:
