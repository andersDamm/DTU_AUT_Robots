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
#define WHEEL_DIAMETER   0.067	/* m */
#define WHEEL_SEPARATION 0.282	/* m 0.252 */
#define DELTA_M (M_PI * WHEEL_DIAMETER / 2000)
#define ROBOTPORT	24902
#define AJAX 0.5 /* m/s² */
#define SIZE_OF_ARRAY 10000
#define CONVERSION_FACTOR_ACC 0.01
#define K_FOR_STRAIGHT_DIRECTION_CONTROL 1
#define K_FOR_ACCELERATING_DIRECTION_CONTROL 0.001
#define K_FOR_FOLLOWLINE 0.1
#define NUMBER_OF_IRSENSORS 8
#define CRITICAL_IR_VALUE 0.5
#define IS_SIMULATION 0 //1=simulation, 0=real world

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
}motiontype;

enum {mot_stop=1,mot_move,mot_turn,mot_followLineCenter,mot_followRightLine,mot_followLeftLine};

void update_motcon(motiontype *p);


int fwd(double dist, double speed,int time);
int turn(double angle, double speed,int time);
int followLineCenter(double dist, double speed, int time);

void getTransformedIRData(double* output); // Calibfunction - Calibrates in relation to black_mean.
int minIntensity();            // Minimum intensity function
double centerOfGravity(char color);  // Finding the line with centre of gravity algorithm

double leftMostNegSlope();
double leftMostPosSlope();

// Returns a number from 0 to 7, which indicates the position 
// at which the right most negative slope begins.
double rightMostNegSlope();	
									      
// Returns a number from 0 to 7, which indicates the position 
// at which the right most positive slope begins.
double rightMostPosSlope();
char stopLine();
int followRightLine(double dist, double speed, int time);

int followRightLine(double dist, double speed, int time);

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

enum {ms_init,ms_fwd,ms_turn,ms_followLineCenter,ms_followRightLine,ms_followLeftLine,ms_end};

int main()
{
  poseTimeLog_t poseTimeLog_a[SIZE_OF_ARRAY];
  int counter = 0;
  int running,n=0,arg,time=0;
  double dist=0,angle=0;

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
    len=sprintf(buf,"push  t=0.2 cmd='mrcobst width=0.4'\n");
    send(lmssrv.sockfd,buf,len,0);
}

}


  /*
  /  Read sensors and zero our position.
  */
  rhdSync();

  odo.w=0.256;
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
  n=4; dist=2;angle= - 90.0/180*M_PI;
  mission.state= ms_followRightLine;
  break;

  case ms_fwd:
    if (fwd(dist,0.6,mission.time))  mission.state=ms_turn; // Square min ven
    break;

    case ms_turn:
    if (turn(angle,0.3,mission.time)){
     n=n-1;
     if (n==0)
       mission.state=ms_end;
   else
       mission.state=ms_fwd;
}
break;

case ms_followLineCenter:
if (followLineCenter(dist,0.3,mission.time)) mission.state = ms_end;
break;

case ms_followRightLine:
if (followRightLine(dist,0.3,mission.time)) mission.state = ms_end;
break;

case ms_followLeftLine:
if (followRightLine(dist,0.3,mission.time)) mission.state = ms_end;
break;

case ms_end:
mot.cmd=mot_stop;
running=0;
    // Output the data
if(log_data_to_file(poseTimeLog_a, counter)){
  printf("No data logged\n");
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
   } else{
       printf("array out of bounds\n");
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
    //    Good place for outputting debugging variables.
    //    printf(" laser %f \n",laserpar[3]);
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

    delta_l = p->left_enc - p->left_enc_old;
    if (delta_l > 0x8000) delta_l -= 0x10000;
    else if (delta_l < -0x8000) delta_l += 0x10000;
    p->left_enc_old = p->left_enc;
    p->left_pos += delta_l * p->cl;

    delta_u = ((double)delta_r * p->cr + (double)delta_l * p->cl)/2;
    delta_theta = ((double)delta_r * p->cr - (double)delta_l * p->cl)/(double)p->w;
    p->theta += delta_theta;
    p->x_pos += delta_u*cos(p->theta);
    p->y_pos -= delta_u*sin(p->theta);

}


void update_motcon(motiontype *p){
    double d, d_angle;
    if (p->cmd !=0){
        p->finished=0;
        switch (p->cmd){
            case mot_stop:
            p->curcmd=mot_stop;
            break;

            case mot_move:
            p->startpos=(p->left_pos+p->right_pos)/2;
            p->curcmd=mot_move;
            break;

            case mot_turn:
            if (p->angle > 0)
                p->startpos=p->right_pos;
            else
                p->startpos=p->left_pos;
            p->curcmd=mot_turn;
            break;

            case mot_followLineCenter:
            p->curcmd=mot_followLineCenter;
            break;
	    
	    case mot_followRightLine:
            p->curcmd=mot_followRightLine;
            break;
	    
	    case mot_followLeftLine:
            p->curcmd=mot_followRightLine;
            break;
        }
        p->cmd=0;
    }

    switch (p->curcmd){
        case mot_stop:
            p->motorspeed_l=0;
            p->motorspeed_r=0;
            break;

        case mot_move:
            d=((p->motorspeed_l+p->motorspeed_r)/2)*((p->motorspeed_l+p->motorspeed_r)/2)/(2*(AJAX));

            if ((p->right_pos+p->left_pos)/2- p->startpos >= p->dist){
                p->finished=1;
                p->motorspeed_l=0;
                p->motorspeed_r=0;
            }
            else if((p->right_pos+p->left_pos)/2- p->startpos > p->dist-d){ // Deacceleration
                p->motorspeed_l -= AJAX*CONVERSION_FACTOR_ACC - K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
                p->motorspeed_r -= AJAX*CONVERSION_FACTOR_ACC + K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
            }
            else if(p->motorspeed_l<p->speedcmd){                           // Acceleration
                p->motorspeed_l += AJAX*CONVERSION_FACTOR_ACC - K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
                p->motorspeed_r += AJAX*CONVERSION_FACTOR_ACC + K_FOR_ACCELERATING_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
            }
            else {
                p->motorspeed_l=p->speedcmd - K_FOR_STRAIGHT_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
                p->motorspeed_r=p->speedcmd + K_FOR_STRAIGHT_DIRECTION_CONTROL*(odo.theta_ref - odo.theta);
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

        case mot_followLineCenter:
            if (p->right_pos < p->dist) {
            p->motorspeed_l = p->speedcmd - K_FOR_FOLLOWLINE*(minIntensity() - 3.5);
            p->motorspeed_r = p->speedcmd + K_FOR_FOLLOWLINE*(minIntensity() - 3.5);
            }
            else {
                p->motorspeed_l = 0;
                p->motorspeed_r = 0;
                p->finished = 1;
            }
            break;

        case mot_followRightLine:
            if (p->right_pos < p->dist) {
                p->motorspeed_l = p->speedcmd - K_FOR_FOLLOWLINE*(rightMostPosSlope() - 2.);
                p->motorspeed_r = p->speedcmd  + K_FOR_FOLLOWLINE*(rightMostPosSlope() - 1);
            }
            else {
                p->motorspeed_l = 0;
                p->motorspeed_r = 0;
                p->finished = 1;
            }
            break;
	    
	case mot_followLeftLine:
            if (p->right_pos < p->dist) {
                p->motorspeed_l = p->speedcmd - K_FOR_FOLLOWLINE*(rightMostPosSlope() - 2.);
                p->motorspeed_r = p->speedcmd  + K_FOR_FOLLOWLINE*(rightMostPosSlope() - 1);
            }
            else {
                p->motorspeed_l = 0;
                p->motorspeed_r = 0;
                p->finished = 1;
            }
            break;
    }
}

int fwd(double dist, double speed,int time){
    if (time==0){
        mot.cmd=mot_move;
        mot.speedcmd=speed;
        mot.dist=dist;
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
int followLineCenter(double dist, double speed, int time){   // linesensor input???
  if(time == 0){
     mot.cmd = mot_followLineCenter;
     mot.speedcmd = speed;
     mot.dist = dist;
     return 0;
 }
 else return mot.finished;
}

int followRightLine(double dist, double speed, int time){   // linesensor input???
  if(time == 0){
     mot.cmd = mot_followRightLine;
     mot.speedcmd = speed;
     mot.dist = dist;
     return 0;
 }
 else return mot.finished;
}

int followLeftLine(double dist, double speed, int time){   // linesensor input???
  if(time == 0){
     mot.cmd = mot_followRightLine;
     mot.speedcmd = speed;
     mot.dist = dist;
     return 0;
 }
 else return mot.finished;
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
  printf("\nStarting logging of data\n");
  outFile = fopen("Log.dat", "w");
  if (outFile == NULL) {
    fprintf(stderr, "ERROR: couldn't open %s for writing\n", "Log.dat");
    return 1;
}
for(i = 0;i < size; i++) {
    fprintf(outFile,"%i,%f,%f,%f\n",poseTimeLog_out[i].time, poseTimeLog_out[i].x, poseTimeLog_out[i].y, poseTimeLog_out[i].theta);
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
      if(IS_SIMULATION==0){
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

//finds black line with IR sensors using the center of gravity algorithm
//color: 0=black, 1=white
double centerOfGravity(char color){
  // Input is raw data from linesensors. Between each photoLED exist one "i";
    int sumI = 0, sumXI=0, i;
    double input[NUMBER_OF_IRSENSORS];
    getTransformedIRData(input);
    for(i = 0; i< NUMBER_OF_IRSENSORS; i++){
        input[i] = color==0 ? 1-input[i] : input[i];    // 0 is black, everything else is white.
    }
    for(i=0; i < NUMBER_OF_IRSENSORS; i++){
        sumI += input[i];
        sumXI += i*input[i];
    }
    return (double)sumXI/(double)sumI;
}




//We follow the convention of the x-axis given as the numbers of the IR-sensors 0 to 7. The LEFT-most IR-sensor is no. 7 and the RIGHT-most IR-sensor is no. 0.!!!!!!!

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
double rightMostPosSlope(){
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









/*
//checks if the IR sensors detect a stop line.
//return values: 1=line detected, 0=no line detected
char stopLine(){
  int i;
  double input[NUMBER_OF_IRSENSORS];
  getTransformedIRData(input);
  for(i=0;i<NUMBER_OF_IRSENSORS;i++){
      if(linesensor->data[i]<0.9*maxIRValues[i]){
          return 0;
      }
  }
  return 1;
}
*/