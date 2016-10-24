
#include "p30f6014a.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include <libpic30.h>

#include "motor_led/e_epuck_ports.h"
#include "motor_led/e_init_port.h"
#include "motor_led/e_led.h"
#include "motor_led/e_motors.h"
#include "uart/e_uart_char.h"
#include "a_d/advance_ad_scan/e_prox.h"
#include "a_d/advance_ad_scan/e_ad_conv.h"
#include "contrib/radio_swis/v2/e_radio_swis.h"
#include "contrib/robot_id/e_robot_id.h"

// General configuration
#define GROUP_ID        0x54
#define COM_POWER       RADIO_SWIS_SW_ATTENUATOR_0DB       // this is a value from 3 (lowest power) to 31 (full power)

// AUXILIARY
#define NB_SENSORS        8		// number of sensors
#define BIAS_SPEED        400		// robot bias speed
#define MAXSPEED          800		// maximum robot speed
#define DIST_THRESHOLD    150
#define TURNSTEPNUMBER    1320

#define NUM_ROBOTS        3
#define MIN_CONNECTIONS   2
#define COMM_RADIUS       0.7
#define COMM_STEPS        15
#define MIN_AVOID_STEPS   4
#define DECISION_INTERVAL 1000   	// the decision interval in milliseconds

// STATES
#define FORWARD           1
#define COHERENCE         2
#define TURN              3

// Constants for the position of the selector
enum {
	SELECTOR_RIGHT = 0,
	SELECTOR_REAR = 4,
	SELECTOR_LEFT = 8,
	SELECTOR_FRONT = 12,
};

int robot_id;

// Calibration function and data for the proximity sensors
void calibrate_sensors(void);
int _EEDATA(32) EEcalib[16];	// 16 bytes (EEPROM) in order to perform access to a row -> last bytes are null
int calib_sensor[16];			// Same, but stored in RAM

char buffer[200];

int distances[NB_SENSORS];  // for sensor readings
int avoid_steps;          	//counter for avoidance
int have_to_broadcast;
int num_neighbors;       	//communication management
int prev_num_neighbors;   	//communication management
int num_msgs;

//wall following and obstacle avoidance variables
int weightleft[NB_SENSORS] = {-10,-10,-5,0,0,5,10,-10};
int weightright[NB_SENSORS] = {10,10,5,0,0,-5,-10,10};

// Function to set wheel speeds within bounded limits
void setSpeed(int LeftSpeed, int RightSpeed)
{
	if (LeftSpeed < -MAXSPEED) {LeftSpeed = -MAXSPEED;}
	if (LeftSpeed >  MAXSPEED) {LeftSpeed =  MAXSPEED;}
	if (RightSpeed < -MAXSPEED) {RightSpeed = -MAXSPEED;}
	if (RightSpeed >  MAXSPEED) {RightSpeed =  MAXSPEED;}
  
  	e_set_speed_left(LeftSpeed);
    e_set_speed_right(RightSpeed);
}

// Function to obtain calibrated proximity sensor readings
void getSensorValues(int *sensorTable)
{
	unsigned int i;
	for (i=0; i < NB_SENSORS; i++) {
		sensorTable[i] = e_get_prox(i) - calib_sensor[i];
	}		
}

// Waits for a certain amount of time
// Note that the effective waiting time is not always the same (because of possible interrupts).
void wait_a_while(unsigned long num) {
    while (num > 0) {num--;}
}

// Sensor calibration function
void calibrate_sensors(void)
{
	int i;
	_prog_addressT EE_addr;
	_init_prog_address(EE_addr, EEcalib);
	
	// Read the calibration from EEPROM
	_memcpy_p2d16(calib_sensor, EE_addr, _EE_ROW);
	
	// Calibrate if there is no configuration in EEPROM or if the user wants to do so
	if ( (calib_sensor[8] != 0x5555) || (e_get_selector() == SELECTOR_REAR) ){
		// Calibrate the proximity sensors
		for (i=0; i<8; i++){
			e_set_led(i, 1);
		}
		wait_a_while(1000000l);
		for (i=0; i<8; i++) {
			calib_sensor[i] = e_get_prox(i);
		}
		
		calib_sensor[8] = 0x5555;	// Set a control value
		// Write to EEPROM		
		_erase_eedata(EE_addr, _EE_ROW);
		_wait_eedata();
		_write_eedata_row(EE_addr, calib_sensor);
		_wait_eedata();
		
		e_led_clear();
	}	
}

// Auxiliary function to turn the e-puck (mode = 1 - turn 180deg, mode = 2 - turn in bounded random direction)
void turn(int mode){

	int steps;
	int dir=0;

	if(mode==1) steps=TURNSTEPNUMBER/2;
	if(mode==2) {
		steps=rand()%TURNSTEPNUMBER/6+1;
		if(rand()%20<10) dir=1;
	}

	// set speed to zero and reset wheel step counters
	setSpeed(0,0);
	e_set_steps_left(0);
	e_set_steps_right(0);

	// Turn until desired number of steps has been reached
	if(dir==1){
		setSpeed(-BIAS_SPEED,BIAS_SPEED);
		while(e_get_steps_right()<steps);
	}
	else{
		setSpeed(BIAS_SPEED,-BIAS_SPEED);
		while(e_get_steps_left()<steps);
	}
	setSpeed(0,0);
}

// Moves the robot in the straight line. If a obstacle is detected in
// the robot's path, it executes a Braintenber-like control in order
// to avoid it. This controller is executed always durint MIN_AVOID_STEPS
// robot control steps.
void move_and_avoid(){

	int leftwheel, rightwheel;		// motor speed left and right
	int i;
	int gostraight;

	leftwheel=BIAS_SPEED;
	rightwheel=BIAS_SPEED;

	getSensorValues(distances); 	// read sensor values

	gostraight=0;
	for (i=0; i<NB_SENSORS; i++) {
		if (distances[i]>DIST_THRESHOLD) {break;}		// if obstacle detected
	}

	if (i==NB_SENSORS && avoid_steps==0) {
		gostraight=1;
	}

	if (gostraight==0 || avoid_steps>0) {
		if(avoid_steps==0)
			avoid_steps=MIN_AVOID_STEPS;				// reset number of steps for Braintenberg execution
		else
			avoid_steps--;
		for (i=0; i<NB_SENSORS; i++) {
			// Braintenberg-like obstacle avoidance
			leftwheel+=weightleft[i]*(distances[i]>>4);
			rightwheel+=weightright[i]*(distances[i]>>4);
		}
	}
  	setSpeed(leftwheel, rightwheel);
}

// Initializes timer1
// This gives us an interrupt every DECISION_INTERVAL ms.
void start_timer1(int interval) {
    T1CON = 0;
    T1CONbits.TCKPS = 3;                // prescsaler = 256
    TMR1 = 0;                           // clear timer 1
    PR1 = (interval*MILLISEC)/256.0;    // interrupt after requested # of ms
    IFS0bits.T1IF = 0;                  // clear interrupt flag
    IEC0bits.T1IE = 1;                  // set interrupt enable bit
    T1CONbits.TON = 1;                  // start Timer1
}

// Timer 1 interrupt
// This code is executed every DECISION_INTERVAL ms. It allows us
// to decide the frequency of information broadcast by the robot.
void _ISRFAST _T1Interrupt(void) {
    
    // Clear interrupt flag
    IFS0bits.T1IF = 0;
	
    // Update the state variables
    have_to_broadcast++;
}

// Broadcasts a packet with the robot id via the robot radio module.
void broadcast() {

    e_radio_swis_send(GROUP_ID, RADIO_SWIS_BROADCAST, (char *)&robot_id, sizeof(robot_id));
    e_set_front_led(1);
    sprintf(buffer, "robot id %d - broadcast \r\n",robot_id);
    e_send_uart1_char(buffer, strlen(buffer));
}

// Check if any packet was received by the robot radio module, update
// number of messages received if yes.
void packet_received() {
    unsigned int size;
    unsigned char packet[6];
    int rssi;
    
    if (e_radio_swis_packet_ready(packet, &size)) {
		if(robot_id != *((int*)&packet[0])){		// Sender is not same robot
			rssi=e_radio_swis_get_rssi();
			num_msgs++;
			e_set_body_led(1);
			sprintf(buffer, "robot id %d - packet received = %d - rssi = %d \r\n",robot_id,*((int*)&packet[0]),rssi);
			e_send_uart1_char(buffer, strlen(buffer));
		}
    }
}

// Updates number of neighbors variables
void update_neighbors(){
  
  prev_num_neighbors=num_neighbors;
  num_neighbors=num_msgs;
  num_msgs=0;
}

// Checks if connections were lost (below MIN_CONNECTIONS)
int lost_connection(){

  if(num_neighbors<prev_num_neighbors && num_neighbors<MIN_CONNECTIONS){
    return 1;
  }
  return 0;
}

// Checks if connections were regained
int regained_connection(){

  if(num_neighbors>prev_num_neighbors){
    return 1;
  }
  return 0;
}

// Main program
int main() {
    int i;

    // Initialize system and sensors
    e_init_port();
    e_init_uart1();
    e_init_motors();
    e_init_ad_scan();
    e_init_robot_id();
    e_radio_swis_init_default(RADIO_SWIS_SLAVE_MASTER, NULL,
				RADIO_SWIS_HW_ATTENUATOR_0DB, 
				RADIO_SWIS_SW_ATTENUATOR_0DB);
    e_radio_swis_set_group(GROUP_ID);
    e_radio_swis_set_payload_length(4);
    robot_id = e_radio_swis_get_address();
	
    // Calibrate the proximity sensors or retrive the calibration from EEPROM
    calibrate_sensors();

    // Reset if power on (this is necessary for some e-pucks)
    if (RCONbits.POR) {
        RCONbits.POR = 0;
        __asm__ volatile ("reset");
    }

    // Initialize variables
    avoid_steps=0;
    have_to_broadcast=0;
    num_neighbors=0;
    prev_num_neighbors=0;
    num_msgs=0;

    // Say hello (via Bluetooth) to a base station
    sprintf(buffer, "WCS starting - robot id = %d \r\n",robot_id);
    e_send_uart1_char(buffer, strlen(buffer));

    // Initialize the random number generator
    srand(e_get_prox(0) + e_get_prox(2) + e_get_prox(4) + e_get_prox(6));

    // Start timer 1 (this takes a decision)
    start_timer1(DECISION_INTERVAL);

    // Main Loop
    while (1) {

	e_set_body_led(0);
	e_set_front_led(0);

	move_and_avoid();

	packet_received();

	// Broadcast and check if connections were lost/regained
	if(have_to_broadcast>2){
		have_to_broadcast=0;
		broadcast();
		update_neighbors();

		if(regained_connection()){ // If connection regained, blink led, send message to base station via bluetooth, do random turn
			for (i=0; i<8; i++){
				e_set_led(i, 1);
			}
			sprintf(buffer, "robot id %d - regained connection \r\n",robot_id);
			e_send_uart1_char(buffer, strlen(buffer));
			turn(2);
		}
		if(lost_connection()){  // If connection lost, blink led, send message to base station via bluetooth, do 180deg turn
			for (i=0; i<8; i++){
				e_set_led(i, 0);
			}
			sprintf(buffer, "robot id %d - lost connection \r\n",robot_id);
			e_send_uart1_char(buffer, strlen(buffer));
			turn(1);
		}
	}

	// Wait for some time
        wait_a_while(100000);
    }

    return 0;
}

