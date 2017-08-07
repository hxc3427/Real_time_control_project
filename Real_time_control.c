#include <stdlib.h>
#include <stdio.h>
#include <hw/inout.h>
#include <stdint.h>
#include <time.h>
#include <semaphore.h>
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <pthread.h>
#include <sys/timeb.h>
#include <sched.h>
#include <sys/neutrino.h>
#include <string.h>
#include <stdint.h>
#include <sys/mman.h>

#define Port_Length 1
#define Data_Base 0x280 //base value of digital io port
#define Base_Address 0x280
sem_t syncA_input;
sem_t syncDAC_function;
sem_t syncPID;

//-----------------------------------DAC MMAP VARIABLES----------------



uintptr_t DAC_mode;
uintptr_t DAC_Range;
uintptr_t DAC_MSB;
uintptr_t DAC_LSB;
uintptr_t DACBSY;// DAC STATE
uintptr_t DAC_Page;

//------------------------------DAC CALCULATION VARIABLES--------------------

int calValue=0;
signed int MSB = 0, MSB1;
signed int LSB = 0;
signed short int reading;
float voltage[2];
float period, Desired_Output=0;
int DA_code;
int channel = 0 ;

//---------------------------------------------------------------------
int setpoint= 3;//set the voltage
int ADC_measured_volts=0;
int PID_op_DAC_ip=0;
int synct = 0;


void* sync_thread();
void* A_input_function();
void* PID_controlled_output();
void* DAC_function();


int main(int argc, char *argv[]) {

	pthread_t A_input;
	pthread_t PID;
	pthread_t DAC;
	pthread_t syncthreads;
	//pthread_attr_t attribute;

	sem_init(&syncA_input,0,0);
	sem_init(&syncPID,0,0);
	sem_init(&syncDAC_function,0,0);


	//pthread_attr_init(&attribute);
	pthread_create(&syncthreads, NULL, &sync_thread, NULL);
	pthread_create(&A_input, NULL, &A_input_function, NULL);
	pthread_create(&PID, NULL, &PID_controlled_output, NULL);
	pthread_create(&DAC, NULL, &DAC_function, NULL);
	while(!synct);
	sleep(1);

		pthread_join(syncthreads,NULL);
	    pthread_join(A_input,NULL);
	    pthread_join(PID,NULL);
	    pthread_join(DAC,NULL);
	    sem_destroy(&syncA_input);
	    sem_destroy(&syncPID);
	    sem_destroy(&syncDAC_function);

	return NULL;
}

void* sync_thread(){
	synct = 0;
	sleep(15);
	synct = 1;
	return NULL;
}

void* A_input_function(){

	sem_post(&syncA_input);

	uintptr_t dir_ptr=mmap_device_io(Port_Length,Data_Base +0xB); //pointer for setting direction of io ports
	uintptr_t da_ptr=mmap_device_io(Port_Length,Data_Base +0x8);  //pointer to port A
	uintptr_t ADC_channel=mmap_device_io(Port_Length,Data_Base +0x2);  //pointer to ADC board channel
	uintptr_t ADC_input=mmap_device_io(Port_Length,Data_Base +0x3);  //pointer to ADC board channel
	uintptr_t ADC_CMD=mmap_device_io(Port_Length,Data_Base +0x0);  //pointer to ADC board channel
	uintptr_t ADC_MSB=mmap_device_io(Port_Length,Data_Base +0x1);  //pointer to ADC board channel
	uintptr_t ADC_LSB=mmap_device_io(Port_Length,Data_Base +0x0);  //pointer to ADC board channel

	signed char MSB;
	signed char LSB;
	signed short int reading;
	float voltage;

    if ( ThreadCtl(_NTO_TCTL_IO, NULL) == -1)   // root permission
				{

				perror("Failed to get I/O access permission");
				//return 1;
				}
	if(dir_ptr == MAP_DEVICE_FAILED)  // test for root permission
				{
				perror("Failed to map control register");
				//return 2;
				}

	if(da_ptr == MAP_DEVICE_FAILED) // test for root permission
				{
				perror("Failed to map control register");
				//return 2;
				}

	out8(dir_ptr,0x00);  //Select direction of Digital IO
	out8(ADC_channel,0x44); //select channel 4 or Pin 43
	//delay(100);
	out8(ADC_input,0x00);   //select input range of -10 to 10
	//delay(100);

	while(1)
				{
					sem_wait(&syncA_input);
				out8(ADC_CMD,0x80);  //initiate ADC conversion
				while(in8(ADC_input)&& 0x80 !=0)
				{};

				LSB = in8(ADC_LSB);
				MSB = in8(ADC_MSB);
				reading = (MSB*256) + LSB;
				voltage = reading;

				voltage = ((voltage/32768)*10);
		      //  printf("ADC Measured value is %f \n",voltage);
		        ADC_measured_volts= voltage;

		        if (synct == 1) {
			        sem_post(&syncPID);
			         break;
		           }
		        else {
			         //Start converting the next one!
			          out8(ADC_CMD, 0x80);
			           sem_post(&syncPID);
		            }
				}
	return NULL;
}

void* DAC_function(){


		 sem_post(&syncDAC_function);



		 DACBSY = mmap_device_io(Port_Length,Data_Base +0x03);  //DAC STATE
		 DAC_mode = mmap_device_io(Port_Length,Data_Base +0x0B);  //UPDATE TYPE
		 DAC_LSB = mmap_device_io(Port_Length,Data_Base +0x06);  //pointer to port A
		 DAC_MSB = mmap_device_io(Port_Length,Data_Base +0x07);  //pointer to port A
		 DAC_Range = mmap_device_io(Port_Length,Data_Base +0x0E);  //DAC POLARITY
		 DAC_Page = mmap_device_io(Port_Length,Data_Base +0x01);  //PAGE SELECT

		 if ( ThreadCtl(_NTO_TCTL_IO, NULL) == -1)   // root permission
						{

						perror("Failed to get I/O access permission");
						//return 1;
						}

		 out8(DAC_Page, 0x00);
		 	    // delay(100);

		 	     out8(DAC_mode, 0x00); //SIMULTANEOUS UPDATE
		 	   //  delay(100);



		 		//out8(DAC_mode, in8(DAC_mode)& 0xDF);   //select input range of -10 to 10
		 		//delay(100);

		         out8(DAC_Page, 0x02);
		        // delay(100);

		         out8(DAC_Range, 0x0A);
		         //out8(ADC_Polarity, 0x01);
		 	//	delay(100);
		 		 out8(DAC_Page, 0x00);
		 		       // delay(100);




		 	   /* Give this thread root permissions to access the hardware */


				 while(1)
							{
					 sem_wait(&syncDAC_function);

					 	 	 Desired_Output= PID_op_DAC_ip;
					 		DA_code=((Desired_Output/10)*2048)+2048;
					        	//printf("%d DA_code\n",DA_code);



				               LSB = DA_code&255;
				                MSB = (DA_code/256);

				                out8(DAC_LSB, LSB);

				                out8(DAC_MSB, MSB+(channel<<6));
				                if (synct == 1) {

				                			         break;
				                		           }


							}
				 return NULL;
}

void* PID_controlled_output(){

	int previous_error = 0;
	int integral=0;
	double ki=180;///////////////////
	double kp=200; //measure through hit and trial
	double kd=196;//////////////////
	int proportional = 0;
	int derivative = 0;
	//int dt=500;

	while(1){
		sem_wait(&syncPID);
		double error = setpoint - ADC_measured_volts;

		proportional = kp*error;

		integral +=(error);

		if(integral > 10)
			integral = 10;
		else if(integral < -10)
			integral = -10;

		integral *=ki;

		derivative = (previous_error - error)/dt;


		derivative *=kd;

		previous_error=error;

		PID_op_DAC_ip=(int)(proportional + integral + derivative);

		if(PID_op_DAC_ip > 5)

			PID_op_DAC_ip=5;

		else if(PID_op_DAC_ip < -5)

			PID_op_DAC_ip= -5;

		sem_post(&syncDAC_function);

		if (synct == 1) {
			break;
	}
	//printf("PID Controlled value %d \n",PID_op_DAC_ip);
}
	return NULL;
}
