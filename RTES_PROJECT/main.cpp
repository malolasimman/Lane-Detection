/* 
 * file name    : main.cpp
 * Hardware     : Raspberry Pi 3B
 * project name : Real-time Lane Detection and Blindspot
                  monitoring for Enhanced Semi-Autonomous Vehicle Safety 
 * author       : Malola Simman Srinivasan Kannan, Shrinithi Venkatesan, Sriraj Vemparala
 * Reference    :https://github.com/ChipHunter/LaneDetection_with_openCV/tree/master/Project1/src

 **/
#include <stdio.h>
#include <pigpio.h>
#include <sys/types.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <time.h>
#include <pthread.h>
#include <signal.h>
#include <sched.h>
#include <syslog.h>
#include <errno.h>
#include <semaphore.h>
#include <string>
#include <sys/sysinfo.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "laneDetector.h"
#include <memory>
#include <time.h>
#include <sys/time.h>
#include <sys/sysinfo.h>

#define BUTTON_PIN_1 17
#define BUTTON_PIN_2 15
#define BUZZER 18
#define LED 4
#define TRIG_PIN 23
#define ECHO_PIN 24
#define TRIG_PIN_US2 5
#define ECHO_PIN_US2 6
#define ERROR (-1)
#define OK (0)
#define NUM_THREADS (4)
#define NUM_CPUS (4)
#define NSEC_PER_SEC (1000000000)
#define NSEC_PER_MSEC (1000000)
#define NSEC_PER_MICROSEC (1000)
#define DELAY_TICKS (1)
#define ERROR (-1)
#define OK (0)
#define TIMER_INTERVAL 1000000 // Timer interval in microseconds


typedef struct
{
    int threadIdx; //thread index 
} threadParams_t;

int numberofprocessors = 1;
struct timespec start_time = {0, 0};
struct timespec end_time = {0, 0};
struct timespec delta_time;


pthread_t thread_blind_spot, thread_alarm, thread_lanedetection;
threadParams_t threadParams_blind_spot, threadParams_alarm, threadParams_lanedetection;
struct sched_param rt_param_blind_spot, rt_param_alarm, rt_param_lanedetection;
pthread_attr_t thread_attr_blind_spot, thread_attr_alarm, thread_attr_lanedetection;

sem_t sem_blind_spot, sem_alarm, sem_img;

double deadline;
struct sched_param rt_attr_transform;
struct sched_param rt_attr_log_time;

pthread_attr_t rt_sched_attr_blind_spot;
pthread_attr_t rt_sched_attr_alarm;
pthread_attr_t rt_sched_attr_lanedetection;
int rt_max_prio, rt_min_prio;
int interval=1000;
pid_t mainpid;
int numberOfProcessors=NUM_CPUS;
int dotted=0; 
int solid=0; 
int flag2=0;
int button1_count = 0;
int button2_count =0;
int left_flag =0;
int right_flag =0;
int thd1flag=0;
unsigned timerHandle;


void button1_callback(int gpio, int level, uint32_t tick)
 {
    if (button1_count == 0) {

        thd1flag=1;
         left_flag =1;
         button1_count = 1;
       }
    else
    {
       button1_count = 0; 
    }
}
void button2_callback(int gpio, int level, uint32_t tick) {
   if (button2_count == 0) {

       thd1flag=1;
         right_flag=1;
         button2_count = 1;
       }
    else
    {
       button2_count = 0; 
    }
}

void gpio_init()
{
    gpioInitialise();
    gpioSetMode(TRIG_PIN, PI_OUTPUT);
    gpioSetMode(ECHO_PIN, PI_INPUT);
   
}


int delta_t(struct timespec *stop, struct timespec *start, struct timespec *delta_t)
{
  int dt_sec=stop->tv_sec - start->tv_sec;
  int dt_nsec=stop->tv_nsec - start->tv_nsec;


  // case 1 - less than a second of change
  if(dt_sec == 0)
  {

	  if(dt_nsec >= 0 && dt_nsec < NSEC_PER_SEC)
	  {
		  delta_t->tv_sec = 0;
		  delta_t->tv_nsec = dt_nsec;
	  }

	  else if(dt_nsec > NSEC_PER_SEC)
	  {
		  delta_t->tv_sec = 1;
		  delta_t->tv_nsec = dt_nsec-NSEC_PER_SEC;
	  }

	  else // dt_nsec < 0 means stop is earlier than start
	  {
		 return(ERROR);  
	  }
  }

  // case 2 - more than a second of change, check for roll-over
  else if(dt_sec > 0)
  {

	  if(dt_nsec >= 0 && dt_nsec < NSEC_PER_SEC)
	  {
		  delta_t->tv_sec = dt_sec;
		  delta_t->tv_nsec = dt_nsec;
	  }

	  else if(dt_nsec > NSEC_PER_SEC)
	  {
		  delta_t->tv_sec = delta_t->tv_sec + 1;
		  delta_t->tv_nsec = dt_nsec-NSEC_PER_SEC;
	  }

	  else // dt_nsec < 0 means roll over
	  {
		  delta_t->tv_sec = dt_sec-1;
		  delta_t->tv_nsec = NSEC_PER_SEC + dt_nsec;
	  }
  }

  return(OK);
}



void *alarm(void *)
{
    struct timespec start_time, end_time,delta_time; 
    gpioSetMode(BUTTON_PIN_1, PI_INPUT);
    gpioSetPullUpDown(BUTTON_PIN_1, PI_PUD_UP);
    gpioSetMode(BUTTON_PIN_2, PI_INPUT);
    gpioSetPullUpDown(BUTTON_PIN_2, PI_PUD_UP);
    gpioSetMode(BUZZER, PI_OUTPUT);
    gpioSetMode(LED, PI_OUTPUT);
    gpioSetAlertFunc(BUTTON_PIN_1, button1_callback);
    gpioSetAlertFunc(BUTTON_PIN_2, button2_callback);

    while (1) {
        
       sem_wait(&sem_alarm);
        clock_gettime(CLOCK_REALTIME ,&start_time);
       int count = 0;
       if(flag2)
       {
          
           gpioWrite(LED, 1);
           clock_gettime(CLOCK_REALTIME ,&end_time); 
            usleep(2000);
           gpioWrite(LED, 0); 
           usleep(2000);
          right_flag=0;
          left_flag=0;
         
       }
       else
       {
             
           gpioWrite(BUZZER, 1); 
           clock_gettime(CLOCK_REALTIME ,&end_time);
            usleep(2000);
            gpioWrite(BUZZER, 0);
            usleep(2000);

        
          right_flag=0;
          left_flag=0;

        }
       
      delta_t(&end_time,&start_time,&delta_time);
      syslog(LOG_CRIT,"1 duration time sec=%d,usec%d",(delta_time.tv_sec),((delta_time.tv_nsec)/NSEC_PER_MICROSEC));
    }
 
    gpioTerminate();
    return 0;
}
 



void *blind_spot(void *)
{
  
   float distance;
    uint32_t start_time_us, end_time_us;

    struct timespec start_time, end_time,delta_time; 
     
    while (1) {
        int count = 0;
        sem_wait(&sem_blind_spot);
        clock_gettime(CLOCK_REALTIME ,&start_time);
        // Send a 10 us pulse to the trigger pin
        if(left_flag)
        {
        gpioWrite(TRIG_PIN, 1);
        usleep(1000);
        gpioWrite(TRIG_PIN, 0);
        
        // Wait for the echo pin to go high
        while (gpioRead(ECHO_PIN) == 0);
        start_time_us = gpioTick();
        
        // Wait for the echo pin to go low
        while (gpioRead(ECHO_PIN) == 1);
        end_time_us = gpioTick();
        // Calculate the distance in cm
        distance = ((end_time_us - start_time_us) / 1000000.0) * 17150.0;
        if((dotted==1) && (distance>30))
        {
            flag2=1;
            sem_post(&sem_alarm);
        }
        else 
        {
             flag2=0; 
             sem_post(&sem_alarm);
        }
             
       }
       else if(right_flag)
       {
        gpioWrite(TRIG_PIN_US2, 1);
        usleep(1000);
        gpioWrite(TRIG_PIN_US2, 0);
        
        // Wait for the echo pin to go high
        while (gpioRead(ECHO_PIN_US2) == 0);
        start_time_us = gpioTick();
        
        // Wait for the echo pin to go low
        while (gpioRead(ECHO_PIN_US2) == 1);
        end_time_us = gpioTick();
        // Calculate the distance in cm
        distance = ((end_time_us - start_time_us) / 1000000.0) * 17150.0;
        if((dotted==1)&&(distance>30))
        {
            flag2=1;
            sem_post(&sem_alarm);
        }
        else
        {
             flag2=0;
             sem_post(&sem_alarm);
        }
       
       //|
        //printf("US2_distance=%.2f\n",distance);
      }
      clock_gettime(CLOCK_REALTIME ,&end_time);
      delta_t(&end_time,&start_time,&delta_time);
      syslog(LOG_CRIT,"2 duration time sec=%d,msec%d",(delta_time.tv_sec),((delta_time.tv_nsec)/NSEC_PER_MSEC));

   }  
    gpioTerminate();
    
    return 0;
}  
  

void *image_processing(void *)
{
    struct timespec start_time, end_time,delta_time; 

   cv::Mat frame, img_reduce_noise, img_edge_detect, img_mask;
    std::vector<cv::Vec4i> lines, solidlines, dottedlines;
    
    try {
    laneDetector *lane_detection;
 lane_detection = new laneDetector("/home/rpi/Desktop/theVideo.mp4");

        while (true) {
             clock_gettime(CLOCK_REALTIME ,&start_time);
            lane_detection->readFrame(frame);

            lane_detection->Noise_reduction(frame, img_reduce_noise);

            lane_detection->edgeDetection(img_reduce_noise, img_edge_detect);

            lane_detection->mask_requiredlane(img_edge_detect, img_mask);

            lane_detection->houghLines(img_mask, lines,solidlines, dottedlines);

            clock_gettime(CLOCK_REALTIME ,&end_time);
            delta_t(&end_time,&start_time,&delta_time);
            syslog(LOG_CRIT,"3 duration time sec=%d,msec%d",(delta_time.tv_sec),((delta_time.tv_nsec)/NSEC_PER_MSEC));
            if (!lines.empty()) {

                lane_detection->plot(lines, solidlines, dottedlines,frame);
               
            }

            cv::waitKey(5);

        } 
    } catch(std::exception& e) {
    
        std::cout << "some exception: " << e.what() << std::endl;

    } catch (...) {

        std::cout << "other exception: " << std::endl;

    }
        
return 0;
}

int main(int argc, char** argv)
{
    int rc;
     int i;
    cpu_set_t threadcpu;
    int coreid;
    gpio_init();
  
   numberOfProcessors = get_nprocs_conf();

    openlog(NULL, LOG_PID , LOG_USER);
    sem_init(&sem_blind_spot, 0, 0);
    sem_init(&sem_alarm, 0, 0);
    sem_init(&sem_img, 0, 0);


   mainpid=getpid(); // Stores the thread's Process ID in the `mainpid` variable

   rt_max_prio = sched_get_priority_max(SCHED_FIFO); // getting the maximum priority for the SCHED_FIFO
   rt_min_prio = sched_get_priority_min(SCHED_FIFO); // getting the minimum priority for the SCHED_FIFO
    
       CPU_ZERO(&threadcpu);
       coreid=i%numberOfProcessors;  // calculating the core id 
       printf("Setting thread %d to core %d\n", i, coreid);
       CPU_SET(coreid, &threadcpu);
       
       rc=pthread_attr_init(&rt_sched_attr_lanedetection);                                     // Initialize the attributes
       rc=pthread_attr_setinheritsched(&rt_sched_attr_lanedetection, PTHREAD_EXPLICIT_SCHED);  // Set the inheritance attribute
       rc=pthread_attr_setschedpolicy(&rt_sched_attr_lanedetection, SCHED_FIFO);             // Set the scheduling policy to MY_SCHEDULER
       rc=pthread_attr_setaffinity_np(&rt_sched_attr_lanedetection, sizeof(cpu_set_t), &threadcpu); //sets the thread's affinity to a specific core

       rc=pthread_attr_init(&rt_sched_attr_blind_spot);                                     // Initialize the attributes
       rc=pthread_attr_setinheritsched(&rt_sched_attr_blind_spot, PTHREAD_EXPLICIT_SCHED);  // Set the inheritance attribute
       rc=pthread_attr_setschedpolicy(&rt_sched_attr_blind_spot, SCHED_FIFO);             // Set the scheduling policy to MY_SCHEDULER
       rc=pthread_attr_setaffinity_np(&rt_sched_attr_blind_spot, sizeof(cpu_set_t), &threadcpu); //sets the thread's affinity to a specific core

       rc=pthread_attr_init(&rt_sched_attr_alarm);                                     // Initialize the attributes
       rc=pthread_attr_setinheritsched(&rt_sched_attr_alarm, PTHREAD_EXPLICIT_SCHED);  // Set the inheritance attribute
       rc=pthread_attr_setschedpolicy(&rt_sched_attr_alarm, SCHED_FIFO);             // Set the scheduling policy to MY_SCHEDULER
       rc=pthread_attr_setaffinity_np(&rt_sched_attr_alarm, sizeof(cpu_set_t), &threadcpu); //sets the thread's affinity to a specific core
       rt_param_blind_spot.sched_priority=rt_max_prio-2;                          // Set the scheduling priority
       rt_param_alarm.sched_priority = rt_max_prio-1;
       rt_param_lanedetection.sched_priority = rt_max_prio - 3;
       pthread_attr_setschedparam(&rt_sched_attr_blind_spot, &rt_param_blind_spot);
       pthread_attr_setschedparam(&rt_sched_attr_alarm, &rt_param_alarm);
       pthread_attr_setschedparam(&rt_sched_attr_lanedetection, &rt_param_lanedetection);
       

    if(pthread_create(&thread_lanedetection,&thread_attr_lanedetection,image_processing,NULL) !=0)
    {
        syslog(LOG_ERR,"pthread_create error from timestamp");
        exit(1);
    }
    
    if(pthread_create(&thread_blind_spot,&thread_attr_lanedetection,blind_spot,NULL) !=0)
    
    {
        syslog(LOG_ERR,"pthread_create error from timestamp");
        exit(1);
    }
    
    if(pthread_create(&thread_alarm,&thread_attr_alarm,alarm, NULL) !=0)
    {
        syslog(LOG_ERR,"pthread_create error from timestamp");
        exit(1);
    }
    
       if(pthread_join(thread_lanedetection, NULL) !=0 )
    {
        syslog(LOG_ERR,"pthread_join");
        exit(1);
    }
   
    if(pthread_join(thread_blind_spot, NULL) !=0 )
    {
        syslog(LOG_ERR,"pthread_join");
        exit(1);
    }

    if (pthread_join(thread_alarm, NULL) !=0 )
    {
        syslog(LOG_ERR,"pthread_join");
        exit(1);
    }
    gpioTerminate();


    sem_destroy(&sem_blind_spot);
    sem_destroy(&sem_alarm);
    sem_destroy(&sem_img);
    closelog();
return 0;
}
