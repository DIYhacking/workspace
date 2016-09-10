#include "sensor_task.h"
#include <ch.h>
#include <hal.h>
#include "stm32l_rtc.h"
#include "ntc.h"

#define MAX_INTERVAL 60
#define MIN_INTERVAL 20
#define MAX_DEADTIME 120

volatile static int g_temperature;
volatile static int g_timestamp = -1;
volatile static bool g_wireless;

/*Heart Rate*/
volatile int rate[10];                    // array to hold last ten IBI values
volatile unsigned long sampleCounter = 0;          // used to determine pulse timing
volatile unsigned long lastBeatTime = 0;           // used to find IBI
volatile int P =712;//512                      // used to find peak in pulse wave, seeded
volatile int T = 712;                     // used to find trough in pulse wave, seeded
volatile int thresh = 725; //525->5V,725->3V               // used to find instant moment of heart beat, seeded
volatile int amp = 100;                   // used to hold amplitude of pulse waveform, seeded
volatile bool firstBeat = true;        // used to seed rate array so we startup with reasonable BPM
volatile bool secondBeat = false;      // used to seed rate array so we startup with reasonable BPM

volatile int BPM;                   // int that holds raw Analog in 0. updated every 2mS
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // int that holds the time interval between beats! Must be seeded!
volatile bool Pulse = false;     // "True" when User's live heartbeat is detected. "False" when not a "live beat".
static volatile bool QS = false;        // becomes true when Arduino finds a beat.
volatile int heart;
int intervalN,BP;
int status;
int peak,trough,thrsh,pl;

volatile static int g_heart;
volatile static int g_temp;
volatile static float g_gsr;


bool sensor_get_temperature(int *temperature, int *timestamp, bool *wireless)
{
    chSysLock();
    *temperature = g_temperature;
    *timestamp = g_timestamp;
    *wireless = g_wireless;
    chSysUnlock();
    
    int now = rtc_get();
    if (*timestamp < 0 || *timestamp < now - MAX_DEADTIME)
        return false;
    
    return true;
}

bool pass_heart_temp(int *heart, int *temper, float *gsr1)
{
	*heart=g_heart;
	*temper=g_temp;
	*gsr1=g_gsr;
	return true;
}

void sensor_reset_timestamp()
{
    int oldnow = rtc_get();

    //rtc_init(); //something wrong here

    chSysLock();


    if (g_timestamp < 0 || g_timestamp < oldnow - MAX_DEADTIME)
    {
        g_timestamp = -1;
    }
    else
    {
        g_timestamp = 0;
    }
    chSysUnlock();


}

/* Check if the NTC sensor is connected, and if it is, update the reading. */
static bool check_ntc()
{
    int temp = ntc_read();
    
    if (temp < -50000 || temp > 300000)
        return false;
    
    int time = rtc_get();
    chSysLock();
    g_temperature = temp;
    g_timestamp = time;
    g_wireless = false;
    chSysUnlock();
    
    return true;
}

/* Read until one reading has been received correctly, or the timeout passes. */
static bool handle_rf_packets()
{
    int i = -1, c;
    uint8_t packet[3];
    int reading = -9999;
    systime_t starttime = chTimeNow();
    
    /* Each packet consists of 4 bytes:
     * 11001100 HLHLHLHL HLHLHLHL HLHLHLHL
     * 
     * The first byte is the constant start character 0xCC.
     * The rest of the bytes have two interleaved words, H and L, where
     * H is the inverse of L. This is done to keep DC offset of the signal
     * at zero, and to provide an error check.
     * 
     * The value of L is the temperature in celcius, in 8.4 fixed point format.
     */
    
    while (chTimeNow() - starttime < S2ST(MAX_DEADTIME) &&
           !chThdShouldTerminate())
    {
        c = sdGetTimeout(&SD1, MS2ST(100));
        
        if (c < 0 && ntc_connected())
            return false;
        
        if (c == 0xCC)
        {
            i = 0; /* Start of packet */
        }
        else if (i >= 0)
        {
            packet[i++] = c;
            
            if (i == 3)
            {
                /* Last byte of packet received, parse it. */
                int v = packet[0] | (packet[1] << 8) | (packet[2] << 16);
                int low = v & 0x555555;
                int high = ((v & 0xAAAAAA) >> 1) ^ 0x555555;
                
                if (low == high)
                {
                    /* Packet is valid, collect the bits */
                    int temp = 0;
                    for (int j = 0; j < 12; j++)
                    {
                        temp |= ((v >> (j * 2)) & 1) << j;
                    }
                    temp = (int16_t)(temp << 4);
                    temp = temp * 1000 / 256;
                    
                    if (reading == temp)
                    {
                        // Ok, got two equal packets
                        int time = rtc_get();
                        
                        chSysLock();
                        g_temperature = temp;
                        g_timestamp = time;
                        g_wireless = true;
                        chSysUnlock();
                    
                        return true;
                    }
                    else
                    {
                        // Wait for another packet to confirm
                        reading = temp;
                    }
                }
                
                i = -1;
            }
        }
    }
    
    return false;
}

static const SerialConfig config = {
    2400,
    0,
    USART_CR2_STOP1_BITS | USART_CR2_LINEN,
    0
};

static BaseSequentialStream *chp;

/**
 * @brief   Prints a decimal unsigned number.
 *
 * @param[in] n         the number to be printed
 */
void test_printn(uint32_t n) {
  char buf[16], *p;

  if (!n)
 	  chSequentialStreamPut(chp, '0');
  else {
	  p = buf;
    while (n)
      *p++ = (n % 10) + '0', n /= 10;
    while (p > buf)
      chSequentialStreamPut(chp, *--p);
  }
}

/**
 * @brief   Prints a line without final end-of-line.
 *
 * @param[in] msgp      the message
 */
void test_print(const char *msgp) {

  while (*msgp)
    chSequentialStreamPut(chp, *msgp++);
}

/**
 * @brief   Prints a line.
 *
 * @param[in] msgp      the message
 */
void test_println(const char *msgp) {

  test_print(msgp);
  chSequentialStreamWrite(chp, (const uint8_t *)"\r\n", 2);
}


int heartbeat(int adc, int* intervalN, int* BP)//, int* intervalN, int* status, int* peak,int* trough,int* thrsh,int* pl )
{
	 Signal = adc; // read the Pulse Sensor

	 sampleCounter += 56; //10 //2 //Increase count->decrease heart rate // keep track of the time in mS with this variable

	 int N = sampleCounter - lastBeatTime;       // monitor the time since the last beat to avoid noise

	    //  find the peak and trough of the pulse wave
	  if(Signal < thresh && N > (IBI/5)*3){       // avoid dichrotic noise by waiting 3/5 of last IBI
	    if (Signal < T){                        // T is the trough
	      T = Signal;                         // keep track of lowest point in pulse wave
	    }
	  }

	  if(Signal > thresh && Signal > P){          // thresh condition helps avoid noise
	    P = Signal;                             // P is the peak
	  }                                        // keep track of highest point in pulse wave

	  //  NOW IT'S TIME TO LOOK FOR THE HEART BEAT
	  // signal surges up in value every time there is a pulse
	  *intervalN=N;
	  if (N > 250){                                   // avoid high frequency noise
	    if ( (Signal > thresh) && (Pulse == false) && (N > (IBI/5)*3) ){
	      Pulse = true;                               // set the Pulse flag when we think there is a pulse
	      //digitalWrite(blinkPin,HIGH);                // turn on pin 13 LED
	      IBI = sampleCounter - lastBeatTime;         // measure time between beats in mS
	      lastBeatTime = sampleCounter;               // keep track of time for next pulse

	      if(secondBeat){                        // if this is the second beat, if secondBeat == TRUE
	        secondBeat = false;                  // clear secondBeat flag
	        for(int i=0; i<=9; i++){             // seed the running total to get a realisitic BPM at startup
	          rate[i] = IBI;
	        }
	      }

	      if(firstBeat){                         // if it's the first time we found a beat, if firstBeat == TRUE
	        firstBeat = false;                   // clear firstBeat flag
	        secondBeat = true;                   // set the second beat flag
	        //sei();                               // enable interrupts again
	        //return;                              // IBI value is unreliable so discard it
	      }


	      // keep a running total of the last 10 IBI values
	      int runningTotal = 0;                  // clear the runningTotal variable

	      for(int i=0; i<=8; i++){                // shift data in the rate array
	        rate[i] = rate[i+1];                  // and drop the oldest IBI value
	        runningTotal += rate[i];              // add up the 9 oldest IBI values
	      }

	      rate[9] = IBI;                          // add the latest IBI to the rate array
	      runningTotal += rate[9];                // add the latest IBI to runningTotal
	      runningTotal /= 10;                     // average the last 10 IBI values
	      BPM = 60000/runningTotal;               // how many beats can fit into a minute? that's BPM!
	      *BP=BPM;
	      g_heart=BPM;
	      return BPM;
	      QS = true;
	     // *status=1;// set Quantified Self flag
	      // QS FLAG IS NOT CLEARED INSIDE THIS ISR
	    }
	  }

	  if (Signal < thresh && Pulse == true){   // when the values are going down, the beat is over
	    //digitalWrite(blinkPin,LOW);            // turn off pin 13 LED
	    Pulse = false;                         // reset the Pulse flag so we can do it again
	    amp = P - T;                           // get amplitude of the pulse wave
	    thresh = amp/2 + T;                    // set thresh at 50% of the amplitude
	    P = thresh;                            // reset these for next time
	    T = thresh;
	   // *thrsh=thresh;
	   // *pl=Pulse;
	    //*status=2;
	  }

	  if (N > 2500){                           // if 2.5 seconds go by without a beat
	    thresh = 712;//512                          // set thresh default
	    P = 712;                               // set P default
	    T = 712;                               // set T default
	    lastBeatTime = sampleCounter;          // bring the lastBeatTime up to date
	    firstBeat = true;                      // set these to avoid noise
	    secondBeat = false;
	    //*status=3;// when we get the heartbeat back
	  }

	  //sei();                                   // enable interrupts when youre done!
}

/** @} */
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int TestThread2(){//LM35 adc //void *p
	palClearPad(GPIOA,GPIOA_LM35);
	static const ADCConversionGroup convgrp = {
		        0,
		        1,
		        NULL,
		        NULL,

		        // CR
		        //0,
				ADC_CR1_RES_0,//set control register 1 in ADC to 10 bit resolution
				ADC_CR2_SWSTART | ADC_CR2_DELS_0,

		        // SMPR
		        0,
		        0,
		        ADC_SMPR3_SMP_AN0(ADC_SAMPLE_16),

		        // SQR
		        ADC_SQR1_NUM_CH(1),
		        0,
		        0,
		        0,
		        ADC_SQR5_SQ1_N(ADC_CHANNEL_IN5)//set ADC as pin PA4 (IN4) (HAS BEEN PULLED DOWN IN BOARD.H)
		    };
		  int adc_oversample()
		    	  {
		    	      int sum = 0;
		    	      // Oversample by 256 and divide by 16 to get 4 more bits of accuracy.
		    	      for (int i = 0; i < 16; i++)
		    	      {
		    	          adcsample_t buf[16];

		    	          adcAcquireBus(&ADCD1);
		    	          adcConvert(&ADCD1, &convgrp, buf, 16);
		    	          adcReleaseBus(&ADCD1);

		    	         /* for (int j = 0; j < 16; j++)
		    	          {
		    	              sum += buf[j];
		    	          }*/
		    	          sum = buf[0];
		    	      }

		    	      return sum;
		    	      //return (sum + 8) / 16;
		    	  }
		    int val2 = adc_oversample();
		    double temp = (val2*300)/1024;
		    g_temp=temp;

		   // int delaytime = ST2MS(delay);//system ticks to ms

	 // chp = p;
	  //int a=1;
	  //return val;

	 // test_println("");

	  //test_printn(temp);
	  //chThdSleepMilliseconds(100);
return temp;
}

float TestThread3(){//GSR adc //void *p
	palClearPad(GPIOC,GPIOC_GSR);
	static const ADCConversionGroup convgrp = {
		        0,
		        1,
		        NULL,
		        NULL,

		        // CR
		        0,
				//ADC_CR1_RES_0,//set control register 1 in ADC to 10 bit resolution
				ADC_CR2_SWSTART | ADC_CR2_DELS_0,

		        // SMPR
		        0,
		        0,
		        ADC_SMPR3_SMP_AN0(ADC_SAMPLE_16),

		        // SQR
		        ADC_SQR1_NUM_CH(1),
		        0,
		        0,
		        0,
		        ADC_SQR5_SQ1_N(ADC_CHANNEL_IN10)//set ADC as pin PA4 (IN4) (HAS BEEN PULLED DOWN IN BOARD.H)
		    };
		  int adc_oversample()
		    	  {
		    	      int sum = 0;
		    	      // Oversample by 256 and divide by 16 to get 4 more bits of accuracy.
		    	      for (int i = 0; i < 16; i++)
		    	      {
		    	          adcsample_t buf[16];

		    	          adcAcquireBus(&ADCD1);
		    	          adcConvert(&ADCD1, &convgrp, buf, 16);
		    	          adcReleaseBus(&ADCD1);

		    	        /*  for (int j = 0; j < 16; j++)
		    	          {
		    	              sum += buf[j];
		    	          }*/
		    	          sum = buf[0];
		    	      }

		    	      return sum;
		    	     // return ((sum + 8.0) / 16.0);
		    	  }
		  float volt;
		 // for(int j=0;j<16;j++)
		 // {
		  float val2 = adc_oversample();
		  volt = (val2*3.0)/4096.0;

		 // }
		  g_gsr=volt*100;
		  // g_temp=temp;

		   // int delaytime = ST2MS(delay);//system ticks to ms

	 // chp = p;
	  //int a=1;
	  //return val;

	 // test_println("");

	  //test_printn(temp);
	  //chThdSleepMilliseconds(100);
return (volt);
}


void sensorStuff(void *p){//pulse sensor


	//int curtime = osalOsGetSystemTimeX();
		//static int delay;

		//parameters for ADC
		//Reference: adc_IId.h (Search: ADCConversionGroup) and
		//stm32l152xb.h (Search: Bit definition for ADC_CR1 register)
			static const ADCConversionGroup convgrp = {
		        0,
		        1,
		        NULL,
		        NULL,

		        // CR
		        //0,
				ADC_CR1_RES_0,//set control register 1 in ADC to 10 bit resolution
				ADC_CR2_SWSTART | ADC_CR2_DELS_0,

		        // SMPR
		        0,
		        0,
		        ADC_SMPR3_SMP_AN0(ADC_SAMPLE_16),

		        // SQR
		        ADC_SQR1_NUM_CH(1),
		        0,
		        0,
		        0,
		        ADC_SQR5_SQ1_N(ADC_CHANNEL_IN1)//set ADC as pin PA1 (IN1) (HAS BEEN PULLED DOWN IN BOARD.H)
		    };
		  int adc_oversample()
		    	  {
		    	      int sum = 0;
		    	      // Oversample by 256 and divide by 16 to get 4 more bits of accuracy.
		    	      for (int i = 0; i < 16; i++)
		    	      {
		    	          adcsample_t buf[16];

		    	          adcAcquireBus(&ADCD1);
		    	          adcConvert(&ADCD1, &convgrp, buf, 16);
		    	          adcReleaseBus(&ADCD1);

		    	         /* for (int j = 0; j < 16; j++)
		    	          {
		    	              sum += buf[j];
		    	          }*/
		    	          sum = buf[0];
		    	      }

		    	      return sum;
		    	      //return (sum + 8) / 16;
		    	  }
		    int val = adc_oversample();


		   // int delaytime = ST2MS(delay);//system ticks to ms

	  chp = p;
	  //int a=1;
	  //return val;
	 // palSetPad(GPIOB, 6);
	 		          //chThdSleepMilliseconds(50);/////////
int tempp=TestThread2();
int gsr=TestThread3()*10000;
heart = heartbeat(val,&intervalN,&BP);//,&intervalN,&status,&peak,&trough,&thrsh,&pl);// input adc and pointer (to store interval b/w heart beats)
//chThdSleepMilliseconds(8);//8 when delay increases, heart rate increases.
//test_println("");
	  //test_println("*** ChibiOS/RT test suite");
	  //test_printn(val);
	  //test_print("--");


	  // test_printn(delaytime);
	 // test_print("--");

	 // chprintf(chp,"%d",heart);

	  test_printn(BP);
	  test_print("--");

	  test_printn(intervalN);
	  test_print("--");
	  test_printn(tempp);
	  	  test_print("--");
	    test_printn(gsr);
	    test_println("--");

	     //   test_printn(thrsh);
	        //test_print("--");

	            //test_printn(pl);*/

	  const int sensorMin = 0;      // sensor minimum, discovered through experiment
	  const int sensorMax = 1024;    // sensor maximum, discovered through experiment

	    int sensorReading = val;
	    // map the sensor range to a range of 12 options:
	  //  int range = map(sensorReading, sensorMin, sensorMax, 0, 11);

	    // do something different depending on the
	    // range value:
	    /////ASCII Art Madness
	   /* switch (range) {
	    case 0:
	      test_println("");
	      break;
	    case 1:
	    	test_println("---");
	      break;
	    case 2:
	    	test_println("------");
	      break;
	    case 3:
	    	test_println("---------");
	      break;
	    case 4:
	    	test_println("------------");
	      break;
	    case 5:
	    	test_println("--------------|-");
	      break;
	    case 6:
	    	test_println("--------------|---");
	      break;
	    case 7:
	    	test_println("--------------|-------");
	      break;
	    case 8:
	    	test_println("--------------|----------");
	      break;
	    case 9:
	    	test_println("--------------|----------------");
	      break;
	    case 10:
	    	test_println("--------------|-------------------");
	      break;
	    case 11:
	    	test_println("--------------|-----------------------");
	      break;

	    }*/



	  //QS=false;



	  //int pastime = osalOsGetSystemTimeX();
	  //delay = pastime - curtime;
}


static Thread *sensorThread = 0;
static WORKING_AREA(sensorThread_wa, 256);

static msg_t sensor_task(void *arg)
{


	  chRegSetThreadName("blinker");

	  while (true) {

		  sensorStuff(&SD1);
		 // palSetPad(GPIOB, 6);
		          chThdSleepMilliseconds(50);//30,150
		          //palClearPad(GPIOB, 6);
		          //chThdSleepMilliseconds(200);

	 }

	/*chRegSetThreadName("sensor");
    
    sdStart(&SD1, &config);
    
    while (!chThdShouldTerminate())
    {
        int sleep = 5;
        if (ntc_connected())
        {
            check_ntc();
        }
        else
        {
            if (!g_wireless)
            {
                chSysLock();
                g_timestamp = -1;
                chSysUnlock();
            }
            
            palSetPad(GPIOA, GPIOA_RF_POWER);
            bool status = handle_rf_packets();
            palClearPad(GPIOA, GPIOA_RF_POWER);
            
            if (status)
            {
                // After successful read, we can sleep a bit longer
                sleep = MIN_INTERVAL;
            }
            else
            {
                sleep = 0;
            }
        }
        
        // Sleep while monitoring any requests to shut down the thread
        for (int i = 0; i < sleep * 10; i++)
        {
            chThdSleepMilliseconds(100);
            if (chThdShouldTerminate())
                break;
        }
    }
    
    sdStop(&SD1);*/
	  //osalInit
    return 0;
}

void sensor_start()
{
    if (sensorThread != NULL)
        return;
    
    sensorThread = chThdCreateStatic(sensorThread_wa, sizeof(sensorThread_wa),
                                     NORMALPRIO + 10, sensor_task, NULL);
}

void sensor_stop()
{
    if (sensorThread)
    {
        chThdTerminate(sensorThread);
        chThdWait(sensorThread);
        sensorThread = NULL;
    }
}
