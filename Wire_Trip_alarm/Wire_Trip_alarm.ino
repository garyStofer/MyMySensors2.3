/* WIRE_TRIP and PIR Sensor node for MySensor library  >= V2.3.2 and built under Arduino >= 1.8.19

  MySensor Node application for wire trip or PIR sensor running on Mega328P powered by 2 AA  Lithium or  2AA Alkaline cells for > 2years.
	Standby current consumption is ~< 18uA due to the 200kohm pull up resistor on the signal pin.
 
* The trip wire keeps the signal pin at logic low. When the wire is broken the signal is pulled high throug an external pull-up. 
* In the case when a PIR sensor is attached, the signal pin will go from low to high when motion is detected. 
* This can also be used with window and door switches or IR-beam sensors etc..
*	PCB can be ordered at OSHPCB.com search for "nrfMysensor" in shared projects, Eagle files at https://github.com/garyStofer/Eagle5_PCB/tree/master/nRF_Mysensor
* Uses PinChange interrupt feature on PortC so that sensor gets triggered on rising AND falling edge of signal (INT0/1 will only do one or the other on 328p)
* Rising edge sets the alarm state, falling edge clears it -- or the state is toggled depending on #define "STATUS_TOGGLE". 

* Atmel 328p runs at @3.3V @8mhz internal clock with standard optiboot loader 

************************* IMPORTANT ************************************ 
* Use 56K-baud or less when running at 8Mhz and internal clock 
* Must set Brownout voltage to 1.8V and internal clock 8mhz with 4ms startup delay using Atmel Studio programmer with ICE-MKII after loading Optiboot 32Pin bootloader.
* If Brownout voltage is left at default 2.7V battery will not be fully used up.
* If 8Mhz startup delay is left at 65 ms the bootloader might not properly connect when device was in sleep mode. 
* FUSES : 0xFE,0XDE,0xD2 ( for opti boot)
 


  If hooking this up with a arduino pro or pro mini follow these instructions -- Better to implement it with the PCB/shematics listed below.
	 
  -  Mega328P must be setup to run at 8Mhz internal clock for max reliability and minimum power consumption. See above 
  -  Signal input is using pin change interrupt on port PC. Therefore Analog input A0 (PC0,ADC0) is not available
  -  Standard Arduino power LED and status LED on pin PB5 (SCK) needs to be removed for minimum power consumption.
  -  Input voltage regulator removed, feed B+ to VCC directly. Radio
  -  Standard Radio connection, VCC,D9,D10,MOSI,MISO,SCK,GND for nRF24L
  -  Voltage divider 1M/470K 1% between B+ and BATTERY_V_DIV_GND_PIN for minimum power consumption.
  -  Red LED D8 to GND for loss of RF communication indication. -- Only failure to talk to the parent node is indicated (1st hop).
  -  White or Blue LED VCC to D8 for other indication.
  -  Battery life expectancy with two AA is >2 years
  -- MUST set Brownout fuse to 1.8V in order to run battery down sufficiently, see above.
	
	See schematis in Eagle files at https://github.com/garyStofer/Eagle5_PCB/tree/master/nRF_Mysensor
	PCB can be ordered at OSHPCB search for "nrfMysensor" in shared projects
*/
#include <EEPROM.h>   // for the node id storage

#include "Sensor_config.h"  // local configuration for items concerning this sensor only



 // This is to prevent a battery powered node from emptying its battery when there is no gateway to talk to upon startup.
#define MY_TRANSPORT_WAIT_READY_MS (60000ul) 
// This aborts trying to connect to a networks after given time. Execution will advance to "Setup()" after n mseconds
// The connection status can then be evaluated and the node put into permanent sleep mode.
// In order for this battery node to not go into an infinit loop of searching for a new parent if the controller happens to be off 
// we set the parent statically as the controller node (i.e. 0).  If the controller is down, this node will then fail the 
// transmission but won't start a search for new parent (SNP), exhausting the battery in the process if none is found.
#define MY_PARENT_NODE_IS_STATIC			
#define MY_PARENT_NODE_ID 0
// #define MY_NODE_ID 11  // in case you want to make it a specific node ID instead of getting one from the controller, EEPROM reset is required to register the new ID

#undef MY_RF24_PA_LEVEL
#ifdef AC_POWERED
  #define MY_RF24_PA_LEVEL RF24_PA_MAX
  #define BATTERY_REPORT_INTERVAL 2
  #ifdef PIR
    const char *sketch = "AC-PIR-Sensor";
  #else
    const char *sketch = "AC-Trip-Sensor";
  #endif  
#else
  #define MY_RF24_PA_LEVEL RF24_PA_HIGH
   #ifdef PIR
    const char *sketch = "BAT-PIR-Sensor";
     #define BATTERY_REPORT_INTERVAL 24
   #else
    const char *sketch = "BAT-Trip-Sensor";
    #define BATTERY_REPORT_INTERVAL 1
  #endif
#endif

// do not move this further up 
#include <MySensors.h>      // Requires MySensor v2.3.1 or higher  -- must follow include file MyRadio_NRF24
#ifdef MY_REPEATER_FEATURE 
#error This node is not comaptible with Reapeater mode
#endif

#define CHILD_1 1
MyMessage msg(CHILD_1, V_TRIPPED);

#ifdef WITH_SWITCH
MyMessage msg2(CHILD_2, V_STATUS);   
#endif

#define BATTERY_SENSOR_ANALOG_PIN 7// ADC7, Mega328P device pin 22, this is ANALOG pin 7
#define BATTERY_V_DIV_GND_PIN	  7	 // PD7, Mega328P device pin 11, this is DIGITAL pin 7  	

#define SWITCH_PIN BATTERY_V_DIV_GND_PIN  // This is the same as the BATTERY_V_DIF_GND_PIN.  battery measure voltage divider serves as the pull-up for the switch
#define LED_PIN		 		   8	// PB0, Mega328P device pin 12
#define Signal_PIN 		  	  14    // PC0, aka ADC0, aka A0, Mega328P device pin 23  -- Do not move this pin -- Pin-Change code below sets up PC0 as input


#define ADC_BITVALUE (1.1f / 1024)    // ADC using 1.1V internal reference
#define ADC_BATTERY_DIV  (3.125f)     // external voltage divider ratio 1M:470K
#define BATTERY_EMPTY (2.6f)
#define BATTERY_FULL  (3.15f)		    // A brand new Alkaline battery has about 1.6V 
//#define BATTERY_REPORT_INTERVAL 1 	// report the battery status only every nth cycle for lower power consumption on trip wire with door or window switch
// already defined above 			 	// set this number to low number or 1 if there is no door or window switch in the trip wire that frequently activates the node


#define RED_LED_ON  {pinMode(LED_PIN, OUTPUT);digitalWrite(LED_PIN, HIGH);}  
#define BLUE_LED_ON {pinMode(LED_PIN, OUTPUT); digitalWrite(LED_PIN, LOW);} // blue 
#define LEDS_OFF    {pinMode(LED_PIN, INPUT);} // red LED



// Pin change interrupt to capture the event from the sensor
static  short AlarmInstances = 0;     // init so that it sends the battery status on first alarm after startup
static byte Signal_pin_Changed = 0;   // Flag to allow operation without sleep in main loop 
ISR(PCINT1_vect)  // ISR for pinchange interrupt on port C
{
	  Signal_pin_Changed=true;
}
//  presentation callback function for Mysensors
void presentation()  //  is called when the controller requests info about the nature of the device 
{

// Send the sketch version information to the Gateway and Controller
  sendSketchInfo(sketch, SKETCH_VERSION);
 // Register all sensors to gateway (they will be created as child devices)
 #ifdef PIR
  present(CHILD_1, S_MOTION,"PIR",false);   // gives the sensor a meaningfull name so that domotics can display something upon discovery
 #else
  present(CHILD_1, S_MOTION,"Wire-Trip",false);   // gives the sensor a meaningfull name so that domotics can display something upon discovery
#endif

#ifdef WITH_SWITCH
  present(CHILD_2, S_BINARY, "Switch",false);
#endif
}

// before callback funtion for Mysensors
void before()		// executes before the radio starts up
{
  Serial.print(sketch); Serial.print(": ");
  Serial.println(SKETCH_VERSION );
#ifdef WITH_SWITCH
  Serial.println("With Switch");  
#endif
 // Serial.print("Mysensor Version: ");   // already printed our by library logo print
 // Serial.println(MYSENSORS_LIBRARY_VERSION);
  Serial.println("Device Startup... waiting for network");
  RED_LED_ON;
  delay (50);
  LEDS_OFF;
  
  pinMode(SWITCH_PIN, INPUT); // Not INPUT_PULLUP since this is also the voltage divider gnd pin for the battery measurment and therefore has a pullup resistor on the HW

#ifdef WITH_SWITCH
  // for switch output
  digitalWrite(RELAY_PIN, 0);
  pinMode(RELAY_PIN, OUTPUT);
#endif  

  // Erasing the EEprom of the chip -- Node will loose it's ID number, Gateway and Controller will furnish a new ID number
  if (digitalRead( SWITCH_PIN ) == 0 )
  {
    Serial.print("Erasing EEPROM\n");
   
    for (int i = 0; i < 512; i++)
      EEPROM.write(i, 0xff);

    Serial.println("Clearing of EEprom complete.");
  }
  BLUE_LED_ON;  // white or blue LED
   // between "before" and "setup" the radio and network is beeing initialized by MySensor lib.
   

}
void setup() // after Radio connected 
{
  

  if ( ! isTransportReady()) // if the init of the network failed because of timing out do to missing gateway controller. 
  {
    Serial.print("\nNo network to connect to -- STOP\n");
    Serial.flush();
    RED_LED_ON;
    delay (600);
    LEDS_OFF;
    hwPowerDown(WDTO_SLEEP_FOREVER);	 // This will power down the node indefinetly.  Cycle power to restart.
  }
  
   // when here we should have connection with the gateway/controller
  // blink BLUE led to confirm that the connection is established 
  
  for( int i=0; i<4; i++)
  {
    LEDS_OFF; 
    delay(100);
    BLUE_LED_ON;
    delay(15);
    LEDS_OFF;
  } 
  
#ifdef PIR
  Serial.print("PIR Sensor Setup() -- Node ID:");
#else
   Serial.print("Trip Sensor Setup() -- Node ID:");
#endif   
  Serial.println( getNodeId() );
  Serial.flush();

  pinMode(BATTERY_V_DIV_GND_PIN, INPUT);
  digitalWrite(BATTERY_V_DIV_GND_PIN, LOW);   // bottom of V-divider for battery measurement
 
  analogReference( INTERNAL);  // on mega328 this is 1.1V
  analogRead(BATTERY_SENSOR_ANALOG_PIN);	// This needs to be done once upfront to setup the ADC before the first reading is taken

  // Setting up the pin change interrupt of Port C

#ifdef AC_POWERED
  pinMode(Signal_PIN, INPUT_PULLUP);  // for AC nodes -- pull up faster with internal 35Ko resistor  
#else
#ifndef PIR
  Serial.println("Battery pwrd nodes need external 200K pullup");
#endif  
  pinMode(Signal_PIN, INPUT);       // PC0, aka A0, aka digital pin14, aka device pin23  -- INPUT without pullup to trigger alarm when external wire to gnd is broken
#endif                              // built in pullup is about 35Ko, This draws about 95us from the 3.0V battery constantly. Use external 200Ko resistor to reduce the 
                                    // constant current load for an unbroken trip wire. 

  // now enable the pin change interrupt for PC0, 
  PCMSK1 |= 1 << PCINT8; // enable PCinterupt 8 , aka PC0, aka A0, aka pin14
  PCICR  |= 1 << PCIE1; // enable PCinterupt 1 vector

#ifdef WITH_SWITCH                                    
  request(CHILD_2, V_STATUS);       // sync the switch with the current state in the controller
#endif
}


void loop()
{
  unsigned int ADC_count;  // for battery reading
  static int BatteryPcnt;  // calc the percent charge of battery --
  float BatteryV;
  unsigned char ack_fail =0;
 
  LEDS_OFF;			// LEDS off
  //Serial.print("loop.\n");

  // The NODE sleeps here until pin-change happens -- This requires a 328P as in a PicoPower device 
  // Pinchnage interrupt on portC pin is discretly enabled in the setup(). 

#ifndef AC_POWERED // if battery powered we go to sleep here until woken up by pinchange 
  sleep(0);  // sleep(0) got fixed for 2.3.2 version of MySensor
  // when "HERE" the node woke up because of the pin change interrupt
#endif

 if ( Signal_pin_Changed == false)  // this is for the non sleep case where we contineously loop so we can receive switch commands from the controller
  return;
 
 #ifdef STATUS_TOGGLE
    Serial.print("Tripped\n");
    
    msg.set(1);    // send the ON state 
    if (!send(msg, false))    // this returns the state of the HW ACK to the next node (1st hop) -- this only works for single hop networks,
                            // second argument is protocol level ACK, but that requires that the message receive function is implemented and the recived message is checked.
      ack_fail++;
      
    // wait a bit, then send the off state    
    delay(1000);
    msg.set(0); 
    if (!send(msg, false))    
      ack_fail++;

   

  #else      // report steady status
    delay(500); //for de-bouncing of the switch/trip wire 
 
    if (digitalRead(Signal_PIN) )	//Rising edge of  status signal. i.e. tripped
    {
      Serial.print("SET\n");
      msg.set(1);    // The status of the  Sensor output
    }
    else
    {
      Serial.print("CLEAR\n");
      msg.set(0);    // The status of the Sensor output
    }
 
    if (!send(msg, false))    // this returns the state of the HW ACK to the next node (1st hop) -- this only works for single hop networks,
                              // second argument is protocol level ACK, but that requires that the message receive function is implemented and the recived message is checked.
      ack_fail++;

  #endif // STATUS_TOGGLE

   if (ack_fail) // the gateway did not hear us -- range issue or gateway is off
   {
      Serial.print("Failed to HW-ack\n");
      RED_LED_ON;  	// set the red LED indicating failure to communicate with parent (gateway or router)
      delay(200);   // so I can see it
      return;
   }

  // send Battery status every Nth time
  if (AlarmInstances++ % BATTERY_REPORT_INTERVAL == 0 ) // 
  {
 
    Serial.print("Battery voltage,Percent: ");
               
    // setup the voltage divider for the battery measurement
    digitalWrite(BATTERY_V_DIV_GND_PIN, LOW);   // bottom of V-divider for battery measurement
    pinMode(BATTERY_V_DIV_GND_PIN, OUTPUT);     // activate the voltage divider by pulling the low end to GND

    BLUE_LED_ON;	// LED on for battery load
	  delay(500);  // for cap charge up

    ADC_count = analogRead(BATTERY_SENSOR_ANALOG_PIN);
    LEDS_OFF; 

    BatteryV = ADC_count *  ADC_BITVALUE * ADC_BATTERY_DIV;
	  Serial.print(BatteryV);
	  Serial.print(", ");
    BatteryPcnt = 100 * (BatteryV - BATTERY_EMPTY) / (BATTERY_FULL - BATTERY_EMPTY);
	  if (BatteryPcnt > 100 )
      BatteryPcnt = 100;

    if (BatteryPcnt < 0 )
      BatteryPcnt = 0;
    Serial.println(BatteryPcnt);
    pinMode(BATTERY_V_DIV_GND_PIN, INPUT);      // deactivate the voltage divider by letting it float

    sendBatteryLevel(BatteryPcnt);
    
  }
  Signal_pin_Changed = false;   // reset the flag now that the alarm signal pin change has been processed
}

#ifdef WITH_SWITCH
// function to receive switch commands from the controller
void receive(const MyMessage &message)
{
	// We only expect one type of message from controller. But we better check anyway.
  if (message.isAck())
  {
    Serial.println("This is an ack from the gateway");
  }
  else if (message.type == V_STATUS && message.sensor == CHILD_2 )
  {
    Serial.print("Incoming msg for Relay:");
    Serial.print(", New status: ");
    Serial.println(message.getBool());
    digitalWrite(RELAY_PIN, message.getBool()); // change relay state
  }
  else 
  {
    Serial.print("Unknown message received");
    Serial.println(message.type);    
  }
}

#endif
