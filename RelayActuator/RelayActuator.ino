/**  For MySensor library  V2.3.2 and built under Arduino 2.0.4
  
 *   Sensor node that acts as a single or dual light or appliance switch 
 *   Push button on node controls relay #1 only. Push button control works locally since feedback via controller is not always reliable or speedy. 
 *     
  
 */

#include "EEPROM.h"
#include "Sensor_config.h"


 
#include <MySensors.h>
#define SWITCH_PIN 7
#define LED_PIN	 8	    // PB0, Mega328P device pin 12
//#define DUAL  				// for two relays

#define RELAY_ON 	1  	// GPIO value to write to turn on attached relay
#define RELAY_OFF 0 	// GPIO value to write to turn off attached relay

#define RED_LED_ON  {pinMode(LED_PIN, OUTPUT);digitalWrite(LED_PIN, HIGH);}  
#define BLUE_LED_ON {pinMode(LED_PIN, OUTPUT);digitalWrite(LED_PIN, LOW);} 
#define LEDS_OFF    {pinMode(LED_PIN, INPUT);} // red LED

#define CHILD_ID1   1
#define RELAY_PIN1  16  // aka A2, PC2, ADC2 -- chip pin 25
MyMessage msg1(CHILD_ID1, V_STATUS); 

#ifdef DUAL 
#define CHILD_ID2   2
#define RELAY_PIN2  17  // aka A3, PC3, ADC3 -- chip pin 26
MyMessage msg2(CHILD_ID2, V_STATUS);   
#endif

// NOTE: The various wait(50) are needed if the protocol handshake is enabled in the previous transmission to te controller,
// otherwise the following transmission gets potentially blanked by the ACK message being transmitted from the gateway
// best to not have the protocol level ack feature enabled on any calls 

static void ChangeRelays(const MyMessage& msg)
{
   // Change relay states
   bool state= msg.getBool();   
   Serial.print( "Switch");
   
  if (msg.sensor == CHILD_ID1)
  {
    Serial.print( "1: ");
    Serial.println( state);
    msg1.set(state);    // record the state in local msg1 structure so that the button toggle can be done locally
    digitalWrite(RELAY_PIN1, state );
      
    if (state)
      RED_LED_ON
    else
      BLUE_LED_ON
  }
#ifdef DUAL
  else if (msg.sensor == CHILD_ID2)
  {
    Serial.print( "2: ");
    Serial.println( state);
    msg2.set(state);
    digitalWrite(RELAY_PIN2, state );
   
  }
#endif

}

void before()
{
  Serial.println("Device Startup in before().. waiting for network..");
  RED_LED_ON;
  delay (50);
  LEDS_OFF;

  // Make sure relays are off when starting up
  digitalWrite(RELAY_PIN1, RELAY_OFF);
  pinMode(RELAY_PIN1, OUTPUT);

#ifdef DUAL    
  digitalWrite(RELAY_PIN2, RELAY_OFF);
  pinMode(RELAY_PIN2, OUTPUT);
#endif  

  pinMode(SWITCH_PIN, INPUT_PULLUP); 
  
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

void presentation()
{
 bool ret;
	// Send the sketch version information to the gateway and Controller
	sendSketchInfo("AC-Relays with Switch",SKETCH_VERSION);
 
  // Register sensors to node
  Serial.print( "Present S1 ... ");
  ret = present(CHILD_ID1, S_BINARY, "AC-SW1",false); 
  // wait(50);  // needed if present() is beeing called with 3rd argument = true ( protocol ACK)
  Serial.print( "returned: "); Serial.println( ret);

  
#ifdef DUAL
  Serial.println( "Present S2 ... ");
  ret = present(CHILD_ID2, S_BINARY, "AC-SW2",false);
  // wait(50);  // needed if present() is beeing called with 3rd argument = true ( protocol ACK)
  Serial.print( "returned: "); Serial.println( ret); 

#endif  

    
}

void setup()
{
  bool ret;
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
  
  Serial.print("Sensor Setup() -- Node ID:");
  Serial.println( getNodeId() );

  // to set the relays with the current status on the controller we issue a request for status up front
  ret = request(CHILD_ID1, V_STATUS);
  Serial.print( "request for S1 in setup returned: "); Serial.println( ret);
  

#ifdef DUAL
  // to set the relays with the current status on the controller we issue a request for status up front
  wait (200) ; //allow the first request to cmplete 
  ret =request(CHILD_ID2, V_STATUS);
  Serial.print( "request for S2 in setup returned: "); Serial.println( ret);
  
#endif  

  Serial.flush();
}

void loop()
{
	bool  ack;
 // NOTE: getting a reply back from the GW after sending a toggled state doesn't always work because of potential range issues.

// To fix this the code directly changes the local state variable and controls the relays immediatly instead of waiting for a
// reply back from the controller/GW. Should a reply be received then the switch simply gets set again to the same state as it's already at.
// If the outgoing "set" message to the controller did not make it but the "request" got through then the relays flicks momentairly. This is an indication
// that the connection to the controller is weak, however if the request message is lost then no indication is present. 

  if (digitalRead(SWITCH_PIN) == 0)  // affects relay 1 
  {
    bool ret;
    bool state1 = (msg1.getBool() ? false : true);  // Toggle 
    msg1.set(state1);
    ChangeRelays(msg1);  // affect relay immediatly -- so that the switch still works when the gateway or controller is off line 
    
    wait(500); // pause for debounce, then wait until the button is released
    while( digitalRead(SWITCH_PIN) == 0)
      wait(10); 
  
    Serial.println( "Send new status for child 1 to controller");
    ack = send(msg1, true); // second argument is ACK request, if device for this message was setup for "echo" in the call to present
                            // then true means a Protocol level ACK is going to be performed. if present() was alled with false (no echo) 
                            // then a true here returns the single-hop HW ack from the radio hardware itself.    
    if (! ack)
      Serial.println(" 1st hop Nack on sending status to GW");
   
 
    // now send out a request for status to the controller -- if all communication went well the controller sends the status back 
    // which gets processed by the receive function and since the status has not changed the relay just gets commanded again to the same state.
    // If however the communcation to the gateway is not good, then then possible the opposite state gets commanded on the relay which is then
    // an indication to the user that the link is iffy.
     
    wait(50);  // needed if send() above is beeing called with 2nd argument = true (request ACK)
    ret = request(CHILD_ID1, V_STATUS);
    Serial.print( "request for child1 status in loop returned: "); Serial.println( ret);
    Serial.flush();  

  
  }
}

void receive(const MyMessage &message)
{
	// We only expect one type of message from controller. But we better check anyway.
  if (message.isAck())
  {
    Serial.println("This is an ack from gateway");
  }
  else if (message.type == V_STATUS)
  {
   // Write some debug info
    Serial.print("Incoming msg for Relay:");
    Serial.print(message.sensor);
    Serial.print(", New status: ");
    Serial.println(message.getBool());
    ChangeRelays(message);
  }
  else 
  {
    Serial.print("Unknown message received");
    Serial.println(message.type);    
  }

}

