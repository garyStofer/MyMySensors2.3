// Over-arching include file enables NRF24 transport and contains the PAN_ID plus channel number and other vital parameters of the PAN

#define SKETCH_VERSION "1.7"

#include "MyRadio_NRF24.h"  // This file resides in a "fake" library "~/Arduino/library/MyMYsenors" so that it can be included by all the nodes of th PAN 
                            // It needs to be included before MySensor.h
//#undef  MY_RF24_IRQ_PIN  // for nodes not having the IRQ line from the radio connected to IRQ0 (pin2)
#undef MY_DEBUG
//#define MY_DEBUG          // turns on Mysensor library debug output -- see other debug defines in MyRadio_NRF24.h


//#define PIR           // For PIR sensor devices -- comment out for Wiretrip

//#define AC_POWERED  // for non Battery nodes running from AC and having the CPU running at 5V/16mhz
                      // undefine AC_POWERED for battery operation 
//#define STATUS_TOGGLE // for fast and noisy switches, such as IR barrier where the contact pulse can be only a few millsoconds long and
                        // therefore the open/closed status is transient in nature.  It transmits an open/closed transitio each time the contact changes
                      
#ifdef PIR
  #undef STATUS_TOGGLE  // the PIR sensor has an ON delay built in, adjustable with potentiometer
#endif   

#define WITH_SWITCH   // This adds a relay switch 

#ifdef WITH_SWITCH    // switch output for alarm siren etc
  #define AC_POWERED    // nodes with relay switch must be AC powered -- they can not sleep and be woken up by a pin change event
  #define CHILD_2 2
  #define RELAY_PIN  17  // aka A3, PC3, ADC3 -- chip pin 26
#endif
