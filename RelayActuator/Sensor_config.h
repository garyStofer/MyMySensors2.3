 // Overarching include file enables NRF24 transport and contains the PAN_ID plus channel number and other 
 // vital parameters of the PAN
#include "MyRadio_NRF24.h"  // This file resides in a "fake" library "~/Arduino/library/MyMYsenors" so that it can be included by all the nodes 
                             // of the PAN
                             // It needs to be included before MySensor.h
//#undef  MY_RF24_IRQ_PIN  // for nodes not having the IRQ line from the radio connected to IRQ0 (pin2) see MyRadio_NRF24.h
#define SKETCH_VERSION "1.2"
// Enable repeater functionality for this node
// #define MY_REPEATER_FEATURE
// #define MY_NODE_ID 0x4
// #define MY_DEBUG_VERBOSE_RF24
// #define MY_SPECIAL_DEBUG
// #define MY_DEBUG