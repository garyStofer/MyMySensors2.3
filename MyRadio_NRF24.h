// Enable and select radio type attached
#define MY_RADIO_RF24
#define MY_RF24_IRQ_PIN (2)
#ifdef MY_RF24_BASE_RADIO_ID
#error Include sequence wrong, MyRadio_NRF24.h must be first
#endif
#define MY_RF24_BASE_RADIO_ID 0x00,0x71,0x0f,0xfe,0xca    //My own PAN-ID  At RT
//#define MY_RF24_BASE_RADIO_ID 0x00,0x71,0x0f,0x33,0x2a// My own PAN-ID -- alternate
//#define MY_RF24_CHANNEL  76   //this is the default 
#define MY_RF24_CHANNEL  112 //2520 mhz -- outside of regular wifi channels
#define MY_RF24_DATARATE RF24_250KBPS // this is the default 250KBPS is the slowest
#define MY_RF24_PA_LEVEL RF24_PA_MAX
// chices are :  
//  #define MY_RF24_PA_LEVEL RF24_PA_HIGH
//  #define MY_RF24_PA_LEVEL RF24_PA_MAX
//  #define MY_RF24_PA_LEVEL RF24_PA_LOW
//  #define MY_RF24_PA_LEVEL RF24_PA_MIN

#define MY_BAUD_RATE 57600    // slow down because this node might run at 8Mhz 
//#define MY_BAUD_RATE 115200

// Enable debug prints to serial monitor
// #define MY_DEBUG_VERBOSE_RF24
// #define MY_SPECIAL_DEBUG
// #define MY_DEBUG