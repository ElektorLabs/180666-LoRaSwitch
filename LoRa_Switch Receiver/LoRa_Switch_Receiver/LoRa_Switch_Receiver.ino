/****************************************************************************************
 * 
 * 
 * 
 * 
 * Librarys requierd: 
 * -CRC32 by Christopher Baker
 * -Crypro by Rhys Weatherley
 * -MCCI LoRaWAN LMIC library by IBM, Matthias Kooijman, Terry Moore, ChaeHee Won, Frank Rose ( Version 2.3.1 ) 
 * -U8G2 by Oliver
 * 
 * 
 * 
 */

/* Includes */
#include <lmic.h>
#include <hal/hal.h>
#include <U8g2lib.h>
#include <SPI.h>
#include <Wire.h>

#include <EEPROM.h>
#include <CRC32.h>
#include <Crypto.h>
#include <AES.h>
#include <SHA256.h>
#include <string.h>

#include "serialcli.h"

/*
 * Pin-Mapping for the RFM95 LoRa Module
 */
const lmic_pinmap lmic_pins = {
    .nss = PA4,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = PA0,
    .dio = {PB10, PB1, PB0},
};

/*
 * Also we will have some defines for the other may attached 
 * ICs on the baord
 */

#define NOR_NCS PB11
#define VBAT_IN PA1

#define GPS_ACRIVE_ON PC13

//This is specific for the LoRaSwitch TX part
#define SWITCH      PB15
#define RELAY_SET   PB12
#define RELAY_RESET PB13
#define AC_POWER_ON PB14



#define TX_INTERVAL 5000
//LMIC Callbacks requiered to compile the code even if not used
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

void onEvent (ev_t ev) { }

//Commands are build uning #define s
#define CMD_NOP ( 0 )
#define CMD_OUTPUT_SET ( 1 )
#define CMD_OUTPUT_GET ( 2 )
#define CMD_FORCE_FC_SYNC ( 255 ) 


typedef struct {
  uint32_t FrameCounter;    //4 Byte
  uint8_t Command;          //1 Byte
  uint8_t Parameter;        //1 Byte
  uint8_t UNUSED_DATA[6];   //6 Byte we don't use and fill with random stuff
} switchdata_t; // 12 Byte Data



//This will be AES256 encrypted and need to be a 16byte block 
typedef struct {
  switchdata_t SwitchData;
  uint32_t CRC32; //We try to check the Data integrity 
} payload_t; // 4 Byte CRC + 12 Byte Frame = 16 Byte Payload

//This will be the actuall packet transmitted
typedef struct{
  uint32_t CRC32; //CRC for the whole paket, as we may get garbage from the radio
  payload_t Payload; //16Byte Block
} datapaket_t; //20 Byte of Data

typedef enum {
  off=0,
  on,
  blink_on,
  blink_off
} led_mode_t;

typedef enum {
  blue=0,
  red,
  green
} led_color_t;

//We need some flags and use a bitfield for this 
typedef struct  {
  uint8_t next_state:1;     //What need to transmitt
  uint8_t current_state:1;  //Last value we got an ack for 
  uint8_t pressed_cnt:6;     //We count how often the switch has been pressed 
} switch_state_t;

switch_state_t switch_state;

typedef struct {
  uint8_t txerr:1;
  uint8_t txactive:1;
  uint8_t retry_cnt:2;
  uint8_t reserved:4;
  uint32_t framecounter_tx;
} tx_flags_t;

tx_flags_t tx_flags;

typedef struct{
  uint8_t power_state:1;
  uint8_t powerreq_state:1;
  uint8_t pulse_state_set:7;
  uint8_t pulse_state_reset:7;
  uint8_t pulsecount:8;
} relay_state_t;

relay_state_t relaystate;

switchdata_t LastProcessedData;

osjob_t txjob;
osjob_t rxjob;

osjob_t timeoutjob;

uint8_t AES_KEY[32]={0x00,};




/**************************************************************************************************
 *    Object        : AES256 Encryption and Decryption
 *    Description   : Used for Dataencryption and Decryption
 *    Input         : none
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
AES256 aes256;

/**************************************************************************************************
 *    Object        : SHA256 Library
 *    Description   : Unused to generate the key from the passphrase
 *    Input         : none
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
SHA256 sha256;


//____________________________________________________________________________
// This contains the function prototypes we need to declare in this project
// Not all functions have prototyps here, only the ones neede to make the 
// code compile 
//____________________________________________________________________________

void set_ledmode( led_color_t color, led_mode_t mode );
void tx(datapaket_t* Data, osjobcb_t func) ;
void  EncryptData( datapaket_t* EncData );
bool ReadKey(char* key, uint8_t* maxLen);
bool GenerateAESKey( void );
bool ProcessSwitchDataPaket( switchdata_t* Data);
static void tx_func (osjob_t* job);

//_____________________________________________________________________________


/**************************************************************************************************
 *    Function      : Interrupthandler
 *    Description   : Cyclic ISR called by timer
 *    Input         : none
 *    Output        : none
 *    Remarks       : Used for switch debounce and led controle ( also blink )
 **************************************************************************************************/
void Update_IT_callback(HardwareTimer*)
{ 
  //We use global flags for communcation
  static uint8_t switchinput=255;    //This is used to filter the input
  static uint8_t divider = 0;         //We use the divider to get from 100ms to 1s
  static uint8_t relay_divider = 0;
  //We check if the devider reaches the desired value
  if(divider >=9 ){
    //This will be called every second once
  
    //We reset the divider 
    divider = 0;
  } else{
    divider++;
  }
  // This will handle the debounce 
  switchinput = switchinput << 1;
  if(digitalRead(SWITCH) == 0){
    switchinput |= 0; // can be replaced by NOP
  } else {
    switchinput |= 1;
  }
  //Next is to decide if the button is pressed or not
  if( 0x80 == switchinput ){
    //Button pressed, toggle state

    if(relaystate.powerreq_state!=0){
      relaystate.powerreq_state=0;
    } else {
      relaystate.powerreq_state=1;
    }

    if(switch_state.pressed_cnt<63){
      //we increment the pressed buttons here
      switch_state.pressed_cnt++;
    }


  } else if( 0x01 == switchinput ) {
    //Button released
    //Here we don't care for a released switch for now
  } 

  if(relay_divider >=2){ //This will be called every 200ms and check power good state
    relaystate.power_state = GetSwitchActiveState();
    if ( relaystate.powerreq_state != relaystate.power_state){
      //We need to switch the relay 
      if( relaystate.pulsecount > 64 ){
        // We have a problem and need to throw an error!
      } else {
          if(  relaystate.powerreq_state > 0 ){
            //switch the relay ON
            
            relaystate.pulse_state_reset=0;
            if(relaystate.pulse_state_set==0){
              relaystate.pulse_state_set=20;
              if(relaystate.pulsecount<255){
                relaystate.pulsecount++;
              }
            }
          } else {
            //switch the relay OFF
            relaystate.pulse_state_set=0;
            if(relaystate.pulse_state_reset==0){
              relaystate.pulse_state_reset=20;
              if(relaystate.pulsecount<255){
                relaystate.pulsecount++;
              }
            }
            
          }
      }
    } else {
      relaystate.pulsecount=0;
    }

    if( relaystate.pulse_state_set>5){
      digitalWrite(RELAY_SET,HIGH);
    } else {
      digitalWrite(RELAY_SET,LOW);    
    } 

    if(relaystate.pulse_state_set>0){
      relaystate.pulse_state_set--;
    }

    if(relaystate.pulse_state_reset>2){
      digitalWrite(RELAY_RESET,HIGH);
    } else {
      digitalWrite(RELAY_RESET,LOW);
    }

    if(relaystate.pulse_state_reset>0){
      relaystate.pulse_state_reset--;
    }

    relay_divider = 0;
  } else {
    relay_divider++;
  }
  
  

}

/**************************************************************************************************
 *    Function      : GetSwitchActiveState
 *    Description   : Reads the current switch state
 *    Input         : none
 *    Output        : bool
 *    Remarks       : none
 **************************************************************************************************/
bool GetSwitchActiveState( void ){
  bool state = digitalRead(AC_POWER_ON);
  state = !state;
  return state;

}

/**************************************************************************************************
 *    Function      : SetSwitchActiveState
 *    Description   : Sets the desired switch state
 *    Input         : bool
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void SetSwitchActiveState( bool on ){
  //We set the output here accordingly
  if(false == on){
    relaystate.powerreq_state = 0;
  } else {
    relaystate.powerreq_state = 1;
  }

}



/**************************************************************************************************
 *    Function      : ProcessSwitchDataPaket
 *    Description   : Reads the decryptet packet and process the request / command
 *    Input         : switchdata_t*
 *    Output        : bool
 *    Remarks       : If this function returns true it means the answare will be delayed transmitted
 **************************************************************************************************/
bool ProcessSwitchDataPaket( switchdata_t* Data){

bool delayed_response = false;

  switch(Data->Command){
    case CMD_NOP: {//NOP
      //We do nothing but send an ACK to the sender
      delayed_response=true;
    } break;

    case CMD_OUTPUT_SET:{
        //We need to change the output to a desired value
        switch(Data->Parameter){
          case 0x00:{
            //We set the output low
            SetSwitchActiveState(false);
          } break;

          case 0x01:{
            //We set the output high
            SetSwitchActiveState(true);
          }

          default:{
            //We do nothing
          }

        }

      delayed_response = true;

    } break;

    case CMD_OUTPUT_GET:{
        //We need to report the current state back to the sender
        //Nothing to do here
        delayed_response=false;
    } break;

    case CMD_FORCE_FC_SYNC:{
       delayed_response = false;

    }

    default:{
      //We do nothing
    }

  }

  return delayed_response;

}

/**************************************************************************************************
 *    Function      : DecryptData
 *    Description   : Trys to decrypt a received datapacket
 *    Input         : uint8_t* DataArray, uint8_t ArrayLenght
 *    Output        : bool
 *    Remarks       : returns false if the packet can't be valid decoded
 **************************************************************************************************/
bool DecryptData(uint8_t* DataArray, uint8_t ArrayLenght ){
payload_t decrypted_payload;
datapaket_t* pkt_ptr;
bool fault = false;
  
if(ArrayLenght==(sizeof(datapaket_t))){
   //Next is the CRC check 
   pkt_ptr = (datapaket_t*)(DataArray);

   uint32_t checksum = CRC32::calculate((uint8_t*)(&pkt_ptr->Payload) , (size_t )(sizeof(payload_t) ) );
    if( checksum != pkt_ptr->CRC32 ){
      Serial.println("CRC32 Payload error");
      fault = true;
    } else {
      //We try to decode the data inside as the packet seems valid 
       aes256.setKey(AES_KEY, sizeof(AES_KEY));
       aes256.decryptBlock( (uint8_t*)(&decrypted_payload), (uint8_t*)(&pkt_ptr->Payload) );
       //We grab the CRC32 from the decrypted data
       checksum =  CRC32::calculate((uint8_t*)&decrypted_payload.SwitchData , (size_t )(sizeof(switchdata_t) ) );
       if(checksum != decrypted_payload.CRC32 ){
         Serial.print("Decrypt CRC32 error, expected:");
         Serial.print( checksum);
         Serial.print(" transmitted: ");
         Serial.println(decrypted_payload.CRC32);
         uint8_t * ptr = (uint8_t *)(&decrypted_payload.SwitchData);
         fault = true;
       } else {
          //If the framcounter is +1 to ours we will accept it and take the new state
          //we get a problem if the framcounter is lower than the one we do expect
          uint32_t framecounter_tx = ReadFrameCounter();
          if(decrypted_payload.SwitchData.FrameCounter == ( framecounter_tx+1 ) ){
            //This will also handle an overflow onthe 32bit counter
            fault = false;
            //We can now process the command from within the packet
            Serial.println("Valid packet received");
            WriteFrameCounter(decrypted_payload.SwitchData.FrameCounter);
            //We update our framcounter with the one we got from the RX Part
            memcpy((void*)(&LastProcessedData), (const void*)(&decrypted_payload.SwitchData), sizeof( switchdata_t ) );
          } else {
            Serial.println("Framecounter to low, send new one for sync back...") ;
            LastProcessedData.Command = CMD_FORCE_FC_SYNC;
            LastProcessedData.FrameCounter = decrypted_payload.SwitchData.FrameCounter; //This is the FC we will send back 
   
            fault = false;
          }

          
       }

    }
   


  } else {
    Serial.println("Wrong datalenght received"); 
    fault = true;
  }
 
  return fault;
}


/**************************************************************************************************
 *    Function      : EncryptData
 *    Description   : This will generate a crypted packet
 *    Input         : datapaket_t* EncData
 *    Output        : none
 *    Remarks       : Writs the encrypted data into the pointer given
 **************************************************************************************************/
void  EncryptData( datapaket_t* EncData ){
  //We generate the content here and put the payload to the TX part
  datapaket_t paket;
  payload_t payload;
  //We need to read the current framecounter and AES key to use

  payload.SwitchData.FrameCounter = ReadFrameCounter(); //Shall be in sync with the last paket received
  for(uint8_t i=0;i<6;i++){
      payload.SwitchData.UNUSED_DATA[i]=0;
  }
  //This dependy heavy on the last command processed...
  payload.SwitchData.Command = LastProcessedData.Command;
  switch(LastProcessedData.Command){
    case CMD_NOP:{
      payload.SwitchData.Parameter = 0;
    } break;

    //We send the current state, regardless of GET or SET
    case CMD_OUTPUT_GET:
    case CMD_OUTPUT_SET:{

       if(false == GetSwitchActiveState()){
        payload.SwitchData.Parameter = 0;
      } else {
        payload.SwitchData.Parameter = 1;
      } 
     
    } break;

    case CMD_FORCE_FC_SYNC:{

        payload.SwitchData.Command = CMD_FORCE_FC_SYNC;
        payload.SwitchData.Parameter=0;
        payload.SwitchData.FrameCounter = LastProcessedData.FrameCounter;
        uint32_t* data = (uint32_t*)(&payload.SwitchData.UNUSED_DATA[0]);
        *data = ReadFrameCounter(); 



    } break;

    default:{
      payload.SwitchData.Parameter = 255;
    } break;

  }
  
  
  payload.CRC32 = CRC32::calculate((uint8_t*)(&payload.SwitchData) , (size_t)(sizeof(switchdata_t) ) );  
  aes256.setKey(AES_KEY, sizeof(AES_KEY));
  aes256.encryptBlock((uint8_t*)(&paket.Payload), (uint8_t*)(&payload) );
  //Data has now been encrypted and placed to the paket
  paket.CRC32 = CRC32::calculate( (uint8_t*)(&paket.Payload) , (size_t)(sizeof(payload_t) ) );  
  //The packet is now assambled and can be transmitted
  memcpy(EncData, &paket, sizeof(datapaket_t) );

  

}




/**************************************************************************************************
 *    Function      : tx 
 *    Description   : Used for data transmission
 *    Input         : osjob_t* job, osjobcb_t func
 *    Output        : none
 *    Remarks       : This will send data encrypted to a remote switch
 **************************************************************************************************/
void tx(datapaket_t* Data, osjobcb_t func) {
  tx_flags.txactive=1;
  os_radio(RADIO_RST); // Stop RX first
  delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet
  LMIC.dataLen = 0;
  //We need to copy the packet to the LMIC buffer
  if(sizeof(datapaket_t) <= 51 ){
    //We can't transmitt more than 51 byte data, this is an LMIC limit
    memcpy( &LMIC.frame[0],(void*)(Data), sizeof(datapaket_t)  );
    LMIC.dataLen = sizeof(datapaket_t);
    LMIC.osjob.func = func;
    os_radio(RADIO_TX);
    Serial.println("Packet send");
  } else {
    // This is a general design fault
    Serial.print("Packet to large:");
    Serial.print(sizeof(datapaket_t));
    Serial.println(" to transmitt");
  }
  
}


/**************************************************************************************************
 *    Function      : rx
 *    Description   : Used to enable RX with timeout
 *    Input         : osjobcb_t func
 *    Output        : none
 *    Remarks       : This will send data encrypted to a remote switch
 **************************************************************************************************/
void rx(osjobcb_t func) {
  LMIC.osjob.func = func;
  LMIC.rxtime = os_getTime(); // RX _now_
  os_radio(RADIO_RST); // Stop RX first
  delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet
  os_radio(RADIO_RXON);
  Serial.println("Switch to receive");
  // Timeout RX (i.e. update led status) after 3 periods without RX
  os_setTimedCallback(&timeoutjob, os_getTime() + ms2osticks(30000), rxtimeout_func);
  //We after 30 seconds we restart the TX part 

}


/**************************************************************************************************
 *    Function      : rxtimeout_func
 *    Description   : Called if a RX timout occured
 *    Input         : osjob_t* job
 *    Output        : none
 *    Remarks       : This will indecate an error and will schedule a retransmission if possible
 **************************************************************************************************/
static void rxtimeout_func(osjob_t *job) {
  
  rx(rx_func); //Go into RX

}




/**************************************************************************************************
 *    Function      : rx_func
 *    Description   : Called if a new packet has received
 *    Input         : osjob_t* job
 *    Output        : none
 *    Remarks       : This will validate the response and restart RX if the pakte is broken
 **************************************************************************************************/
static void rx_func (osjob_t* job) {
  char msg[64];
  bool fault = true;
  //This is currently only for debugging
  Serial.print("Got ");
  Serial.print(LMIC.dataLen);
  Serial.println(" bytes");
  Serial.println();
  //We need to do the first round checks....
  fault = DecryptData(LMIC.frame, LMIC.dataLen );
  for(uint8_t i=0;i<LMIC.dataLen;i++){
      LMIC.frame[i]=0;
  }
  LMIC.dataLen=0;
  
  if(fault == false){
    switch_state.current_state = switch_state.next_state;     
    tx_flags.txerr=0;
    tx_flags.txactive=0;
    tx_flags.retry_cnt=0;
    Serial.println("Process received data");
    //We need to process the command
    if(false == ProcessSwitchDataPaket( &LastProcessedData )){ 
      os_setTimedCallback(&txjob, os_getTime() + ms2osticks(100), tx_func);
       Serial.println("Send response in 100ms");
    } else {
      os_setTimedCallback(&txjob, os_getTime() + ms2osticks(1000), tx_func);
      Serial.println("Send response in 1000ms");
    }
   
    //We need to switch to TX mode
  } else {
    //We need to handle the fault and keep the callback active....
    Serial.println("Invlaid Data");
    rx(rx_func);
  }


}

/**************************************************************************************************
 *    Function      : ReadFrameCounter
 *    Description   : Reads the current framecounter
 *    Input         : none
 *    Output        : uint32_t 
 *    Remarks       : none
 **************************************************************************************************/
uint32_t ReadFrameCounter( void ){
    //We will read the framecounter from the eeprom and do some integrity checks
    uint8_t data=0;
    uint8_t inverted_data=0;
    EEPROM.get(0, data);
    EEPROM.get(8, inverted_data);
    inverted_data = (~inverted_data) ;
    if(data != inverted_data ){
      Serial.print(data);
      Serial.print(" is not equal ");
      Serial.println(inverted_data);
      Serial.println("EEPROM Framecounter currupt, restore to 0");
      WriteFrameCounter(0);
      data=0;
    } else {
      // All good
    }
    
    return data;
}

/**************************************************************************************************
 *    Function      : WriteFrameCounter
 *    Description   : Writes the current framecounter
 *    Input         : uint32_t 
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void WriteFrameCounter( uint32_t Value){
  //Write Data
  uint8_t data= Value;
  uint8_t inverted_data = ~Value;
  EEPROM.put(0, data);
  EEPROM.put(8, inverted_data ) ;
  
}

/**************************************************************************************************
 *    Function      : ReadIncFrameCounter
 *    Description   : Reads and also increments the framecounter
 *    Input         : noone
 *    Output        : uint32_t
 *    Remarks       : Not interrupt save
 **************************************************************************************************/
uint32_t ReadIncFrameCounter( void ){
  uint32_t fcnt = ReadFrameCounter();
  WriteFrameCounter( fcnt+1);
  return fcnt;
}

/**************************************************************************************************
 *    Function      : WriteKey
 *    Description   : Write the key into eeprom
 *    Input         : char* key, uint8_t keyLen
 *    Output        : none
 *    Remarks       : up to 32 charakter passphrase
 **************************************************************************************************/
void WriteKey(char* key, uint8_t keyLen){
/* we write the whole key to the eeprom */
/* We use up to 66 byte in eeprom */
/* starting at address 128 */
  char buffer[33]={0,};
  

  uint8_t maxKeyLen = sizeof(buffer);
  if(keyLen<33){
    for(uint8_t i=0;i<keyLen;i++){
     buffer[i]=key[i]; 
     
    }
  }
  buffer[32]=keyLen;
  uint32_t checksum =  CRC32::calculate(( uint8_t*)(buffer) , (size_t )(sizeof(buffer) ) );
  for(uint8_t i=0; i<sizeof(buffer);i++){
    EEPROM.write( (128+i), buffer[i] );
  }
  EEPROM.put(164,checksum);

  if(false ==ReadKey(buffer, &maxKeyLen ) ){
    //Write has failed!
    Serial.println("Write to EEPROM failed for Key");
  } else {
    //We update the SHA256 value we keep in RAM
    
  }
}

/**************************************************************************************************
 *    Function      : ReadKey
 *    Description   : Write the key into eeprom
 *    Input         : char* key, uint8_t keyLen
 *    Output        : none
 *    Remarks       : up to 32 charakter passphrase
 **************************************************************************************************/
bool ReadKey(char* key, uint8_t* maxLen){
/* if the key in eeprom is invalid we generate a new one */
bool datavalid=false;
uint8_t buffer[33]={0,};
uint32_t saved_checksum = 0;
EEPROM.get(164,saved_checksum);
for(uint8_t i=0;i<sizeof(buffer);i++) {
  buffer[i]=EEPROM.read(128+i);
}

uint32_t checksum =  CRC32::calculate(( uint8_t*)(buffer) , (size_t )(sizeof(buffer) ) );

  if(saved_checksum!=checksum){
    datavalid = false;
    Serial.println("CRC error");
  } else{
    if(*maxLen<buffer[32]){
      datavalid=false;
      Serial.println("Input buffer < Passphrase");
    } else {
      for(uint8_t i=0;i<buffer[32];i++){
        key[i]=buffer[i];
      }
      datavalid=true;
      *maxLen = buffer[32];
    }
  
  }

  return datavalid;
}

/**************************************************************************************************
 *    Function      : GenerateAESKey
 *    Description   : Will do a SHA256 for the EEPROM Passphrase
 *    Input         : none
 *    Output        : bool
 *    Remarks       : will return false if no key can be generated
 **************************************************************************************************/
bool GenerateAESKey( ){
bool key_generated = false;
uint8_t buffer[32];
uint8_t keyLen=sizeof(buffer);

  if(false == ReadKey((char*)(&buffer[0]), &keyLen)){
    //This is kind of problematic and we need to warn at least.....
    Serial.println("No valid key in EEPROM fround");
    Serial.println("Set a Key or transmission will not work");
  } else {
    sha256.reset();
    sha256.update(buffer, keyLen);
    sha256.finalize(AES_KEY, sizeof(AES_KEY)); 
    key_generated=true;
    Serial.println("AES KEY generated form passphrase");
  }
 return key_generated;
}

/**************************************************************************************************
 *    Function      : WriteTXPower
 *    Description   : Will write the desired TX power to EEPROM
 *    Input         : uint8_t power
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void WriteTXPower(uint8_t power){
  uint8_t inv_power = ( ~(power) );
  EEPROM.put(16, power );
  EEPROM.put(17, inv_power );
  LMIC.txpow = GetTXPower();
}


/**************************************************************************************************
 *    Function      : GetTXPower
 *    Description   : Will read the desired TX power from EEPROM
 *    Input         : none
 *    Output        : uint8_t power
 *    Remarks       : none
 **************************************************************************************************/
uint8_t GetTXPower( void ){
  uint8_t power=0;
  uint8_t npower=0;
  EEPROM.get(16,power);
  EEPROM.get(17,npower);

  if( (uint8_t)(power) == ((uint8_t)(~npower )) ) {
    //Power okay 
    return power;
  } else {
    Serial.print("Value of ");
    Serial.print(power);
    Serial.print(" is not ");
    Serial.println(npower);
    Serial.println("EEPROM  TX Power value currupt, fall back to 2dBm");
    return 2;
  }

}

/**************************************************************************************************
 *    Function      : WriteSF
 *    Description   : Will wirte the new Datarate / SF into EEPROM
 *    Input         : u1_t
 *    Output        : none
 *    Remarks       : Only SF7 to SF10 are supported
 **************************************************************************************************/
void WriteSF(u1_t sf){
  uint8_t inv_sf = ( ~(sf) );
  EEPROM.put(20, sf );
  EEPROM.put(21, inv_sf );
  LMIC.datarate = sf;
  LMIC.rps = updr2rps(LMIC.datarate);
  
}

/**************************************************************************************************
 *    Function      : GetSF
 *    Description   : Will wirte the new Datarate / SF into EEPROM
 *    Input         : none
 *    Output        : u1_t
 *    Remarks       : Only SF7 to SF10 are supported
 **************************************************************************************************/
u1_t GetSF( void ){
  uint8_t sf=0;
  uint8_t nsf=0;
  EEPROM.get(20,sf);
  EEPROM.get(21,nsf);
  nsf = ~nsf;

  if( sf == nsf ) {
    //SF okay 
    if( (sf>=DR_SF10) && ( sf <=DR_SF7 ) ){
      return sf;
    } else {
      Serial.print("Value of ");
      Serial.print(sf);
      Serial.print(" is out of range fall back to SF7 ");
      WriteSF(DR_SF7);
    }
  
  } else {
    Serial.print("Value of ");
    Serial.print(sf);
    Serial.print(" is not ");
    Serial.println(nsf);
    Serial.println("EEPROM  SF value currupt, fall back to SF7");
    return DR_SF7;
  }

}

/**************************************************************************************************
 *    Function      : SetLMICDataRate
 *    Description   : Will wirte the new Datarate / SF in Radio
 *    Input         : u1_t
 *    Output        : none
 *    Remarks       : will take any datarate
 **************************************************************************************************/
void SetLMICDataRate( u1_t  DataRate){
  //We write the datarate to eeprom +
  WriteSF( DataRate);

  
  LMIC.datarate = DataRate;
  LMIC.rps = updr2rps(LMIC.datarate);
}


/**************************************************************************************************
 *    Function      : GetLMICDataRate
 *    Description   : Will read the Datarate / SF from Radio
 *    Input         : none
 *    Output        : u1_t
 *    Remarks       : none
 **************************************************************************************************/
u1_t  GetLMICDataRate(){
  return LMIC.datarate;
}

/**************************************************************************************************
 *    Function      : txdone_func
 *    Description   : Called after transmission has completed
 *    Input         : osjob_t* job
 *    Output        : none
 *    Remarks       : Thsi will switch into RX mode
 **************************************************************************************************/
static void txdone_func (osjob_t* job) {
  rx(rx_func); //We switch into RX mode and wait till we get an other packet
}


/**************************************************************************************************
 *    Function      : tx_func
 *    Description   : Starts a packet transmission by calling tx
 *    Input         : osjob_t* job
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
static void tx_func (osjob_t* job) {
  datapaket_t paket;
  EncryptData(&paket);
  tx( &paket, txdone_func);
}

/**************************************************************************************************
 *    Function      : lmic_setup
 *    Description   : This will initialize the LMIC Stack
 *    Input         : none
 *    Output        : none
 *    Remarks       : Here used in RAW mode
 **************************************************************************************************/
void lmic_setup( void ){
  os_init(); //LMIC runtime init
  #if defined(CFG_eu868)
    // Use a frequency in the g3 which allows 10% duty cycling.
    LMIC.freq = 869525000;
  #elif defined(CFG_us915)
    LMIC.freq = 902300000;
  #else
    error Region not supported!
  #endif

  //This sets max power as default. 
  LMIC.txpow = GetTXPower();
  
  //We can get 800 meter with DR_SF7B at max tx power
  //we use 125kHz Channel width to still keep airtime small
  //If we use 51 byte payload with LoRaWan Packets we get 102.66ms airtime
  LMIC.datarate = GetSF();
  //We can send every 10 seconds one message
  //With max transmittpower this will limit the range to 150meter

  // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
  LMIC.rps = updr2rps(LMIC.datarate);

  // disable RX IQ inversion
  LMIC.noRXIQinversion = true;
}




/**************************************************************************************************
 *    Function      : setup_timer
 *    Description   : This will initialize the timer used for led and switch debounce
 *    Input         : none
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void setup_timer( void ){

  #if defined(TIM1)
    TIM_TypeDef *Instance = TIM1;
  #else
    TIM_TypeDef *Instance = TIM2;
  #endif

  // Instantiate HardwareTimer object
  HardwareTimer *MyTim = new HardwareTimer(Instance); 

  MyTim->setMode(2, TIMER_OUTPUT_COMPARE);  // In our case, channekFalling is configured but not really used. Nevertheless it would be possible to attach a callback to channel compare match.
  MyTim->setOverflow(100, HERTZ_FORMAT); // 10 Hz
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->resume();
}
  
 

/**************************************************************************************************
 *    Function      : setup
 *    Description   : Thsi will initialize the MCU 
 *    Input         : none
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void setup() {

  /* We initilize the IO's used */
   pinMode(NOR_NCS, OUTPUT);
   pinMode(SWITCH, INPUT_PULLUP);
   pinMode(AC_POWER_ON, INPUT);
   pinMode(RELAY_RESET,OUTPUT);
   pinMode(RELAY_SET,OUTPUT);
   /* We let all leds light up */
   pinMode(LED_RED,OUTPUT);
   pinMode(LED_GREEN,OUTPUT);
   digitalWrite(LED_GREEN,HIGH);
   digitalWrite(LED_RED,HIGH);

   digitalWrite(NOR_NCS,HIGH);
   
  digitalWrite(RELAY_RESET, LOW);
  digitalWrite(RELAY_SET, LOW);

 //We set the structs to the correct state
  relaystate.pulse_state_set =0;
  relaystate.pulse_state_reset =0;
  relaystate.powerreq_state = 0;

 


   //Starting Serial and Serial1
   Serial.begin(115200);
   //Start welcome message
   Serial.println("Starting.....");
   
   /* Init for the LMIC stack */   
   lmic_setup();
  
  
  if( false == GenerateAESKey() ){
     //Okay we need to blink with one of the onboard leds!
   } else {
    //Everything gernerated
   }

   setup_timer();
   //Go into RX Mode  
   rx(rx_func); 
   ReadFrameCounter();
   Serial.println("Entering Loop() ");
   digitalWrite(LED_GREEN,LOW);
   digitalWrite(LED_RED,LOW);

}



/**************************************************************************************************
 *    Function      : loop
 *    Description   : Softwareloop handling all the logic
 *    Input         : none
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void loop() {
  //this is used to mirror some values local
  digitalWrite(LED_GREEN,HIGH);
  //This will keep the lora stack alive and should be called as often as possible
  os_runloop_once();
  //This is the serial CLI for user inputs
  SerialConsoleProcess();
  //We need to turn interrupts off as we access data that is shared with an IRQ
  digitalWrite(LED_GREEN,LOW);
  if(true == GetSwitchActiveState() ){
    digitalWrite(LED_RED,HIGH);    
  } else {
    digitalWrite(LED_RED,LOW);
  }
}
