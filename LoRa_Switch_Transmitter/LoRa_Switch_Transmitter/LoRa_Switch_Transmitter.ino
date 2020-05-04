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


#define Logo_width 64
#define Logo_height 64
static unsigned char Logo_bits[] = {
   0x00, 0x00, 0x00, 0xd0, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff,
   0xff, 0x01, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xff, 0xff, 0x0f, 0x00, 0x00,
   0x00, 0x00, 0xfc, 0xff, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff,
   0xff, 0xff, 0x00, 0x00, 0x00, 0x80, 0xff, 0xff, 0xff, 0xff, 0x01, 0x00,
   0x00, 0xe0, 0xff, 0xff, 0xff, 0xff, 0x07, 0x00, 0x00, 0xf0, 0xff, 0xff,
   0xff, 0xff, 0x0f, 0x00, 0x00, 0xf8, 0xff, 0x03, 0xc0, 0xff, 0x1f, 0x00,
   0x00, 0xfc, 0x7f, 0x00, 0x00, 0xfe, 0x3f, 0x00, 0x00, 0xfe, 0x1f, 0x00,
   0x00, 0xf8, 0x7f, 0x00, 0x00, 0xff, 0x07, 0x00, 0x00, 0xe0, 0xff, 0x00,
   0x80, 0xff, 0x01, 0x00, 0x00, 0xc0, 0xff, 0x01, 0xc0, 0xff, 0x00, 0x00,
   0x00, 0x00, 0xff, 0x03, 0xc0, 0x7f, 0x00, 0x00, 0x00, 0x00, 0xfe, 0x03,
   0xe0, 0x3f, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x07, 0xf0, 0x1f, 0x00, 0x00,
   0x00, 0x00, 0xf8, 0x0f, 0xf0, 0x1f, 0x00, 0xe0, 0x07, 0x00, 0xf8, 0x0f,
   0xf8, 0x0f, 0x00, 0xfc, 0x0f, 0x00, 0xf0, 0x1f, 0xf8, 0x07, 0x00, 0xfe,
   0x1f, 0x00, 0xe0, 0x1f, 0xfc, 0x07, 0x80, 0xff, 0x3f, 0x00, 0xe0, 0x3f,
   0xfc, 0x03, 0x80, 0xff, 0x7f, 0x00, 0xc0, 0x3f, 0xfc, 0x03, 0xc0, 0xff,
   0x7f, 0x00, 0xc0, 0x3f, 0xfe, 0x01, 0xc0, 0xff, 0xff, 0x00, 0x80, 0x7f,
   0xfe, 0x01, 0xe0, 0xff, 0x3f, 0x00, 0x80, 0x7f, 0xfe, 0x01, 0xe0, 0xff,
   0x07, 0x00, 0x80, 0x7f, 0xfe, 0x00, 0xe0, 0x7f, 0x00, 0x00, 0x00, 0x7f,
   0xff, 0x00, 0xe0, 0x07, 0x00, 0x00, 0x00, 0x7f, 0xfe, 0x00, 0x60, 0x00,
   0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xff,
   0xff, 0x00, 0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,
   0x00, 0xe0, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x00, 0xfe, 0xff, 0xff,
   0xff, 0x00, 0x00, 0x00, 0xe0, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00,
   0xfc, 0xff, 0x7f, 0x7f, 0xff, 0x00, 0x00, 0xc0, 0xff, 0xff, 0x1f, 0xff,
   0xfe, 0x00, 0x00, 0xfc, 0xff, 0xff, 0x0f, 0x7f, 0xfe, 0x00, 0x00, 0xff,
   0xff, 0xff, 0x07, 0x7f, 0xfe, 0x01, 0x00, 0xff, 0xff, 0xff, 0x83, 0x7f,
   0xfe, 0x01, 0x00, 0xfe, 0xff, 0xff, 0x81, 0x7f, 0xfe, 0x01, 0x00, 0xfe,
   0xff, 0xff, 0x80, 0x7f, 0xfc, 0x01, 0x00, 0xfc, 0xff, 0x3f, 0xc0, 0x3f,
   0xfc, 0x03, 0x00, 0xf8, 0xff, 0x0f, 0xc0, 0x3f, 0xfc, 0x03, 0x00, 0xe0,
   0xff, 0x03, 0xc0, 0x3f, 0xf8, 0x07, 0x00, 0x80, 0x7f, 0x00, 0xe0, 0x1f,
   0xf8, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xf0, 0x1f, 0xf0, 0x0f, 0x00, 0x00,
   0x00, 0x00, 0xf0, 0x0f, 0xf0, 0x1f, 0x00, 0x00, 0x00, 0x00, 0xf8, 0x0f,
   0xe0, 0x3f, 0x00, 0x00, 0x00, 0x00, 0xfc, 0x07, 0xe0, 0x7f, 0x00, 0x00,
   0x00, 0x00, 0xfe, 0x07, 0xc0, 0xff, 0x00, 0x00, 0x00, 0x00, 0xff, 0x03,
   0x80, 0xff, 0x01, 0x00, 0x00, 0x80, 0xff, 0x01, 0x00, 0xff, 0x07, 0x00,
   0x00, 0xe0, 0xff, 0x00, 0x00, 0xff, 0x0f, 0x00, 0x00, 0xf0, 0xff, 0x00,
   0x00, 0xfc, 0x3f, 0x00, 0x00, 0xfe, 0x7f, 0x00, 0x00, 0xfc, 0xff, 0x01,
   0x80, 0xff, 0x1f, 0x00, 0x00, 0xf0, 0xff, 0x7f, 0xff, 0xff, 0x0f, 0x00,
   0x00, 0xe0, 0xff, 0xff, 0xff, 0xff, 0x07, 0x00, 0x00, 0xc0, 0xff, 0xff,
   0xff, 0xff, 0x03, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00,
   0x00, 0x00, 0xfc, 0xff, 0xff, 0x3f, 0x00, 0x00, 0x00, 0x00, 0xf0, 0xff,
   0xff, 0x0f, 0x00, 0x00, 0x00, 0x00, 0xc0, 0xff, 0xff, 0x01, 0x00, 0x00,
   0x00, 0x00, 0x00, 0xf0, 0x1f, 0x00, 0x00, 0x00 };

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
#define VBAT_IN PB15

#define GPS_ACRIVE_ON PC13

//This is specific for the LoRaSwitch TX part
#define SWITCH PB12
#define LED_GREEN PB13
#define LED_RED PB14
#define LED_BLUE PB15


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

typedef struct {
 //For blue led
  uint8_t blueblink:1;
  uint8_t bluevalue:1;
//For red led
  uint8_t redblink:1;
  uint8_t redvalue:1;
//For green led  
  uint8_t greenblink:1;
  uint8_t greenvalue:1;
//Unused so far 
   uint8_t reserved:2;
} led_state_t;

led_state_t led_state;

osjob_t txjob;
osjob_t timeoutjob;

uint8_t AES_KEY[32]={0x00,};


//This will be 5 Minutes
#define SLEEPTIMEOUT ( 1*60*100 )

volatile bool HasDisplay = false;
volatile uint32_t SleepTimeout = SLEEPTIMEOUT ; //We set this to 5 Minutes 

static void tx_func (osjob_t* job);

/**************************************************************************************************
 *    Object        : I2C OLED display 
 *    Description   : Optinal display to get statusinformation
 *    Input         : none
 *    Output        : none
 *    Remarks       : Uses HW I2C and a fully ( 1024 byte ) Framebuffer 
 **************************************************************************************************/
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, /* reset=*/ U8X8_PIN_NONE);

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

//Function prototypes
void set_ledmode( led_color_t color, led_mode_t mode );
void tx(datapaket_t* Data, osjobcb_t func) ;
void EncryptData( datapaket_t* EncData );
void EncryptRequest( datapaket_t* EncData );
bool ReadKey(char* key, uint8_t* maxLen);
bool GenerateAESKey( void );
void display_TX_Retry(uint8_t rt_cnt);
void display_status_request( void );

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
  
  if(SleepTimeout>0){
    SleepTimeout--;
  }

  //We check if the devider reaches the desired value
  if(divider >=9 ){
    //Check if the led need to blink, first the blue one
    if( 0 != led_state.blueblink )
    {
      digitalWrite( LED_BLUE , !digitalRead( LED_BLUE ));
    } else {
      if( 0 != led_state.bluevalue ){
        digitalWrite( LED_BLUE , HIGH );
      } else {
        digitalWrite( LED_BLUE , LOW );
      }
    }
    //Next is the blue one
    if( 0 != led_state.greenblink )
    {
      digitalWrite( LED_GREEN , !digitalRead( LED_GREEN ));
    } else {
      if( 0 != led_state.greenvalue ){
        digitalWrite( LED_GREEN , HIGH );
      } else {
        digitalWrite( LED_GREEN , LOW );
      }
    }
    //Last one is the red one
    if( 0 != led_state.redblink )
    {
      digitalWrite( LED_RED , !digitalRead( LED_RED ));
    } else {
      if( 0 != led_state.redvalue ){
        digitalWrite( LED_RED , HIGH );
      } else {
        digitalWrite( LED_RED , LOW );
      }
    }
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
    if(switch_state.pressed_cnt<63){
      //we increment the pressed buttons here
      switch_state.pressed_cnt++;
    }
  } else if( 0x01 == switchinput ) {
    //Button released
    //Here we don't care for a released switch for now
  } 
}


//----------------------------------------------------------------------------------------
// We have here changes for the CPU clock to reduce power consumption
//----------------------------------------------------------------------------------------
void SystemClock_125kHz(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14
                              |RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV64;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART2
                              |RCC_PERIPHCLK_RTC;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_HSI;
  PeriphClkInit.Usart2ClockSelection = RCC_USART2CLKSOURCE_HSI;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**************************************************************************************************
 *    Function      : SystemClock_8MHz
 *    Description   : Changes the Clock to 8MHz
 *    Input         : none
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void SystemClock_8MHz(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    /** Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    {
        while(1==1){};
    }
    /** Initializes the CPU, AHB and APB busses clocks 
    */
    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                          |RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
    {
         while(1==1){};
    }
}


/**************************************************************************************************
 *    Function      : set_ledmode
 *    Description   : Sets how the led will behave
 *    Input         : led_color_t color, led_mode_t mode
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void set_ledmode( led_color_t color, led_mode_t mode ){
  
  switch(color){

    case blue:{

        switch( mode ){

            case on:{
                led_state.bluevalue=1;
            } break;

            case off:{
                led_state.bluevalue=0;
            } break;

            case blink_on:{
                led_state.blueblink=1;
            } break;

            case blink_off:{
                led_state.blueblink=0;
            } break;

            default:{

            } break;

        } 

    } break;

    case green:{

            switch( mode ){

            case on:{
                led_state.greenvalue=1;
            } break;

            case off:{
                led_state.greenvalue=0;
            } break;

            case blink_on:{
                led_state.greenblink=1;
            } break;

            case blink_off:{
                led_state.greenblink=0;
            } break;

            default:{

            } break;

        } 

    } break;

    case red:{
            switch( mode ){

            case on:{
                led_state.redvalue=1;
            } break;

            case off:{
                led_state.redvalue=0;
            } break;

            case blink_on:{
                led_state.redblink=1;
            } break;

            case blink_off:{
                led_state.redblink=0;
            } break;

            default:{

            } break;

        } 

    } break;

    default:{

    } break;


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
  bool state = false;
  if(0 == switch_state.current_state ){
      state = false;
  } else {
      state = true;
  }

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

uint8_t wished_value=0;

if(true == on ){
  wished_value=1;
} else {
  wished_value=0;
}


    noInterrupts(); //This will akt as a button press with a wished value
    if(wished_value != switch_state.current_state){
         switch_state.next_state=wished_value;
      
      if(switch_state.pressed_cnt<63){
          //we increment the pressed buttons here
          switch_state.pressed_cnt++;
      }
    }
    interrupts();

}

/**************************************************************************************************
 *    Function      : print_array
 *    Description   : will print an array of data as hex
 *    Input         : uint8_t * data , uint32_t len 
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void print_array(uint8_t * data , uint32_t len ){

  for(uint32_t i=0;i<len;i++){
           Serial.print("0x");
           Serial.print(*data++);
           Serial.print(" ");
  }
  Serial.println("");
}

/**************************************************************************************************
 *    Function      : DecryptData
 *    Description   : Trys to decrypt a received datapacket
 *    Input         : uint8_t* DataArray, uint8_t ArrayLenght
 *    Output        : bool
 *    Remarks       : returns false if the packet can't be valid decoded
 **************************************************************************************************/
bool DecryptData(uint8_t* DataArray, uint8_t ArrayLenght, bool* fast_recon ){
payload_t decrypted_payload;
datapaket_t* pkt_ptr;
bool fault = false;
*fast_recon = false;
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
        //We output the encypted data
        Serial.println("Decrypted Data");
        print_array( (uint8_t*)&decrypted_payload, sizeof(payload_t) );

         fault = true;
       } else {
          //Data is valid and we check if the framecounter we send has been returned
          uint32_t framecounter_tx = ReadFrameCounter()-1;
          if(decrypted_payload.SwitchData.FrameCounter == framecounter_tx){
            //We got an answare to what we transmitted
            if(decrypted_payload.SwitchData.Command == CMD_FORCE_FC_SYNC ){
              fault = true;
              *fast_recon = true;
              uint32_t remote_fc;
              uint32_t* data = (uint32_t*)(&(decrypted_payload.SwitchData.UNUSED_DATA[0]));
              remote_fc = *data;
              remote_fc++;
              Serial.print("Sync Framecounter to:");
              Serial.println( remote_fc );
              WriteFrameCounter( remote_fc )  ;      
              //We need to speedup the retransmitt here 
            } else {
              if( ( decrypted_payload.SwitchData.Command == CMD_OUTPUT_GET  ) || (decrypted_payload.SwitchData.Command == CMD_OUTPUT_SET) ){
                  //We accept as answare any way even if the command is not supported?
                  //We update the remote state
                  if(decrypted_payload.SwitchData.Command == CMD_OUTPUT_GET ){
                      switch_state.current_state = decrypted_payload.SwitchData.Parameter;
                      switch_state.next_state = decrypted_payload.SwitchData.Parameter;

                  } else {
                      switch_state.current_state = decrypted_payload.SwitchData.Parameter;
                  }
                  

                  fault = false;
              } else {
                 //We accept as answare any way even if the command is not supported?
                 Serial.println("Answare with strange command");
                  fault = false;
              }
            
            }
            
            
            //We can also have commands from the other side to us.....
            //Currenrly we use this only as ACK
          } else {
            Serial.println("Framecounter mismatch") ;           
            //Wrong framecounter value
            //We get in the unused data the framcounter we need to use for the next tansmitt...
            fault = true;
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

  payload.SwitchData.FrameCounter = ReadIncFrameCounter();
  payload.SwitchData.Command = CMD_OUTPUT_SET;
  payload.SwitchData.Parameter = switch_state.next_state;
  for(uint8_t i=0;i<6;i++){
      payload.SwitchData.UNUSED_DATA[i]=0;
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
 *    Function      : EncryptRequest
 *    Description   : This will generate a crypted packet
 *    Input         : datapaket_t* EncData
 *    Output        : none
 *    Remarks       : This is only for data request
 **************************************************************************************************/
void  EncryptRequest( datapaket_t* EncData ){
  //We generate the content here and put the payload to the TX part
  datapaket_t paket;
  payload_t payload;
  //We need to read the current framecounter and AES key to use

  payload.SwitchData.FrameCounter = ReadIncFrameCounter();
  payload.SwitchData.Command = CMD_OUTPUT_GET; 
  payload.SwitchData.Parameter = switch_state.next_state;
  for(uint8_t i=0;i<6;i++){
      payload.SwitchData.UNUSED_DATA[i]=0;
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
 *    Function      : tx_func
 *    Description   : Used for data transmission
 *    Input         : osjob_t* job
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
    set_ledmode(blue,blink_on);
    set_ledmode(green,off);

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
  // Enable "continuous" RX (e.g. without a timeout, still stops after
  // receiving a packet)
  os_radio(RADIO_RXON);
  Serial.println("Switch to receive");
  // Timeout RX (i.e. update led status) after 3 periods without RX
  os_setTimedCallback(&timeoutjob, os_getTime() + ms2osticks(3*TX_INTERVAL), rxtimeout_func);


}


/**************************************************************************************************
 *    Function      : rxtimeout_func
 *    Description   : Called if a RX timout occured
 *    Input         : osjob_t* job
 *    Output        : none
 *    Remarks       : This will indecate an error and will schedule a retransmission if possible
 **************************************************************************************************/
static void rxtimeout_func(osjob_t *job) {
  os_radio(RADIO_RST); // Stop RX first
  delay(1); // Wait a bit, without this os_radio below asserts, apparently because the state hasn't changed yet
  if(tx_flags.retry_cnt<2){
    tx_flags.retry_cnt++;
    //We reschedule a TX 
     os_setCallback(&txjob, tx_func); 
     display_TX_Retry(tx_flags.retry_cnt);
     Serial.print("RX Timeout, retry");
     set_ledmode(blue,blink_on);
     set_ledmode(red,blink_on);

  } else {
    //Limit reached, we stop the transmission.....
    tx_flags.txerr=1;
    tx_flags.txactive=0;
    tx_flags.retry_cnt=0;
    Serial.print("RX Timeout, no response");
    
    set_ledmode(blue,off);
    set_ledmode(blue,blink_off);
    set_ledmode(red,blink_off);
    set_ledmode(red,on);
    set_ledmode(green,off);
  }


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
  datapaket_t* pkt_ptr=NULL;
  //This is currently only for debugging
  Serial.print("Got ");
  Serial.print(LMIC.dataLen);
  Serial.println(" bytes");
  bool fast_recon = false;
  //We need to do the first round checks....
  fault = DecryptData(LMIC.frame, LMIC.dataLen, &fast_recon);
  
  if(fault == false){
    //Check Receiver Code for state handling
    if( switch_state.current_state != switch_state.next_state ){
      Serial.println("Settigs don't match");
    }
    tx_flags.txerr=0;
    tx_flags.txactive=0;
    tx_flags.retry_cnt=0;
    os_clearCallback( &timeoutjob );
    Serial.println("ACK received");
    set_ledmode(green,on);
    set_ledmode(green,blink_off);
    set_ledmode(blue, off);
    set_ledmode(blue, blink_off);
    set_ledmode(red,off);
    set_ledmode(red,blink_off);
    display_main_screen();
    if(0==switch_state.current_state)
    {
      set_ledmode(blue, off);
    } else {
      set_ledmode(blue, on);
    }

  } else {
    //We need to handle the fault and keep the callback active....
    //Also we need to witch to RX mode again.... 
    if(false == fast_recon){
     os_radio(RADIO_RXON);
     set_ledmode(green,off);
    } else {
      os_clearCallback( &timeoutjob );
      //We generate a timout in 200ms to speed up the retansmission if requiered
      os_setTimedCallback(&timeoutjob, os_getTime() + ms2osticks(200), rxtimeout_func); 
    }
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
    uint32_t data=0;
    uint32_t inverted_data=0;
    EEPROM.get(0, data);
    EEPROM.get(8, inverted_data);
    if(data != (~inverted_data) ){
      Serial.println("EEPROM Framecounter currupt, restore to zero");
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
  EEPROM.put(0, Value);
  EEPROM.put(8, (~Value) ) ;
  
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
    Serial.print(" is not inversion of ");
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
  //We write the datarate to eeprom 
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
  rx(rx_func); //We switch into RX mode and wait till we gont an answare
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
 *    Function      : tx_func
 *    Description   : Starts a packet transmission by calling tx
 *    Input         : osjob_t* job
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
static void tx_req_func (osjob_t* job) {
  display_status_request();
  datapaket_t paket;
  EncryptRequest(&paket);
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
  LMIC.txpow = GetTXPower(); // will be limted to 15dBm for 868MHz 
  
  //We can get 800 meter with DR_SF7B at max tx power
  //we use 125kHz Channel width to still keep airtime small
  //If we use 51 byte payload with LoRaWan Packets we get 102.66ms airtime
  LMIC.datarate =   LMIC.datarate = GetSF();
  //We can send every 10 seconds one message

  // This sets CR 4/5, BW125 (except for DR_SF7B, which uses BW250)
  LMIC.rps = updr2rps(LMIC.datarate);

  // disable RX IQ inversion
  LMIC.noRXIQinversion = true;
}


/**************************************************************************************************
 *    Function      : setup_display
 *    Description   : This will initialize the display if attached
 *    Input         : none
 *    Output        : none
 *    Remarks       : Will also try to initialize if no display is found
 **************************************************************************************************/
bool setup_display(){
  digitalWrite(GPS_ACRIVE_ON, HIGH); // We need to check if the display comes up fast enough
  delay(100);//100ms delay to get the display active...
  bool found = false;
  Wire.begin();
  uint8_t error = 0;  
  Wire.beginTransmission(0x3C); //0x78
  error = Wire.endTransmission();
   if(error!=0){    
      Wire.beginTransmission(0x3D); //0x7A
      error = Wire.endTransmission();
      if(error != 0 ) {
         Serial.println("No OLED found");
         found = false;
      } else {
          u8g2.setI2CAddress(0x7A);
          Serial.println("OLED found at 0x7A"); 
          found = true;
      }
   } else {
        u8g2.setI2CAddress(0x78); 
        Serial.println("OLED found at 0x78"); 
        found = true;
   }

  if( found != false ){ 
    //This will setup the OLED even if not connected 
    u8g2.begin();
    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setFontRefHeightExtendedText();
    u8g2.setDrawColor(1);
    u8g2.setFontPosTop();
    u8g2.setFontDirection(0);
    u8g2.clearBuffer();
    u8g2.drawXBM(0,0,64,64,Logo_bits);
    u8g2.sendBuffer();
  } else {
    //We set globally the display to disabled 
  }

  return found;
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
  MyTim->setOverflow(100, HERTZ_FORMAT); // 100 Hz
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->resume();
}


/**************************************************************************************************
 *    Function      : display_main_screen
 *    Description   : This is the display drawing section everything we like to print goes here 
 *    Input         : none
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/ 
void display_main_screen(){
  if(false == HasDisplay){
    return;
 }
  //We read the last known input 
  u8g2.clearBuffer();
  //We grab the current known? switchstate and display an power on of off ionc 
  u8g2.setFont(u8g2_font_open_iconic_all_4x_t); //Power icon
  u8g2.drawGlyph(0,0,235);
  
  u8g2.setFont(u8g2_font_profont17_mr );
  if(0==switch_state.current_state){
    
    u8g2.drawStr(0,36,"OFF");

  } else {
    u8g2.drawStr(6,36,"ON");
  }
  


  u8g2.sendBuffer();
}

/**************************************************************************************************
 *    Function      : display_status_request
 *    Description   : This will display the status request 
 *    Input         : none
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/ 
void display_status_request(){
  if(false == HasDisplay){
    return;
  }
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_open_iconic_all_4x_t);
  u8g2.drawGlyph(0,20,96); 
  u8g2.setFont(u8g2_font_profont17_mr );
  u8g2.drawStr(0,0,"Request state");
  u8g2.drawStr(40,20,"from");
  u8g2.drawStr(40,40,"Remote");
  u8g2.sendBuffer();
}

/**************************************************************************************************
 *    Function      : display_TX_Retry
 *    Description   : This will display TX retry
 *    Input         : none
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/ 
void display_TX_Retry(uint8_t rt_cnt){
  if(false == HasDisplay){
    return;
  }
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_open_iconic_all_4x_t);
  u8g2.drawGlyph(0,20,243); 
  u8g2.setFont(u8g2_font_profont17_mr );
  u8g2.drawStr(0,0,"No Response");
  u8g2.drawStr(40,20,"Retry");
  u8g2.setCursor(52, 36);
  u8g2.print(rt_cnt);
  u8g2.sendBuffer();
}

/**************************************************************************************************
 *    Function      : display_goingtosleep
 *    Description   : This will display going to sleep
 *    Input         : none
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/ 
void display_goingtosleep( void ){
  if(false == HasDisplay){
    return;
 }
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_open_iconic_all_4x_t);
  u8g2.drawGlyph(0,0,223); 
  u8g2.setFont(u8g2_font_profont17_mr );

  u8g2.sendBuffer();
}

/**************************************************************************************************
 *    Function      : display_sleepmode
 *    Description   : This will display sleep
 *    Input         : none
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/ 
void display_sleepmode( void ){
    if(false == HasDisplay){
    return;
 }
  u8g2.clearBuffer();
  //Turn all pixels off 
  u8g2.sendBuffer();
}

/**************************************************************************************************
 *    Function      : display_last_known_state
 *    Description   : This will display the last known state
 *    Input         : none
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void display_last_known_state( bool on){
    if(false == HasDisplay){
    return;
 }


}

/**************************************************************************************************
 *    Function      : display_power_on
 *    Description   : This will display power on
 *    Input         : none
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void display_power_on( void ){
    if(false == HasDisplay){
    return;
 }

}

/**************************************************************************************************
 *    Function      : display_power_off
 *    Description   : This will display power off
 *    Input         : none
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void display_power_off( void ){
    if(false == HasDisplay){
    return;
 }

}
/**************************************************************************************************
 *    Function      : setup
 *    Description   : Thsi will initialize the MCU 
 *    Input         : none
 *    Output        : none
 *    Remarks       : none
 **************************************************************************************************/
void setup( ) {

  /* We initilize the IO's used */
   pinMode(SWITCH, INPUT_PULLUP);
   pinMode(LED_GREEN, OUTPUT);
   pinMode(LED_RED,OUTPUT);
   pinMode(LED_BLUE,OUTPUT);
   pinMode(GPS_ACRIVE_ON,OUTPUT);
   //Turn off the OLED
   digitalWrite(GPS_ACRIVE_ON, LOW);
   /* We let all leds light up */
   digitalWrite(LED_BLUE, HIGH);
   digitalWrite(LED_RED, HIGH);
   digitalWrite(LED_GREEN, HIGH);

  //Starting Serial and Serial1
   Serial.begin(115200);
   //Start welcome message
   Serial.println("Starting.....");
   
  /* Next is to initilize the display */
   if(false == setup_display() ){
      Serial.println("Display disabled");
      HasDisplay = false;
   } else {
      Serial.println("Display init done");
      HasDisplay=true;
   }
   Serial.println("Setup LMIC");
   /* Init for the LMIC stack */   
   lmic_setup();
   Serial.println("LMIC init done");
  
   /* We write the first lines to the display */
   Serial.println("Generate AES KEY");
   if( false == GenerateAESKey() ){
     //We need to display an error and stop
    /* disableing all leds */
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_RED, HIGH);
    digitalWrite(LED_GREEN, LOW);


   } else {

     /* disableing all leds */
   digitalWrite(LED_BLUE, LOW);
   digitalWrite(LED_RED, LOW);
   digitalWrite(LED_GREEN, LOW);


   }


  
   setup_timer();
   display_main_screen();
   //we also start a request for the current switch state 
   os_setCallback(&txjob, tx_req_func);     

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
  uint8_t btn_press_cnt=0;
  uint8_t btn_state = 0;
  uint32_t loc_sleeptimout=0;
  bool fault = true;
  static bool led_n_led_powerdown = false;
  static bool prepare_sleep=false;

  if(false == led_n_led_powerdown){
    //This will keep the lora stack alive and should be called as often as possible
    os_runloop_once();
    //This is the serial CLI for user inputs
  }
  //We need to turn interrupts off as we access data that is shared with an IRQ
 
  if(true==SerialConsoleProcess()){
    //we we sleep time to wake up
      noInterrupts(); 
        SleepTimeout = SLEEPTIMEOUT;
      interrupts(); 
  }

  
  
  noInterrupts(); 
  //We grab the data and use it local to reduce the time spend with disabled IRQ
  btn_press_cnt = switch_state.pressed_cnt;
  switch_state.pressed_cnt = 0;
  loc_sleeptimout = SleepTimeout;
  interrupts();

  if( (btn_press_cnt>0) && ( tx_flags.txactive== 0) && (false == led_n_led_powerdown) ){ //Reset will be done in the next loop back to zero   
      noInterrupts(); 
        SleepTimeout = SLEEPTIMEOUT;
        if(switch_state.current_state!=0){
          switch_state.next_state=0;
        } else {
          switch_state.next_state=1;
        }
        btn_state = switch_state.next_state;
      interrupts();
    //We need to send data
    if(btn_state==0){
        os_setCallback(&txjob, tx_func);     
        Serial.println("SWITCH OFF");
        //We set the flag to let the led blink
        
    } else {
        os_setCallback(&txjob, tx_func);
        Serial.println("SWITCH ON");
        //We set the flag to let the led blink 
          
    }
  } else {
    //One of out few conditions has not matched so we check for a power down
    if( (true == led_n_led_powerdown) && ( btn_press_cnt > 0 ) ){
      //We need to wake up first
      //This means powering the led and a compleat new ini for it....
      set_ledmode(green,blink_on);
      SystemClock_8MHz();
      Serial.println("Wake up");
       /* Next is to initilize the display */
      if(false == setup_display() ){
          Serial.println("Display disabled");
          HasDisplay = false;
      } else {
          Serial.println("Display init done");
          HasDisplay=true;
      }
      noInterrupts(); 
           SleepTimeout = SLEEPTIMEOUT ; // Reset timeout
      interrupts();
      led_n_led_powerdown = false;
      display_main_screen();
      //Request currents state from the other side
      os_setCallback(&txjob, tx_req_func);    

    } else if ( false == led_n_led_powerdown ){

        if( ( loc_sleeptimout <= 1000 ) && ( loc_sleeptimout >= 1 ) && ( false == prepare_sleep ) ){
          //10 seconds till we cut power to the led and oled
          prepare_sleep=true;
          display_goingtosleep();
          //We also let all other systems enter sleep is possible to reduce power consumption
        } else if( loc_sleeptimout == 0 ){
          Serial.println("Enter sleep");
          prepare_sleep=false;
          //Enter sleep mode
          led_n_led_powerdown = true;
          //Write sleep symbol to oled 
          display_sleepmode();
          digitalWrite(GPS_ACRIVE_ON, LOW); //Power down the oled 
          
          
          set_ledmode(blue,blink_off);
          set_ledmode(blue,off);

          set_ledmode(red,blink_off);
          set_ledmode(red,off);

          set_ledmode(green,blink_off);
          set_ledmode(green,off);
          
          os_radio(RADIO_RST); // Put radio into sleep
         
          SystemClock_125kHz();
          SystemCoreClockUpdate();
          LowPower_sleep(PWR_LOWPOWERREGULATOR_ON);
          
        }
    } else if ( true == led_n_led_powerdown) {
        //Back to sleep....
        LowPower_sleep(PWR_LOWPOWERREGULATOR_ON);
    }

  } 
 
  
}
