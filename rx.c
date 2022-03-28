#include "N76E003.h"
#include "SFR_Macro.h"
#include "Function_define.h"
#include "Common.h"
#include "Delay.h"
#include "intrins.h"


static const uint8_t A7105_regs[] = {
    0xff, 0x42, 0x00, 0x14, 0x00, 0xff, 0xff ,0x00, 0x00, 0x00, 0x00, 0x01, 0x21, 0x05, 0x00, 0x50,
    0x9e, 0x4b, 0x00, 0x02, 0x16, 0x2b, 0x12, 0x00, 0x62, 0x80, 0x80, 0x00, 0x0a, 0x32, 0xc3, 0x0f,
    0x13, 0xc3, 0x00, 0xff, 0x00, 0x00, 0x3b, 0x00, 0x17, 0x47, 0x80, 0x03, 0x01, 0x45, 0x18, 0x00,
    0x01, 0x0f, 0xff,
};
static const uint8_t tx_channels[16][16] = {
  {0x0a, 0x5a, 0x14, 0x64, 0x1e, 0x6e, 0x28, 0x78, 0x32, 0x82, 0x3c, 0x8c, 0x46, 0x96, 0x50, 0xa0},
  {0xa0, 0x50, 0x96, 0x46, 0x8c, 0x3c, 0x82, 0x32, 0x78, 0x28, 0x6e, 0x1e, 0x64, 0x14, 0x5a, 0x0a},
  {0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x46, 0x96, 0x1e, 0x6e, 0x3c, 0x8c, 0x28, 0x78, 0x32, 0x82},
  {0x82, 0x32, 0x78, 0x28, 0x8c, 0x3c, 0x6e, 0x1e, 0x96, 0x46, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a},
  {0x28, 0x78, 0x0a, 0x5a, 0x50, 0xa0, 0x14, 0x64, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96},
  {0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x64, 0x14, 0xa0, 0x50, 0x5a, 0x0a, 0x78, 0x28},
  {0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x3c, 0x8c, 0x32, 0x82, 0x46, 0x96, 0x14, 0x64},
  {0x64, 0x14, 0x96, 0x46, 0x82, 0x32, 0x8c, 0x3c, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50},
  {0x50, 0xa0, 0x46, 0x96, 0x3c, 0x8c, 0x28, 0x78, 0x0a, 0x5a, 0x32, 0x82, 0x1e, 0x6e, 0x14, 0x64},
  {0x64, 0x14, 0x6e, 0x1e, 0x82, 0x32, 0x5a, 0x0a, 0x78, 0x28, 0x8c, 0x3c, 0x96, 0x46, 0xa0, 0x50},
  {0x46, 0x96, 0x3c, 0x8c, 0x50, 0xa0, 0x28, 0x78, 0x0a, 0x5a, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
  {0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x5a, 0x0a, 0x78, 0x28, 0xa0, 0x50, 0x8c, 0x3c, 0x96, 0x46},
  {0x46, 0x96, 0x0a, 0x5a, 0x3c, 0x8c, 0x14, 0x64, 0x50, 0xa0, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82},
  {0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0xa0, 0x50, 0x64, 0x14, 0x8c, 0x3c, 0x5a, 0x0a, 0x96, 0x46},
  {0x46, 0x96, 0x0a, 0x5a, 0x50, 0xa0, 0x3c, 0x8c, 0x28, 0x78, 0x1e, 0x6e, 0x32, 0x82, 0x14, 0x64},
  {0x64, 0x14, 0x82, 0x32, 0x6e, 0x1e, 0x78, 0x28, 0x8c, 0x3c, 0xa0, 0x50, 0x5a, 0x0a, 0x96, 0x46},
};

static uint8_t curServoNum = 0 ; 
static uint16_t curPWMLen = 0 ; 
static volatile uint8_t timerIntFlag = 0 ; 
static volatile unsigned long microseconds;
static uint32_t id;
static uint8_t txid[5];
static uint16_t word_temp;
static uint8_t chanrow;
static uint8_t chancol;
static uint8_t chanoffset;
static uint8_t channel;
static uint8_t aid[4];
static uint8_t packet[21];
static uint8_t jumper1 = 0;
static uint8_t jumper2 = 0;
static uint16_t total_servo_time=0;
static uint8_t cur_chan_numb=0;
static uint16_t failsafeCnt=0;
static uint16_t Servo_data[10] = {1500,1500,1000,1500,1500,1500,1500,1500};


//GIO P05, SCS P30, SCLK P17 , SDIO P15
#define  CS_on    P30=1
#define  CS_off   P30=0
#define  SCK_on   P17=1
#define  SCK_off  P17=0
#define  SDI_on   P15=1
#define  SDI_off  P15=0
#define  GIO_on   P05=1

#define  GIO_1 (P0 & 0x20) == 0x20 
#define  GIO_0 (P0 & 0x20) == 0x00
#define  SDI_1 (P1 & 0x20) == 0x20 
#define  SDI_0 (P1 & 0x20) == 0x00 



void timer2_1s_init(void)
{    
  TIMER2_DIV_512;
  TIMER2_Auto_Reload_Delay_Mode;
  RCMP2H = TIMER_DIV512_VALUE_1s >> 8  ; 
  RCMP2L = TIMER_DIV512_VALUE_1s ; 
  TL2 = 0  ;
  TH2 = 0  ;
  //start timer2 interrupt
  set_ET2;
  set_EA;
  set_TR2;                                    //Start Timer2
}
void timer2_2ms_init(void){
  timerIntFlag = 0 ; 
  TIMER2_DIV_4;   //timer2 freq = 16Mhz/4 = 4Mhz , 4000 ticks per miliseconds
  TL2 = LOBYTE(65536-8000);	
  TH2 = HIBYTE(65536-8000);  
  set_ET2;  //start timer2 interrupt
  set_EA;   //start all interrupt
  set_TR2;  //Start Timer2
}
void timer2_deinit(void){    
  clr_ET2;
  clr_TR2;
}
void Tmr2Interrupt_ISR (void) interrupt 5
{
  //handle tmr2 interrupt
  clr_TF2;
  timerIntFlag = 2 ; 
  timer2_deinit();
}

void timer3_init(uint16_t microseconds){
  //T3CON = 0x07; //Timer3 Clock = Fsys/128
  T3CON = 0x02;                           		//Timer3 Clock = Fsys/4
  //RL3 = LOBYTE(TIMER_DIV4_VALUE_10us); //65536-40
  //RH3 = HIBYTE(TIMER_DIV4_VALUE_10us); //65536-40
  RL3 = LOBYTE( 65536-4*microseconds ) ; 
  RH3 = HIBYTE( 65536-4*microseconds ) ; 
  
  //RL3 = LOBYTE( 65536-12500*1 ); //100ms
  //RH3 = LOBYTE( 65536-12500*1 ); //100ms
  
  set_ET3;  //start timer3 interrupt
  set_EA;   //start all interrupt
  set_TR3;  //Start Timer3
}
void timer3_deinit(void){
  clr_ET3;
  clr_TR3;
}
void Tmr3Interrupt_ISR (void) interrupt 16
{
  uint16_t pwm_duty = 0 ;
  clr_TF3;
  RL3 = LOBYTE( 65536-4*Servo_data[curServoNum] );
  RH3 = HIBYTE( 65536-4*Servo_data[curServoNum] );
  switch( curServoNum ){
    case 0:
      P14=0;P02=1;
      curPWMLen=Servo_data[curServoNum];
      curServoNum ++ ;
      break;
    case 1:
      P02=0;P01=1;
      curPWMLen+=Servo_data[curServoNum];
      curServoNum ++ ;
      break;
    case 2:
      P01=0;P00=1;
      curPWMLen+=Servo_data[curServoNum];
      curServoNum ++ ;
      if( Servo_data[1] > 1450 && Servo_data[1]  < 1550 ){
        //no speed
        PWM3H = 0 ; PWM3L = 0 ;
        set_SFRPAGE; PWM5H = 0 ; PWM5L = 0 ; clr_SFRPAGE;
        set_LOAD;
      }else{
        if( Servo_data[1] < 1500 ){ //backward
          PWM3H = 0 ; PWM3L = 0 ;
          set_LOAD;
        }else{//forward
          set_SFRPAGE; PWM5H = 0 ; PWM5L = 0 ; clr_SFRPAGE;
          set_LOAD;
        }
      }
      break;
    case 3:
      P00=0;P10=1;
      curPWMLen+=Servo_data[curServoNum];
      curServoNum ++ ;
      if( Servo_data[1] > 1450 && Servo_data[1]  < 1550 ){
        //no speed
        PWM3H = 0 ; PWM3L = 0 ;
        
        set_SFRPAGE; PWM5H = 0 ; PWM5L = 0 ;clr_SFRPAGE;
        set_LOAD;
      }else{
        if( Servo_data[1] < 1500 ){ //backward
          PWM3H = 0 ; PWM3L = 0 ;
          set_LOAD;
          pwm_duty = 1500 - Servo_data[1] ; 
          if( pwm_duty >= 500 )pwm_duty = 500 ; 
          if( pwm_duty <= 500 ){
            set_SFRPAGE; PWM5H = HIBYTE(pwm_duty*4-1) ; PWM5L = LOBYTE(pwm_duty*4-1) ; clr_SFRPAGE;
          }else{
            set_SFRPAGE; PWM5H = 0 ; PWM5L = 0 ; clr_SFRPAGE;
          }
          set_LOAD;
        }else{//forward
          set_SFRPAGE; PWM5H = 0 ; PWM5L = 0 ; clr_SFRPAGE;
          set_LOAD;
          pwm_duty = Servo_data[1] - 1500 ;
          if( pwm_duty >= 500 )pwm_duty = 500 ; 
          if( pwm_duty <= 500 ){
            PWM3H = HIBYTE(pwm_duty*4-1) ; PWM3L = LOBYTE(pwm_duty*4-1) ;
          }else{
            PWM3H = 0 ;PWM3L = 0 ; 
          }
          set_LOAD;
        }
      }
      break;
    case 4:
      P10=0;P11=1;
      curPWMLen+=Servo_data[curServoNum];
      curServoNum ++ ;
      break;
    case 5:
      P11=0;P12=1;
      curPWMLen+=Servo_data[curServoNum];
      curServoNum ++ ;
      break;
    case 6:
      P12=0;P13=1;
      curPWMLen+=Servo_data[curServoNum];
      curServoNum ++ ;
      break;
    case 7:
      P13=0;P14=1;
      curPWMLen+=Servo_data[curServoNum];
      curServoNum ++ ;
      break;
    case 8:
      P14=0;
      RL3 = LOBYTE( 65536-4*(20000-curPWMLen) );
      RH3 = HIBYTE( 65536-4*(20000-curPWMLen) );
      curPWMLen = 0 ;
      curServoNum = 0 ;
      break;
  }
  
  //do servo output here
  //timer3_deinit();
  //timer3_init();
}

void _spi_write(uint8_t command) {  
  uint8_t n=8; 
  SCK_off;//SCK starts low
  SDI_off;
  while(n--) {
    if(command&0x80)SDI_on;
    else SDI_off;
    SCK_on;
    _nop_();//NOP();
    SCK_off;
    command = command << 1;
  }
  SDI_on;
}  
void _spi_write_adress(uint8_t address, uint8_t data2w) {
   CS_off;
  _spi_write(address); 
  _nop_();//NOP();
  _spi_write(data2w);  
   CS_on;
} 
uint8_t _spi_read(void) {
  uint8_t result;
  uint8_t i;
  result=0;
  //pinMode(SDI_pin,INPUT);//make SDIO pin input
  //P15 input
  P15_Input_Mode;
    
  //SDI_on;
  for(i=0;i<8;i++) {                    
  if(SDI_1)  //if SDIO ==1 
      result=(result<<1)|0x01;
    else
    result=result<<1;
    SCK_on;
    _nop_();//NOP();
    SCK_off;
    _nop_();//NOP();
  }
  //pinMode(SDI_pin,OUTPUT);//make SDIO pin output again
  //P15 output 
  P15_PushPull_Mode;
  return result;
  }   
//--------------------------------------------
uint8_t _spi_read_adress(uint8_t address) { 
  uint8_t result;
  CS_off;
  address |=0x40;
  _spi_write(address);
  result = _spi_read();  
  CS_on;
  return(result); 
} 


void A7105_WriteID(uint32_t ida) {
 CS_off;
_spi_write(0x06);
_spi_write((ida>>24)&0xff); 
_spi_write((ida>>16)&0xff);
_spi_write((ida>>8)&0xff);
_spi_write((ida>>0)&0xff);
 CS_on;
}
void A7105_ReadID(){
uint8_t i;
 CS_off;
_spi_write(0x46);
for(i=0;i<4;i++){
aid[i]=_spi_read();
}
CS_on;
}
//----------------------
void Read_Packet() {
uint8_t i;
CS_off;
_spi_write(0x45);
for (i=0;i<21;i++) {
packet[i]=_spi_read();
}
CS_on;
}


//------------------------
void _spi_strobe(uint8_t address) {
 CS_off;
_spi_write(address);
 CS_on;
}
//------------------------
void A7105_reset(void) {
  _spi_write_adress(0x00,0x00); 
}



void debugPrt(UINT8 data1){
	UINT8 prtChar = '0' ;
	
  if(data1/16 < 10){
    prtChar += data1/16 ; 
  }else{
    prtChar = data1/16-10 + 'A' ;
  }
  Send_Data_To_UART0(prtChar);
  
  if(data1%16 < 10){
    prtChar = '0' + data1%16 ; 
  }else{
    prtChar = data1%16-10 + 'A' ;
  }  
  Send_Data_To_UART0(prtChar);
  Send_Data_To_UART0(',');
}

void debugPrtLn(UINT8 data1){
	debugPrt(data1);
  Send_Data_To_UART0('\r');
  Send_Data_To_UART0('\n');
}

void debugPrtStr(UINT8 * strPtr){
	UINT8 ii = 0 ; 
	for(; ii < 250 && strPtr[ii] != '\0'; ii++){
		Send_Data_To_UART0(strPtr[ii]);
	}
}


void uart0_rcv_init(UINT32 u32Baudrate)
{ 
    P06_Quasi_Mode;    //Setting UART pin as Quasi mode for transmit
    P07_Quasi_Mode;    //Setting UART pin as Quasi mode for transmit
    SCON = 0x50;       //UART0 Mode1,REN=1,TI=1
    //SCON = 0x40;      //UART0 Mode1,REN=0,TI=0  
    TMOD |= 0x20;      //Timer1 Mode1
    set_SMOD;          //UART0 Double Rate Enable
    set_T1M;
    clr_BRCK;          //Serial port 0 baud rate clock source = Timer1
#ifdef FOSC_160000
    TH1 = 256 - (1000000/u32Baudrate+1);               /*16 MHz */
#endif      
#ifdef FOSC_166000
    TH1 = 256 - (1037500/u32Baudrate);                  /*16.6 MHz */
#endif
    set_TR1;
    set_TI;            //For printf function must setting TI = 1
}
void uart0_init(UINT32 u32Baudrate)
{ 
    P06_Quasi_Mode;    //Setting UART pin as Quasi mode for transmit
    //P07_Quasi_Mode;    //Setting UART pin as Quasi mode for transmit
    //SCON = 0x50;       //UART0 Mode1,REN=1,TI=1
    SCON = 0x40;      //UART0 Mode1,REN=0,TI=0  
    TMOD |= 0x20;      //Timer1 Mode1
    set_SMOD;          //UART0 Double Rate Enable
    set_T1M;
    clr_BRCK;          //Serial port 0 baud rate clock source = Timer1
#ifdef FOSC_160000
    TH1 = 256 - (1000000/u32Baudrate+1);               /*16 MHz */
#endif      
#ifdef FOSC_166000
    TH1 = 256 - (1037500/u32Baudrate);                  /*16.6 MHz */
#endif
    set_TR1;
    set_TI;            //For printf function must setting TI = 1
}
void uart0_deinit()
{
  clr_TI ; 
  clr_RI ;
  clr_TR1;// stop timer 1 
  P06_Input_Mode; 
}


#define pwm_flash(param1,param2) PWMPH=param1;PWM1H=param2;set_LOAD
void pwm_init(void)
{
  //PWM P11 
  P11_PushPull_Mode;
  PWM_CLOCK_FSYS;
  PWM_CLOCK_DIV_128;
  PWM1_P11_OUTPUT_ENABLE;
  PWM_CENTER_TYPE;
  PWM_IMDEPENDENT_MODE;
  PWMPH = 0xFF;
  PWMPL = 0xFF;
  PWM1H = 0x8F ;
  PWM1L = 0x00 ;
  set_LOAD ; 
  set_PWMRUN;
}
void pwm_deinit(void)
{
  clr_P11; 
  P11_Input_Mode;
  clr_PWMRUN;
  clr_LOAD;
}


void gpio_interrupt_init(void)
{
  //GPIO int
  //GPIO EXTI p1.5 p1.2
  P12_Input_Mode ; 
  P15_Input_Mode ; 
  PICON &= 0x00 ; PICON |= 0x01 ;  //Enable_INT_Port1 ; 
  PICON |= 0x50 ; //GPIO EXTI pin2 pin5 , int type = edges
  PINEN &= 0x00 ; PINEN |= 0x24 ; PIPEN |= 0x24 ; //GPIO EXTI pin2 pin5 , both falling and rising edges 
  //both edges 
  set_EPI ; 
  set_EA ; 
}



void bind_Flysky() {
  static uint8_t counter1=255;
  uint8_t x;
  uint8_t i;
  uint8_t adr=10;
  Send_Data_To_UART0('b');
  Send_Data_To_UART0('0');
  _spi_strobe(0xA0);
  _spi_strobe(0xF0);
  _spi_write_adress(0x0F,0x00);//binding listen on channel 0
  _spi_strobe(0xC0);
  while(counter1){//
    Timer0_Delay1ms(10);//wait 10ms
    /*if (bitRead(counter1,2)==1){ 
      Red_LED_ON;
    }
    if(bitRead(counter1,2)==0){
      Red_LED_OFF;
    }*/
    if (GIO_0){
      x=_spi_read_adress(0x00);
      Send_Data_To_UART0('\r');Send_Data_To_UART0('\n');
      debugPrt(x);
      //if ((bitRead(x,5)==0)){//test CRC&CRF bits
      if ( (x&0x20) == 0){//test CRC&CRF bits
        Read_Packet();
        adr=10;
        for(i=0;i<5;i++){
          //EEPROM.write(adr+i,packet[i]);
          txid[i]=packet[i];
          Send_Data_To_UART0('p');
          Send_Data_To_UART0('k');
          Send_Data_To_UART0('t');
          Send_Data_To_UART0(':');
          debugPrt(packet[i]);
        }
        break;
      }
      else{
        _spi_strobe(0xA0);
        _spi_strobe(0xF0);
        _spi_write_adress(0x0F,0x00);//binding listen on channel 0
        _spi_strobe(0xC0);//try again
        continue;
      }
    }
    else{
      --counter1;
      if (counter1==0){
        counter1=255;
      }
    }
  }
  
  id=(txid[1] | ((uint32_t)txid[2]<<8) | ((uint32_t)txid[3]<<16) | ((uint32_t)txid[4]<<24));
  chanrow=id%16;
  chanoffset=(id & 0xff) / 16;
  chancol=0;
  if(chanoffset > 9) chanoffset = 9;//from sloped soarer findings, bug in flysky protocol
  
}


void loop(void){
  uint8_t x;
  //unsigned long pause;
  uint8_t i ; 
  channel=tx_channels[chanrow][chancol]-chanoffset;
  channel-=1;
  _spi_strobe(0xA0);
  _spi_strobe(0xF0);
  _spi_write_adress(0x0F,channel);
  _spi_strobe(0xC0);
  chancol = (chancol + 1) % 16;
  //pause=micros();
  timer2_2ms_init();
  while(1){
  /*#if defined(FAILSAFE)
        uint8_t n;
        if(failsafeCnt >700){   //fs delay 1575ms
        failsafeCnt=0;
      // enter your fs code here
        Servo_data[2]=1050;  //ER9x, thr min, rest center
        ppm[2] = 1050;
        for (n=0;n<2;n++){
          Servo_data[n]=1500;
          ppm[n] = 1500; }
        for (n=3;n<8;n++){
          Servo_data[n]=1500;
          ppm[n] = 1500; }  
        #if defined(DEBUG)
        Serial.println("failsafe!");
        #endif
        }
     #endif  */
    //if((micros() - pause)>2000){
    if( timerIntFlag != 0 ){
      //Red_LED_OFF;
      chancol = (chancol + 1) % 16;//advance to the next next packet most likely you missed the next one.
      channel=tx_channels[chanrow][chancol]-1-chanoffset;
      failsafeCnt++;
      break;
    }
    if (GIO_1){
      continue;
    }
    x=_spi_read_adress(0x00);
    debugPrt(x);
    //if ( !(bitRead(x,5)==0)){
    if ( (x&0x20)!=0 ){
      continue;
    }
    Read_Packet();
    if (!((packet[1]==txid[1])&& (packet[2]==txid[2])&& (packet[3]==txid[3])&& (packet[4]==txid[4]))){
      continue;
    }
    //Red_LED_ON;
    failsafeCnt=0;
    for (i=0;i<8;i++){
      //cli();
      /*if(packet[6+(2*i)] == 3){
        if(i == 0)P01 = 1;
        if(i == 1)P00 = 1;
        if(i == 2)P10 = 1;
        if(i == 3)P11 = 1;
        if(i == 4)P12 = 1;
        if(i == 5)P13 = 1;
        if(i == 6)P14 = 1;
      }else{
        if(i == 0)P01 = 0;
        if(i == 1)P00 = 0;
        if(i == 2)P10 = 0;
        if(i == 3)P11 = 0;
        if(i == 4)P12 = 0;
        if(i == 5)P13 = 0;
        if(i == 6)P14 = 0;
      }*/
      word_temp=(packet[5+(2*i)]+256*packet[6+(2*i)]);
      //sei();
      if ((word_temp>900) && (word_temp<2200))Servo_data[i]=word_temp;
      //ppm[i]=Servo_data[i];
    }
    break;
  }
}




void main(void) 
{
  
  
  uint8_t i;
  uint8_t if_calibration1;
  uint8_t vco_calibration0;
  uint8_t vco_calibration1;
  uint16_t j = 0 ;

  
  uart0_init(115200);
  Send_Data_To_UART0('s');
  Send_Data_To_UART0('t');
  Send_Data_To_UART0('a');
  Send_Data_To_UART0('r');
  Send_Data_To_UART0('t');
  Send_Data_To_UART0('\r');
  Send_Data_To_UART0('\n');
  
  set_P3SR_0;
  P30_PushPull_Mode;
  set_P1SR_7;
  P17_PushPull_Mode;
  set_P1SR_5;
  P15_PushPull_Mode;
  
  P04_Quasi_Mode;set_P0SR_4;P04=0;
  P03_Quasi_Mode;set_P0SR_3;P03=0;
  P02_PushPull_Mode;set_P0SR_2;P02=0;
  P01_PushPull_Mode;set_P0SR_1;P01=0;
  P00_PushPull_Mode;set_P0SR_0;P00=0;
  P10_PushPull_Mode;set_P1SR_0;P10=0;
  P11_PushPull_Mode;set_P1SR_1;P11=0;
  P12_PushPull_Mode;set_P1SR_2;P12=0;
  P13_PushPull_Mode;set_P1SR_3;P13=0;
  P14_PushPull_Mode;set_P1SR_4;P14=0;
  
  
  PWM_IMDEPENDENT_MODE;
  PWM5_P03_OUTPUT_ENABLE; 
  PWM3_P04_OUTPUT_ENABLE; 
  
  PWM_CLOCK_DIV_8;
  PWMPH = 0x07; //0 ~ 1999
  PWMPL = 0xCF;
  
  PWM3H = 0x00 ; 
  PWM3L = 0x00 ; 
  set_SFRPAGE;						//PWM4 and PWM5 duty seting is in SFP page 1
  PWM5H = 0x00;						
  PWM5L = 0x00;
  clr_SFRPAGE;
  set_LOAD;
  set_PWMRUN;
  
  
  timer3_init(1);
  //how to set pwm
  //PWM3H=0x04 ; 
  //set_LOAD;
  
  
  CS_on;//start CS high
  SDI_on;//start SDIO high
  SCK_off;//start sck low
  
  Timer0_Delay1ms(10);//wait 10ms for A7105 wakeup
  _spi_write_adress(0x00,0x00);//reset A7105
  A7105_WriteID(0x5475c52A);//A7105 id
  A7105_ReadID();
  
  debugPrt(aid[0]);
  debugPrt(aid[1]);
  debugPrt(aid[2]);
  debugPrt(aid[3]);
  debugPrt(aid[4]);
  
  for (i = 0; i < 0x33; i++){
    if(A7105_regs[i] != 0xff) _spi_write_adress(i, A7105_regs[i]);
  }
  
  //BEGIN A7105 init
  _spi_strobe(0xA0);//stand-by
  _spi_write_adress(0x02,0x01);
  while(1){
    if_calibration1=_spi_read_adress(0x02);
    if(if_calibration1==0) break;
  }
  _spi_read_adress(0x22);
  _spi_write_adress(0x24,0x13);
  _spi_write_adress(0x25,0x09);
  _spi_strobe(0xA0);//stand-by
  //END A7105 init

  
  
  
  
  bind_Flysky();
  
  while(1){
    loop();
  }
}

