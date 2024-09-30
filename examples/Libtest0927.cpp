#include <Arduino.h>
//#include "HMI_V1_0.h"
//#include "t0710.h"
//#include "t07101.h"
#include "KeyenceHMI.h"


#define out_1 bitWrite(PORTC, 0, 1)//D37
#define in_1 (!bitRead(PINB, 0))||(hmi.HMI_Read_Bit(0/8,0%8)&&hmi.virtual_input_func)  //D53//註冊idx0的按鈕
#define in_2 (!bitRead(PINB, 1))||(hmi.HMI_Read_Bit(1/8,1%8)&&hmi.virtual_input_func)   //D52//註冊idx1的按鈕
//#include "KeyenceHMI.h"
// put function declarations here:
KeyenceHMI hmi(Serial2);//use serial 1
uint8_t auint8 = 0;

//測試用變數
int x_,v_,a_;
int out1,out2,out3;//接收HMI給的資料
bool F1,F2,F3;
int test_i = 0;
String abc = "";

//flag
bool eng_mode = false;
bool eng_run_mode = false;

void initial();
void OP_flow();
void eng_flow();

void Write_custom(int8_t* data, uint8_t length);
void Read_custom(int8_t* data, uint8_t length);
void print_IOstate();

void setup() {

  Serial.begin(115200);
  initial();
  Serial.println("hello world");
  
//print_IOstate();

  hmi.HMI_Init(115200,1);//1號設備代表測試專案，專門測試lib更改
  hmi.setDataHandler_Read(Read_custom);
  hmi.setDataHandler_Write(Write_custom);
  //放幾個虛擬IO來檢查功能
  hmi.set_virtual_IO(0,53);
  hmi.set_virtual_IO(1,52);
  hmi.set_virtual_IO(2,65);
  hmi.set_virtual_IO(3,67);
  hmi.set_virtual_IO(4,69);
  hmi.set_virtual_IO(5,54);//大急停
}

void loop() {
  //if(EMGstop){}
  //else if(EMGstop){}
  //else{
    if(eng_mode){
      eng_flow();
    }
    else{
      OP_flow();
    }

  //}
  while(Serial.available()){
    char temp = Serial.read();
    abc+=temp;
    Serial.print("input is:");
    Serial.println(abc);
  }

  

  hmi.HMI_Cyclic();
}

// put function definitions here:

void print_IOstate(){
  for (uint8_t pin = 0; pin < 70; ++pin) {
    uint8_t port = digitalPinToPort(pin);
    if (port == NOT_A_PIN) continue;

    uint8_t bit = digitalPinToBitMask(pin);
    volatile uint8_t* ddr = portModeRegister(port);
    volatile uint8_t* out = portOutputRegister(port);

    bool isOutput = (*ddr & bit);
    bool isPulledUp = (*out & bit);

    
    volatile uint8_t *in = portInputRegister(port);
      
    //bool pinState = isOutput ? (*out & bit) : (*in & bit);
    bool pinState = isOutput ? (*out & bit) : (isPulledUp? !(*in & bit):0);//input看pull up，沒有pullup代表有input

    //bool isPulledUp = (*out & bit);

    Serial.print("pin D number: ");
    Serial.print(pin);
    Serial.print("    in(0) or out(1): ");
    Serial.print(isOutput);
    Serial.print("    value is : ");
    Serial.println(pinState);
  }
}
void initial(){
  //IO初始化
  PORTA = 0;          //關閉全部IO
  PORTB = 0;
  PORTC = 0;
  PORTD = 0;
  PORTE = 0;
  PORTF = 0;
  PORTG = 0;
  PORTH = 0;
  PORTJ = 0;
  PORTK = 0;
  PORTL = 0;
//定義INPUT/OUTPUT  
//DDR 0 為IN 
//DDR 1 為OUT 
//DDR 0 and PORT 1 為IN_PULLUP
  
  DDRA = 0b01010100;      //定義INPUT/OUTPUT
  DDRB = 0b00000000;
  DDRC = 0b10001010;
  DDRD = 0b10000000;
  DDRE = 0b00000000;
  DDRF = 0b11000111;
  DDRG = 0b00000000;
  DDRH = 0b00000000;
  DDRJ = 0b00000000;
  DDRK = 0b00000001;
  DDRL = 0b00001010;

  PORTL = 0b01000000;//pull up D43
  PORTK = 0b10101000;
  ////////////////////////////////////////////////////////////////////////////////////
  // Mega 2560 T1,T3,T4,T5
  ////////////////////////////////////////////////////////////////////////////////////
  // TCCR1A
  //    7    |    6    |    5    |    4    |    3    |    2    |    1    |    0    |
  // COM1A1  | COM1A0  | COM1B1  | COM1B0  | COM1C1  | COM1C0  |  WGM11  |  WGM10  |
  ////////////////////////////////////////////////////////////////////////////////////
  // TCCR1B
  //    7    |    6    |    5    |    4    |    3    |    2    |    1    |    0    |
  //  ICNC1  |  ICES1  |    -    |  WGM13  |  WGM12  |  CS12   |  CS11   |  CS10   |
  ////////////////////////////////////////////////////////////////////////////////////
  // CS12-CS10
  // 000 NO_CLK         100 CPU/256
  // 001 CPU_CLK        101 CPU/1024
  // 010 CPU/8          110 EX Falling
  // 011 CPU/64         111 EX Rising
  ////////////////////////////////////////////////////////////////////////////////////
  // WGM13-20
  //       mod              TOP
  // 0000  Normal           0xFFFF
  // 0001  PWM/Phase 8b     0xFF
  // 0010  PWM/Phase 9b     0x1FF
  // 0011  PWM/Phase 10b    0x3FF
  // 0100  CTC              OCRnA
  // 0101  Fast PWM 8b      0xFF
  // 0110  Fast PWM 9b      0x1FF
  // 0111  Fast PWM 10b     0x3FF
  // 1000  PWM/Freq         ICRn
  // 1001  PWM/Freq         OCRnA
  // 1010  PWM/Phase        ICRn
  // 1011  PWM/Phase        OCRnA
  // 1100  CTC              ICRn
  // 1101  -                -
  // 1110  Fast PWM         ICRn
  // 1111  Fast PWM         OCRnA
  ////////////////////////////////////////////////////////////////////////////////////

  cli();    //關閉總中斷

  //Timer1 設定
  OCR1A = 38;
  TCCR1A = 0b00000011;      //模式
  TCCR1B = 0b00011010;      //除頻
  //TCCR3A = 0b00000000;      //模式  //抓周期用
  //TCCR3B = 0b00000001;      //除頻
  //TIMSK1 = (1 << OCIE1A);   //打開T1中斷

  sei();    //打開總中斷

}

void OP_flow(){

}
void eng_flow(){
  if(!eng_run_mode){

  }
  else{
    OP_flow();
  }
}


void Write_custom(int8_t* data, uint8_t length){//可用byte數 = length
  int * w1 = (int*) &data[0];
  int * w2 = (int*) &data[2];
  int * w3 = (int*) &data[4];
  *w1 +=1;
  *w2 = v_;
  *w3 = a_;
  //data寫入x v a

  //工程模式模板
  if(!eng_mode)
    hmi.HMI_Send_Bit(0,8,6);//01046 (71) 非工程模式時設定為0
  else{//in eng mode//01046 (71) 工程模式時設定為1
    hmi.HMI_Send_Bit(1,8,6);
    if(eng_run_mode)
      hmi.HMI_Send_Bit(0,8,7);//010'47' set 0
    else
      hmi.HMI_Send_Bit(1,8,7);//010'47' set 1
  }
}
// bool test0723_1 = false;
void Read_custom(int8_t* data, uint8_t length){//讀取的資料的byte數 = length
  int * r1 = (int*) &data[0];
  int * r2 = (int*) &data[2];
  int * r3 = (int*) &data[4];
  out1 = *r1;
  out2 = *r2;
  out3 = *r3;
  //處理hmi 開關訊號
  if(hmi.LDP(10)){//讀取到第1個按鈕有上升訊號
    F1 = true;
    hmi.HMIprint(F("press 10"));
  }
  if(hmi.LDF(10)){//讀取到第1個按鈕有下降訊號
    F1 = false;
    hmi.HMIprint(F("release 10"));
  }
  if(hmi.LDP(61)){//進出入工程模式s1的cmd
    if(eng_mode){
      if(!eng_run_mode){
        //R_String_cmd0 = "GoToStage1";
        hmi.HMIprint(F("press GoTo_eng_run_mode"));
      }
      else{
        //R_String_cmd0 = "GoToStage0";
        hmi.HMIprint(F("press GoTo_eng_origin"));
      }
    }
    else{
      hmi.HMIprint(F("(press GoToStage1) error state, not in eng mode 0"));
    }
  }


  if(hmi.LDP(62)){//進出工程模式的cmd
    if(!eng_mode){
      //R_String_cmd0 = "GEM";
      hmi.HMIprint("[press GEM]");
    }
    else{
      //R_String_cmd0 = "GOM";
      hmi.HMIprint("[press GOM]");
      hmi.virtual_input_func = false;
    }
  }
  // if(hmi.LDF(32)){//觸發發送訊息
  //   test0723_1 = !test0723_1;
  //   if(test0723_1){
  //     hmi.talk2HMI("hello world!123456sdf!/*-");
  //     Serial.println("hello world!123456sdf!/*-");
  //   }
  //   else{
  //     hmi.talk2HMI("abc:123,+-*");
  //     Serial.println("abc:123,+-*");
  //   }
  // }
}