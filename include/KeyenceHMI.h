#pragma once
#include <Arduino.h>
#include <EEPROM.h>
#include "Timer.h"

/*
**  version v1.00 2024.07.05
*/

//#define HMI_Serial Serial2  // 定義HMI Serial 要使用一組UART
#define HMI_SEND_LENGTH 32  // HMI單次發送byte數 發送是以Word發送，故一定要是偶數
#define HMI_SEND_PAGE 2     // HMI發送page數(乘上byte數就是全部同步數據的數量，上限為16共512個byte 256個word)
#define HMI_READ_LENGTH 32  // HMI單次讀取byte數
#define HMI_READ_PAGE 2     // HMI讀取page數(乘上byte數就是全部同步數據的數量，上限為16共512個byte 256個word)
#define HMI_TIMEOUT 5       // 逾時Timer，單位ms，多久Serial沒收到訊號標示為Timeout，同時用來當收送間的Delay
#define vIOmode 2//1    //0910新增，用來切換舊版1和新版2

enum class HMICommState {
  SEND_COMMAND,         // 發送寫入指令
  WAIT_SEND_DONE,       // 等待寫入指令送完
  SEND_COMMAND_CHECK,   // 接收寫入ACK
  READ_COMMAND,         // 發送讀取指令
  READ_RECEIVE,         // 接收讀取資料
  PROCESS,              // 成功接收後資料處理
};

class KeyenceHMI {
    public:
        //KeyenceHMI();        
        KeyenceHMI(HardwareSerial& serial);//有修改過，改可以外部引入和決定serial，而不是在這裡面設定
        void HMI_Init(long baudRate,uint16_t num);      // HMI 初始化
        void HMI_Cyclic();              // HMI 物件循環流程

        // 寫入HMI交握變數
        void HMI_Send_Bit(bool value, uint8_t address, uint8_t bitnum); // 寫入Bit
        void HMI_Send_Byte(int8_t value, uint8_t address);              // 寫入Byte
        void HMI_Send_Word(int16_t value, uint8_t address);             // 寫入Word (2 byte)
        void HMI_Send_Dword(int32_t value, uint8_t address);            // 寫入2 Word (4 byte)

        // 讀取HMI交握變數
        bool HMI_Read_Bit(uint8_t address, uint8_t bitnum); // 讀取Bit
        int8_t HMI_Read_Byte(uint8_t address);              // 讀取Byte
        int16_t HMI_Read_Word(uint8_t address);             // 讀取Word (2 byte)
        int32_t HMI_Read_Dword(uint8_t address);            // 讀取2 Word (4 byte)


//ˇ---------------------新增功能---------------------ˇ
        using DataHandlerFunction = void(*)(int8_t* data, uint8_t length);//有改data型態，char->int8_t
        void setDataHandler_Read(DataHandlerFunction handler);
        void setDataHandler_Write(DataHandlerFunction handler);

        
        bool LDP(char id);
        bool LDF(char id);
        void set_virtual_IO(uint8_t VT_btn, int8_t D_pin);//0706
        bool virtual_input_func = false;
        void talk2HMI(String s);
        void HMIprint(String s);

        //公用資料
        //要存取pin狀態，是因為後續每次更新，都要根據狀態來判斷要讀取port還是pin
        //有無pullup也先存取 (但程式目前只有針對pullup的功能)
        bool _pin_mode[70] = {0};//output 1, input 0
        bool _pin_pullup[70] = {0};//up 1, no or output 0
        //testing function
        void print_sbuf();
        void print_rbuf();
        void print_vin();
    private:
        HardwareSerial& _serial;
        int8_t VTbtn_vio_map[64] = {0}; //0706//目前預設面板上最多做32個按鈕
        uint8_t LastState[8] = {0};//64/8
        uint16_t fixture_type = 0;

        DataHandlerFunction _dataHandler_read;
        DataHandlerFunction _dataHandler_write;
        void readPinModes(bool isfirst);
        bool sendOnce(char start,char start2);
        void set_FN(uint16_t num);
        void w_EEPROM_PN();//財產編號 property number

        void updatePinStates();
        void process_virtual_IO();//0706
        void set_vb(uint8_t p);
        void clear_vb(uint8_t p);
        void out_vb(uint8_t p);

        //HMI_print_core相關
        void HMI_print_core();//0801
        void HMI_print_core2();//0923
        uint16_t scount= 0;
        uint8_t reset_count= 4;
        String temp_send_str;
        uint8_t sptr = 0;
        bool send2HMI_finished = true;
        String tss_array[8];
        uint8_t tss_rx = 0;
        uint8_t tss_wx = 0;
        
        //檢查連線的變數
        bool VT_is_connect = false;//判斷有無跟人機連接
        uint8_t resend_count = 0;//紀錄cyclic中，send fail次數，如果太高，就會判定為disconnect
//^---------------------新增功能---------------------^

        void HMI_Send_Command();        // 發送Arduino內變數寫入HMI請求封包
        void HMI_Wait_Send_Done();      // 等待寫入HMI請求封包發送完畢
        void HMI_Send_Command_Check();  // 確認HMI有正確ACK寫入封包
        void HMI_Read_Command();        // 發送讀取HMI內部變數請求封包
        void HMI_Read_Receive();        // 接收HMI送回的數值
        void HMI_Process();
        void HMI_Flow_Jump(HMICommState state);
        
        HMICommState m_HMI_State;
        int8_t m_HMI_Send[HMI_SEND_LENGTH * HMI_SEND_PAGE] = {0};
        int8_t m_HMI_ReadBuffer[HMI_READ_LENGTH + 8] = {0};
        int8_t m_HMI_Read[HMI_READ_LENGTH * HMI_READ_PAGE] = {0};
        uint8_t m_HMI_Serial_Write_Buffer_Max = 0;
        uint8_t m_HMI_Read_Index = 0;
        uint8_t m_HMI_Page = 0;
        Timer m_HMI_Timeout;
};