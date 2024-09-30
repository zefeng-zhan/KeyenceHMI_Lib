#include <Arduino.h>
#include "KeyenceHMI.h" 

// - Class constructor
KeyenceHMI::KeyenceHMI(HardwareSerial& serial) : _serial(serial) {
    
}

// - Initialize HMI serial port
void KeyenceHMI::HMI_Init(long baudRate,uint16_t num) {//放在pin mode initial後面!!
    _serial.begin(baudRate, SERIAL_8N1);
    m_HMI_State = HMICommState::SEND_COMMAND;
    m_HMI_Serial_Write_Buffer_Max = _serial.availableForWrite(); // 取得初始的BufferMax來判斷Buffer最大值
    fixture_type = num;
    //HMI_print_core相關初始化
    scount= 0;
    reset_count= 4;
    sptr = 0;
    send2HMI_finished = false;//new core is false,

    //送pin mode過去
    readPinModes(true);
    set_FN(fixture_type);
    VT_is_connect = sendOnce(0x03,0x00);//包裝了SEND_COMMAND + WAIT_SEND_DONE + SEND_COMMAND_CHECK，會回傳是否傳送成功

}
void KeyenceHMI::set_FN(uint16_t num){//fixture number 治具種類編號，方便插上去就可以看到編號知道機台種類，順便送財編
    HMI_Send_Word(num,9);//種類編號 (讓VT那邊顯示，知道這塊MCU載入哪包程式、切換頁面等)

    int32_t PN = (long(EEPROM.read(4000))<<24) + (long(EEPROM.read(4001))<<16) + (long(EEPROM.read(4002))<<8) + (long(EEPROM.read(4003)));//財編6位數字用4byte
    HMI_Send_Dword(PN,5);//word 030A
}
// - HMI Communication flow
void KeyenceHMI::HMI_Cyclic() {
    switch(m_HMI_State) {
        case HMICommState::SEND_COMMAND:
            HMI_Send_Command();
            break;
        case HMICommState::WAIT_SEND_DONE:
            HMI_Wait_Send_Done();
            break;
        case HMICommState::SEND_COMMAND_CHECK:
            HMI_Send_Command_Check();    
            break;
        case HMICommState::READ_COMMAND:
            HMI_Read_Command();
            break;
        case HMICommState::READ_RECEIVE:
            HMI_Read_Receive();
            break;
        case HMICommState::PROCESS:
            HMI_Process();
            break;
    }   
}

void KeyenceHMI::HMI_Send_Command() {   // 發送Arduino內變數寫入HMI封包
    int8_t checksum = (HMI_SEND_LENGTH + 5 + 'W' + 'W' + 0x00 + m_HMI_Page * HMI_SEND_LENGTH / 2 + 0x01 + HMI_SEND_LENGTH / 2 + 0x03); // 先把其他加進來，其實會是常數，編譯器會先預處理掉
    // 寫入標頭
    _serial.print((char) 0x02);                                           // HMI通訊固定標頭STX (02H)
    _serial.print((char) 0x00);                                           // HMI VTX 編號
    _serial.print((char) (HMI_SEND_LENGTH + 5));                          // 數據長度 = 總寫入Byte數 + 指令(2Byte) + 元件編號(2Byte) + 數據個數(1Byte)
    _serial.print('W');                                                   // 設定寫入模式 WW
    _serial.print('W');                                                   // 設定寫入模式 WW
    _serial.print((char) (0x00 + m_HMI_Page * HMI_READ_LENGTH / 2));      // 寫入軟元件編號-1  寫入固定從MW0100開始+Page 因為HMI是用Word計算，故要除以2
    _serial.print((char) 0x01);                                           // 寫入軟元件編號-2
    _serial.print((char) (HMI_SEND_LENGTH / 2));                          // 數據個數，寫入Word數 (總Byte數/2)
    // 寫入資料
    for (int i = 0; i < HMI_SEND_LENGTH; i++) {                             // 把交換資料丟出去
        _serial.print((char) m_HMI_Send[i + m_HMI_Page * HMI_READ_LENGTH]);
        checksum += m_HMI_Send[i + m_HMI_Page * HMI_READ_LENGTH];           // 計算Check Sum

    }
    // 寫入結尾
    _serial.print((char) 0x03);                                           // HMI通訊固定結尾 ETX (03H)
    _serial.print((char) checksum);                                       // 送出checksum
        
    HMI_Flow_Jump(HMICommState::WAIT_SEND_DONE);                            // 跳至WAIT_SEND_DONE流程
}

void KeyenceHMI::HMI_Wait_Send_Done() {
    if (_serial.availableForWrite() == m_HMI_Serial_Write_Buffer_Max) {
        HMI_Flow_Jump(HMICommState::SEND_COMMAND_CHECK);
    }
}

void KeyenceHMI::HMI_Send_Command_Check() {
    while (_serial.available() && m_HMI_Read_Index < 10) {           // 正常ACK長度僅有8個byte，超過10個直接走Timeout流程
        m_HMI_Timeout.Reset();                                          // 有資料的話重置逾時Timer
        m_HMI_ReadBuffer[m_HMI_Read_Index] = (char)_serial.read();   // 把資料放入Read buffer
        m_HMI_Read_Index++;
    }
    m_HMI_Timeout.Set(HMI_TIMEOUT); // 讀取完後開始計算Timeout

    // 響應格式：
    // |      0      |      1      |      2      |      3      |      4      |      5      |      6      |      7      |
    // |  STX(0x02)  | VTNo.(0x00) |   Length    |     'W'     |     'W'     |  Response   |  EXT(0x03)  |  Checksum   |
    // Length 數據長度只包含 3,4,5的部分，所以固定為3
    // Response code 響應代碼： 00=正常 01=命令格式錯誤 02=指定了軟元件編號範圍以外的值
    // Checksum 從 VTNo.累加到EXT 不包含STX，基本上成功也會是固定值
    
    if (m_HMI_Timeout.Check()) { // Timeout後才來判斷，一方面當Serial接受與發送中間的間隔用
        if (m_HMI_ReadBuffer[0] == 0x02 && m_HMI_ReadBuffer[1] == 0x00 && m_HMI_ReadBuffer[2] == 0x03 &&
            m_HMI_ReadBuffer[3] == 'W' && m_HMI_ReadBuffer[4] == 'W' && m_HMI_ReadBuffer[5] == 0x00 &&
            m_HMI_ReadBuffer[6] == 0x03 && m_HMI_ReadBuffer[7] == (int8_t)(0 + 3 + 'W' + 'W' + 0 + 3)) {
            
            // 讀完後處理資料
            m_HMI_Page++;                       // 繼續傳下一頁
            if (m_HMI_Page >= HMI_SEND_PAGE) { // 全部寫完的話在跳去讀取
                m_HMI_Page = 0;
                if(!VT_is_connect){
                    Serial.println("Reconnect (Send)");
                }
                HMI_Flow_Jump(HMICommState::READ_COMMAND);               
            }
            else {
                HMI_Flow_Jump(HMICommState::SEND_COMMAND);
            }
        }
        else // 不符合格式的話，判定Fail，重新再寫入一次
        {
            HMI_Flow_Jump(HMICommState::SEND_COMMAND);
            if(resend_count>10){
                VT_is_connect = false;
            }
            else{
                resend_count++;
                if(resend_count==10)
                    Serial.println("Send check fail, resend (10th) (S)");
            }
        }
    }
}

void KeyenceHMI::HMI_Read_Command() {   // 發送讀取HMI內變數請求封包
    int8_t checksum = (0x05 + 'R' + 'W' + 0x00 + m_HMI_Page * HMI_READ_LENGTH / 2 + 0x02 + HMI_READ_LENGTH / 2 + 0x03);   // 先把其他加進來，其實會是常數，編譯器會先預處理掉
    _serial.print((char)0x02);                                                                                         // HMI通訊固定標頭 STX (02H)
    _serial.print((char)0x00);                                                                                         // VT No.
    _serial.print((char)0x05);                                                                                         // 數據長度 = 5
    _serial.print((char)'R');                                                                                          // 讀取模式 RW
    _serial.print((char)'W');                                                                                          // 讀取模式 RW
    _serial.print((char)(0x00 + m_HMI_Page * HMI_READ_LENGTH / 2));                                                    // 讀取軟元件編號-1  讀取固定從MW0200開始
    _serial.print((char)0x02);                                                                                         // 讀取軟元件編號-2
    _serial.print((char)(HMI_READ_LENGTH / 2));                                                                        // 讀取數據個數(word)，故/2  MW0200~MW020F
    _serial.print((char)0x03);                                                                                         // HMI通訊固定標頭 EXT (03H)
    _serial.print((char)checksum);                                                                                     // 數據個數，寫入Word數 (總Byte數/2)
    
    HMI_Flow_Jump(HMICommState::READ_RECEIVE);
}

void KeyenceHMI::HMI_Read_Receive() {
    while (_serial.available() && m_HMI_Read_Index < (HMI_READ_LENGTH + 8)) {    // 如果有異常長度的話直接走Timeout流程
        m_HMI_Timeout.Reset(); // 有資料的話重置Timer
        m_HMI_ReadBuffer[m_HMI_Read_Index] = (char)_serial.read();
        m_HMI_Read_Index++;
    }
    m_HMI_Timeout.Set(HMI_TIMEOUT); // 讀取完後開始計算Timeout

    // 響應格式：
    // |      0      |      1      |      2      |      3      |      4      |      5      |      6      |      7      |...|     N+5     |     N+6     |     N+7     |
    // |  STX(0x02)  | VTNo.(0x00) |   Length    |     'R'     |     'W'     |  Response   |    Data1    |    Data2    |...|    DataN    |  EXT(0x03)  |  Checksum   |
    // Length 數據長度只包含 3到N+5的部分，所以固定為 _HMI_READ_BYTE+3
    // Response code 響應代碼： 00=正常 01=命令格式錯誤 02=指定了軟元件編號範圍以外的值
    // Checksum 從 VTNo.累加到EXT 不包含STX

    if (m_HMI_Timeout.Check()) {    // Timeout後才來判斷，一方面當Serial接受與發送中間的間隔用
        int8_t checksum = 0;
        if (m_HMI_Read_Index == HMI_READ_LENGTH + 8 &&      // 判斷接收數量有符合
            m_HMI_ReadBuffer[0] == 0x02 &&                  // 首字為STX(02H)，表示響應
            m_HMI_ReadBuffer[2] == HMI_READ_LENGTH + 3 &&   // 讀取長度符合
            m_HMI_ReadBuffer[5] == 0x00)                    // Response code 00表示正常
            {
                for (int i = 1; i <= HMI_READ_LENGTH + 6; i++) { // 計算checksum
                    checksum = checksum + m_HMI_ReadBuffer[i];
                }

                if (m_HMI_ReadBuffer[HMI_READ_LENGTH + 7] == checksum) {
                    for (int i = 0; i < HMI_READ_LENGTH; i++) {                                     // checksum判斷正確後，再把buffer搬入正式讀值區域
                        m_HMI_Read[i + m_HMI_Page * HMI_READ_LENGTH] = m_HMI_ReadBuffer[i + 6];     // ReadBuffer要加上通訊標頭的偏移
                    }

                    m_HMI_Page++;
                    if (m_HMI_Page == HMI_READ_PAGE) { // 是不是全部的Page都讀取過一輪了
                        m_HMI_Page = 0;
                        HMI_Flow_Jump(HMICommState::PROCESS); // 已經讀完就跳Process
                    }
                    else {
                        if(!VT_is_connect){
                            Serial.println("Reconnect (Read)");
                        }
                        HMI_Flow_Jump(HMICommState::READ_COMMAND); // 還沒讀完繼續讀
                    }
                }
        }
        else {
            while(_serial.available()){//把多餘的RX buffer 清掉
                char eat;
                eat = (char)_serial.read();
            }
            //重新插拔發現常常卡住，在這裡檢視後推測buffer不乾淨，因此加上eat
            
            /*Serial.println("Fail format (Read)");
            Serial.println(m_HMI_Read_Index);
            Serial.println(m_HMI_ReadBuffer[0]);
            Serial.println(m_HMI_ReadBuffer[1]);
            Serial.println(m_HMI_ReadBuffer[2]);
            Serial.println(m_HMI_ReadBuffer[3]);
            Serial.println(m_HMI_ReadBuffer[4]);
            Serial.println(m_HMI_ReadBuffer[5]);*/
        }
        if (m_HMI_State == HMICommState::READ_RECEIVE) { // 上一條件式沒有判讀成功，所以狀態還留在Read Receive的話，就重新再發一次ReadCommand
            HMI_Flow_Jump(HMICommState::READ_COMMAND);
            if(resend_count>10){
                VT_is_connect = false;
            }
            else{
                resend_count++;
                if(resend_count==10)
                    Serial.println("Read check fail, resend (10th) (R)");
            }
        }
    }
}

void KeyenceHMI::HMI_Flow_Jump(HMICommState state) {
    m_HMI_State = state;
    m_HMI_Read_Index = 0;   // 讀取Index歸零
    m_HMI_Timeout.Reset();  // 清除Timeout timer
}

void KeyenceHMI::HMI_Send_Bit(bool value, uint8_t address, uint8_t bitnum) {
    // 檢查記憶體位置有無超出範圍，避免寫到奇怪的地方炸掉
    if (address < HMI_SEND_LENGTH * HMI_SEND_PAGE) {        
        if (value) {
            m_HMI_Send[address] |= _BV(bitnum);
        }
        else {
            m_HMI_Send[address] &= ~(_BV(bitnum));
        }
    }
}

void KeyenceHMI::HMI_Send_Byte(int8_t value, uint8_t address) {
    // 檢查記憶體位置有無超出範圍，避免寫到奇怪的地方炸掉
    if (address < HMI_SEND_LENGTH * HMI_SEND_PAGE) {
        m_HMI_Send[address] = value;
    }
}

void KeyenceHMI::HMI_Send_Word(int16_t value, uint8_t address) {
    // 檢查記憶體位置有無超出範圍，避免寫到奇怪的地方炸掉
    if (address < HMI_SEND_LENGTH * HMI_SEND_PAGE / 2) {
    int16_t *ptr;
    ptr = (int16_t *)&m_HMI_Send[address * 2];
    *ptr = value;
  }
}

void KeyenceHMI::HMI_Send_Dword(int32_t value, uint8_t address) {
    // 檢查記憶體位置有無超出範圍，避免寫到奇怪的地方炸掉
    if (address < HMI_SEND_LENGTH * HMI_SEND_PAGE / 4) {
        int32_t *ptr;
        ptr = (int32_t *)&m_HMI_Send[address * 4];
        *ptr = value;
    }
}

bool KeyenceHMI::HMI_Read_Bit(uint8_t address, uint8_t bitnum)
{
    // 讀取因為讀錯不會爆炸，只會讀到奇怪的值，就不檢查了
    return m_HMI_Read[address] >> bitnum & 0x01;;
}

int8_t KeyenceHMI::HMI_Read_Byte(uint8_t address)
{
    return m_HMI_Read[address];
}

int16_t KeyenceHMI::HMI_Read_Word(uint8_t address)
{
    int16_t *ptr;
    ptr = (int16_t *)&m_HMI_Read[address * 2];
    return *ptr;
}

int32_t KeyenceHMI::HMI_Read_Dword(uint8_t address)
{
    int32_t *ptr;
    ptr = (int32_t *)&m_HMI_Read[address * 4];
    return *ptr;
}

void KeyenceHMI::HMI_Process() {
    // 讀取資料的後處理(看read buffer變化)、下次發送前處理(填寫send buffer)
    if(!VT_is_connect){
        VT_is_connect = true;
        resend_count = 0;
        Serial.println("Reconnect (Process)");

        memset(m_HMI_Send, 0, 64);//資料clear
        readPinModes(false);
        set_FN(fixture_type);
        Serial.println("send Once");
        VT_is_connect = sendOnce(0x03,0x00);                    
    }

//跟read buffer (人機傳送的訊息，如按鈕bit狀態、word byte 數值輸入內容等) 有關
    if(LDP(60)&& (m_HMI_Send[8] >> 6 & 0x01)){//有按下vio_func和處於工程模式(要手動在custom部分把proj用的工程模式flag連接到01046)，可開關vio_func
        virtual_input_func = !virtual_input_func;
        if(!virtual_input_func){//如果觸發的是關閉按鈕，那就順便把那些VIO按鈕關閉
            //把按鈕關閉 (把虛擬IO用到的按鈕020XX系列，從1變0)，還沒想到怎麼實作
            //也可以不用，但是下次一打開IO，就會把上次狀態直接一次過去
            //可以在VT端做就好，用60這個按鈕加一堆set bit功能，開關VIO時，順便關閉vio的按鈕
            HMIprint("exit vif!");
        }
        else{
            HMIprint("into vif!");
        }
    }
    
    
    process_virtual_IO();//比較last state狀態，LDP/LDF，並且參考VTbtn_vio_map執行虛擬IO功能，但按鈕可不只設定為虛擬IO，其他功能要自行建立 (用call back fn)
    _dataHandler_read(& m_HMI_Read[8], HMI_READ_LENGTH * HMI_READ_PAGE-8);//前8byte備用再按紐
    w_EEPROM_PN();//算read，按鈕觸發後動作，查看read buffer最底端數字，放進EEPROM
    // 處理結束後更新狀態
    for(int i = 0;i < 8; i++){//更新last state
        LastState[i] = m_HMI_Read[i];
    }

//跟send buffer (給人機的訊息，要顯示在VT的變數、IO更新、HMI_print流程) 有關
    // read處理結束會後跳回 Send Command，這裡先統一處理要send的部分 
    HMI_print_core2();//send部位最先處理print，因為會覆蓋掉所有send buffer
    HMI_Send_Bit(virtual_input_func,9,0);//send "virtual_input_func" to VT 01048
    updatePinStates();
    _dataHandler_write(&m_HMI_Send[10],HMI_READ_LENGTH * HMI_READ_PAGE-10);//外部自定義
    
    
    HMI_Flow_Jump(HMICommState::SEND_COMMAND);
}

void KeyenceHMI::setDataHandler_Read(DataHandlerFunction handler) {
    _dataHandler_read = handler;
}
void KeyenceHMI::setDataHandler_Write(DataHandlerFunction handler) {
    _dataHandler_write = handler;
}

void KeyenceHMI::readPinModes(bool isfirst){
    if(isfirst){//0903 新增：重新插拔沒斷電只要讀暫存，暫存在剛開機會讀取，剛開機初始化才是正確狀態
        for (uint8_t pin = 0; pin < 70; ++pin) {
            uint8_t port = digitalPinToPort(pin);

            if (port == NOT_A_PIN) continue;

            uint8_t bit = digitalPinToBitMask(pin);
            
            bool isOutput = (*portModeRegister(port) & bit);
            bool isPulledUp = (*portOutputRegister(port) & bit);

            _pin_mode[pin] = isOutput ? OUTPUT : INPUT;
            _pin_pullup[pin] = isOutput ? 0:isPulledUp;


            uint8_t bufferIndex = pin / 4;
            uint8_t bitOffset = pin % 4;
            if (isOutput) {
                HMI_Send_Bit(1,bufferIndex,bitOffset*2);
            }
            else if(isPulledUp){
                HMI_Send_Bit(1,bufferIndex,bitOffset*2+1);
            }
        }
    }
    else{
        for (uint8_t pin = 0; pin < 70; ++pin) {
            uint8_t bufferIndex = pin / 4;
            uint8_t bitOffset = pin % 4;
            if (_pin_mode[pin]) {
                HMI_Send_Bit(1,bufferIndex,bitOffset*2);
                //_sendBuffer[bufferIndex] |= (1 << bitOffset*2);
            }
            else if(_pin_pullup[pin]){
                HMI_Send_Bit(1,bufferIndex,bitOffset*2+1);
                //_sendBuffer[bufferIndex] |= (1 << (bitOffset*2+1));
            }
        }
    }
}
bool KeyenceHMI::sendOnce(char start,char start2){
    bool send_sucess = false;
    uint8_t send_times = 0;
    while(!send_sucess && send_times<10){
        int8_t checksum = (HMI_SEND_LENGTH + 5 + 'W' + 'W' + start2 + start + HMI_SEND_LENGTH / 2 + 0x03); // 先把其他加進來，其實會是常數，編譯器會先預處理掉
        // 寫入標頭
        _serial.print((char) 0x02);                                           // HMI通訊固定標頭STX (02H)
        _serial.print((char) 0x00);                                           // HMI VTX 編號
        _serial.print((char) (HMI_SEND_LENGTH + 5));                          // 數據長度 = 總寫入Byte數 + 指令(2Byte) + 元件編號(2Byte) + 數據個數(1Byte)
        _serial.print('W');                                                   // 設定寫入模式 WW
        _serial.print('W');                                                   // 設定寫入模式 WW
        _serial.print((char) start2);      // 寫入軟元件編號-1  寫入固定從MW0100開始+Page 因為HMI是用Word計算，故要除以2
        _serial.print((char) start);                                           // 寫入軟元件編號-2
        _serial.print((char) (HMI_SEND_LENGTH / 2));                          // 數據個數，寫入Word數 (總Byte數/2)
        // 寫入資料
        for (int i = 0; i < HMI_SEND_LENGTH; i++) {                             // 把交換資料丟出去
            _serial.print((char) m_HMI_Send[i]);
            checksum += m_HMI_Send[i];           // 計算Check Sum

        }
        // 寫入結尾
        _serial.print((char) 0x03);                                           // HMI通訊固定結尾 ETX (03H)
        _serial.print((char) checksum);                                       // 送出checksum

        while(_serial.availableForWrite() != m_HMI_Serial_Write_Buffer_Max){}
        while (_serial.available() && m_HMI_Read_Index < 10) {           // 正常ACK長度僅有8個byte，超過10個直接走Timeout流程
            m_HMI_Timeout.Reset();                                          // 有資料的話重置逾時Timer
            m_HMI_ReadBuffer[m_HMI_Read_Index] = (char)_serial.read();   // 把資料放入Read buffer
            m_HMI_Read_Index++;
        }
        m_HMI_Timeout.Set(HMI_TIMEOUT); // 讀取完後開始計算Timeout
            // 響應格式：
            // |      0      |      1      |      2      |      3      |      4      |      5      |      6      |      7      |
            // |  STX(0x02)  | VTNo.(0x00) |   Length    |     'W'     |     'W'     |  Response   |  EXT(0x03)  |  Checksum   |
            // Length 數據長度只包含 3,4,5的部分，所以固定為3
            // Response code 響應代碼： 00=正常 01=命令格式錯誤 02=指定了軟元件編號範圍以外的值
            // Checksum 從 VTNo.累加到EXT 不包含STX，基本上成功也會是固定值
        while(!m_HMI_Timeout.Check()){}
        if (m_HMI_Timeout.Check()) { // Timeout後才來判斷，一方面當Serial接受與發送中間的間隔用
            if (m_HMI_ReadBuffer[0] == 0x02 && m_HMI_ReadBuffer[1] == 0x00 && m_HMI_ReadBuffer[2] == 0x03 &&
                m_HMI_ReadBuffer[3] == 'W' && m_HMI_ReadBuffer[4] == 'W' && m_HMI_ReadBuffer[5] == 0x00 &&
                m_HMI_ReadBuffer[6] == 0x03 && m_HMI_ReadBuffer[7] == (int8_t)(0 + 3 + 'W' + 'W' + 0 + 3)) {
                
                memset(m_HMI_Send, 0, HMI_SEND_LENGTH * HMI_SEND_PAGE);//資料clear
                Serial.println("OK memset 0"); 
                send_sucess = true;
            //完整flow jump 樣貌：需要清掉timout(但實際也可不清，因為下次弄也會reset)、清掉Read_Index (很重要，不然下次一定也是失敗)
                m_HMI_Read_Index = 0;   // 讀取Index歸零
                m_HMI_Timeout.Reset();  // 清除Timeout timer
            }
            else // 不符合格式的話，判定Fail，重新再寫入一次
            {
                //Serial.println("fail, resend");
            //完整flow jump 樣貌：需要清掉timout(但實際也可不清，因為下次弄也會reset)、清掉Read_Index (很重要，不然下次一定也是失敗)
                m_HMI_Read_Index = 0;   // 讀取Index歸零
                m_HMI_Timeout.Reset();  // 清除Timeout timer
            }
        }
        send_times ++;
    }
    return send_sucess;
}

void KeyenceHMI::updatePinStates(){
    for (uint8_t pin = 0; pin < 70; ++pin) {
        if(!(_pin_mode[pin]||_pin_pullup[pin]))//跳過input沒pullup的腳位，代表該腳位沒被使用，因此不需更新狀態
            continue;
        
        uint8_t port = digitalPinToPort(pin);
        if (port == NOT_A_PIN) continue;

        uint8_t bit = digitalPinToBitMask(pin);
        volatile uint8_t *out = portOutputRegister(port);
        volatile uint8_t *in = portInputRegister(port);

        bool pinState;
        uint8_t vmode = vIOmode;
        if((vmode == 2) && virtual_input_func &&_pin_pullup[pin])//如果是虛擬input模式(先用工程模式7,14代替) + 為input腳位    //read(7,12) = 0203C，代表虛擬input flag，決定是否看虛擬input
            pinState = !(*in & bit)||(m_HMI_Send[pin/8] >> (pin%8) & 0x01);//如果現在的pin有pullup代表為input，跳過，因為虛擬in會在pvIO就send
        else
            pinState = _pin_mode[pin] ? (*out & bit) : !(*in & bit);//判斷為input就一定是pullup，因為非pullup被擋掉了
        //bool pinState = _pin_mode[pin] ? (*out & bit) : (_pin_pullup[pin]? !(*in & bit):0);//input看port，port為0代表有input。後者為無pullup，一律回傳0 (不使用)


        uint8_t bufferIndex = pin / 8;
        uint8_t bitOffset = pin % 8;

        HMI_Send_Bit(pinState,bufferIndex,bitOffset);

        
    }
}
void KeyenceHMI::set_virtual_IO(uint8_t VT_btn, int8_t D_pin){
    if(D_pin>=-70 && D_pin<=70)
        VTbtn_vio_map[VT_btn] = D_pin;
}
void KeyenceHMI::process_virtual_IO(){
    uint8_t vmode = vIOmode;
    for(int i = 0; i < 64;i++){
        int8_t pin = VTbtn_vio_map[i];
        if(pin == 0)//Dpin 0 不使用，當作看到0就跳過//0910更新，看到0代表結束了，之後都會是0，因此break
                break;
        if(vmode == 1){
            if(LDP(i)){//讀取到第i個按鈕有上升訊號
                if(pin>0)
                    set_vb(pin);//設為ouput 0//模擬的input按下和output關掉一樣
                else{
                    pin = -pin;
                    out_vb((uint8_t)pin);//設為ouput 1
                }
            }
            if(LDF(i)){//讀取到第i個按鈕有下降訊號
                if(pin>0)
                    clear_vb(pin);//設為ouput 1
                else{
                    pin = -pin;
                    set_vb((uint8_t)pin);//設為ouput 0
                }
            }
        }
        else{//mode==2
            if(pin<0){//output
                if(LDP(i)){
                    pin = -pin;
                    out_vb((uint8_t)pin);
                }
                if(LDF(i)){
                    pin = -pin;
                    set_vb((uint8_t)pin);//模擬的按下和output關掉一樣
                }
            }
            else if(pin>0 && virtual_input_func){//input
                uint8_t bufferIndex = i / 8;// + offset;
                uint8_t bitOffset = i % 8;
                if(HMI_Read_Bit(bufferIndex,bitOffset)){//如果這個按鈕邏輯為1，就要打開虛擬input bit
                    bufferIndex = pin / 8;// + offset;
                    bitOffset = pin % 8;
                    HMI_Send_Bit(1,bufferIndex,bitOffset);
                }
                else{//1是1，0是0，不要忘記0，才能把上次的資訊清掉
                    bufferIndex = pin / 8;// + offset;
                    bitOffset = pin % 8;
                    HMI_Send_Bit(0,bufferIndex,bitOffset);
                }
            }
        }
    }
}
bool KeyenceHMI::LDP(char id)    //上升沿信號 //id:readbuf 0~63 bit
{
  if(id>63||id<0){
    return false;
  }
  uint8_t index = id / 8;// + offset;//bit對應的byte
  uint8_t bit_num = id % 8;//bit位
  
    return ((LastState[index] & (1 << bit_num)) == 0) && HMI_Read_Bit(index,bit_num);//上次是0，這次是1

}

bool KeyenceHMI::LDF(char id)    //下降沿信號 //id:readbuf 0~63 bit
{
  if(id>63||id<0){
    return false;
  }
  uint8_t index = id / 8;// + offset;
  uint8_t bit_num = id % 8;
    return ((LastState[index] & (1 << bit_num)) ) && !HMI_Read_Bit(index,bit_num);//上次是1，這次是0
}

void KeyenceHMI::clear_vb(uint8_t p){//設定為input pulled up (初始狀態視為無input clear掉)
  uint8_t port = digitalPinToPort(p);
  uint8_t bit = digitalPinToBitMask(p);//ex:01000000 (6), 00100000 (5)
  volatile uint8_t *out = portOutputRegister(port);
  volatile uint8_t* ddr = portModeRegister(port);
  *ddr &= ~bit;
  *out |= bit;//set a bit 把digital30對應到的PORT設定為1
}

void KeyenceHMI::set_vb(uint8_t p){//inpullup模擬被按下：設定為out + port 0 (目的讓pin讀取為LOW，嘗試，單用input會變浮動bit)
  uint8_t port = digitalPinToPort(p);
  uint8_t bit = digitalPinToBitMask(p);//ex:01000000 (6), 00100000 (5)
  volatile uint8_t *out = portOutputRegister(port);
  volatile uint8_t* ddr = portModeRegister(port);
  //0709 順序應該不能錯，不然會有瞬間5V直接接GND短路 (所以先關掉out)
  *out &= ~bit;//clear a bit 把digital30對應到的PORT設定為0
  *ddr |= bit;
  
}
void KeyenceHMI::out_vb(uint8_t p){//output模擬被按下：設定為out + port 1 
  uint8_t port = digitalPinToPort(p);
  uint8_t bit = digitalPinToBitMask(p);//ex:01000000 (6), 00100000 (5)
  volatile uint8_t *out = portOutputRegister(port);
  volatile uint8_t* ddr = portModeRegister(port);
  *out |= bit;//clear a bit 把digital30對應到的PORT設定為0
  *ddr |= bit;
  
}

void KeyenceHMI::HMIprint(String s){
    s+='\n';//補尾巴
    if(s.length()>HMI_SEND_LENGTH-1){//還有結尾符號
        return;
    }
    if((tss_wx%8) +1 == (tss_rx%8)){}
    else{
        tss_array[tss_wx%8] = s;
        tss_wx ++;
    }
}
void KeyenceHMI::HMI_print_core2(){
    if(tss_rx != tss_wx){
        for(int i = 0;i<HMI_SEND_LENGTH;i++){//依序連入32個byte暫存
            if(send2HMI_finished){//當啟動時，但還沒跑完32，繼續填0進去
                HMI_Send_Byte(32,i);//不完全8個的話，補空白 (空格ascii為32)
                continue;
            }
            String temps01 = tss_array[tss_rx%8];
            char temp01 = (char)temps01[sptr++];
            if(temp01 == '\n'){//結尾條件  //i+sptr == temp_send_str.length()
                send2HMI_finished = true;//啟動show的flag
                //scount = 0;
                sptr = 0;
            }
            HMI_Send_Byte(temp01,i);//0500~050F
        }
        HMI_Send_Bit(1,HMI_SEND_LENGTH-1,7);
        //Serial.println((char)(0x00+(tss_rx%4)*16));
        if(!sendOnce(0x03,(0x40+(tss_rx%4)*(HMI_SEND_LENGTH/2))))//0500 0510 0520 0530 0500 0510 ...
            Serial.print("send once fail");
        send2HMI_finished = false;//變成判斷字串是否結束，
        tss_rx ++;
        //send 一個bit觸發位移，就可以顯示多行
    }
}

void KeyenceHMI::w_EEPROM_PN(){//財產編號 property number
    if(LDF(63)){
        int32_t PN = HMI_Read_Dword(15);//word 021E  ->  EEPROM  ->  word 030A
        
        EEPROM.update(4000,long(PN)>>24);
        EEPROM.update(4001,long(PN)>>16);
        EEPROM.update(4002,long(PN)>>8);
        EEPROM.update(4003,long(PN));

        //int32_t PN = (long(EEPROM.read(4000))<<24) + (long(EEPROM.read(4001))<<16) + (long(EEPROM.read(4002))<<8) + (long(EEPROM.read(4003)));
    }
}

void KeyenceHMI::print_sbuf(){
    Serial.println("this sbuf is:");
    /*
    for(int i = 0; i<HMI_READ_BYTE+10 ; i++){
        Serial.print("index: ");
        Serial.print(i);
        Serial.print("      data: ");
        Serial.println((uint8_t)_sendBuffer[i]);
    }
    */
    for(int i = 0; i<11 ; i++){//print前面A2V pinstate部分
        Serial.print("index: ");
        Serial.print(i);
        Serial.print("      data: ");
        Serial.println((uint8_t)m_HMI_Send[i]);
    }
  
}

void KeyenceHMI::print_rbuf(){
    Serial.println("(rrr)this rbuf is:");
    /*
    for(int i = 0; i<HMI_READ_BYTE+10 ; i++){
        Serial.print("index: ");
        Serial.print(i);
        Serial.print("      data: ");
        Serial.println((uint8_t)_readBuffer[i]);
    }
    */
    for(int i = 0; i<8 ; i++){//print前面V2A 按鈕部分
        Serial.print("index: ");
        Serial.print(i);
        Serial.print("      data: ");
        Serial.println((uint8_t)m_HMI_Read[i]);
    }
}

void KeyenceHMI::print_vin(){
    for(int i = 0; i<32 ; i++){
        Serial.print("index: ");
        Serial.print(i);
        Serial.print("      vinput: ");
        Serial.println((int8_t)VTbtn_vio_map[i]);
        
    } 
}