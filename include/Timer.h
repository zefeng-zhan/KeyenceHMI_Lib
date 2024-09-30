#pragma once
#include <Arduino.h>

/*
** version v1.00 2024.07.05
** 系統時millis為1ms +1的 32bit無符號整數，爆表一輪的時間約為49.7天，正常使用下應該很難會遇到溢位的狀態
*/

class Timer {
    public:
        Timer();
        void Set(uint32_t value);   // 設定Timer On的時間，單位:ms
        void Reset();               // 重設Timer
        bool Check();               // 讀取Timer狀態 true = 計時時間到
        uint32_t Read_Value();      // 讀取Timer ON到現在經過的時間，單位ms

    private:
        uint32_t m_Set_Value = 0;   // Timer啟動的系統時間戳記
        uint32_t m_Stamp = 0;       // Timer計時時間，單位ms
};