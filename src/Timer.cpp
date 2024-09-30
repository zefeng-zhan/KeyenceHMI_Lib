#include "Timer.h"

// - Class constructor
Timer::Timer() {
}

void Timer::Set(uint32_t value) {
    // 已經Set過一次的話，不更動起始紀錄點，只更改計時到的點，仿PLC Timer設計
    if (m_Set_Value == 0) { 
        m_Stamp = millis();
    }
    m_Set_Value = value;
}

void Timer::Reset() {
    // 用m_Set_Value來判別Timer是否開關就好，不另設變數
    m_Set_Value = 0;
}

bool Timer::Check() {
    // 類PLC LD Timer的動作，回傳Timer是否到了設定的時間
    if (m_Set_Value > 0) {
        return (millis() - m_Stamp ) > m_Set_Value;
    }
    return false;
}

uint32_t Timer::Read_Value() {
    // 可用來算從Timer啟動後的時間(正數)
    return (millis() - m_Stamp);
}