#ifndef __BUTTON__

#define __BUTTON__

#include <Arduino.h>


#define BUTTON_DEFAULT_INIT {B_RELIEZE, B_OFF, 0, 0, false, false, false}

typedef enum {
    B_RELIEZE=1,
    B_PUSH,
}b_event_t;

typedef enum {
    B_ON=1,
    B_OFF,
    B_WAIT_ON,
    B_WAIT_OFF,
}b_state;



typedef struct {
    b_event_t event;
    b_state state;
    uint32_t timer;
    uint8_t pin;
    bool logic;
    bool last_value1;
    bool last_value2;
}button_t;


void buttonInit(button_t *btn, uint8_t pin, uint8_t mode);
void buttonControler(button_t *btn, uint32_t wait1_ms);
bool buttonPush(button_t *btn);
bool buttonRelieze(button_t *btn);
bool buttonClickR(button_t *btn);
bool buttonClickF(button_t *btn);
bool buttonHold(button_t *btn, uint32_t hold_ms);
bool buttonToggle(button_t *btn);


#endif