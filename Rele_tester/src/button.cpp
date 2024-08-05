#include "button.h"



void buttonInit(button_t *btn, uint8_t pin, uint8_t mode) {
    pinMode(pin, mode);
    btn->pin = pin;
    btn->logic = mode == INPUT;
}

void buttonControler(button_t *btn, uint32_t wait1_ms) {
    switch (btn->state) {
    case B_ON:
        if(digitalRead(btn->pin) == btn->logic) {
            btn->event = B_PUSH;
            btn->state = B_ON;
        }
        else {
            btn->timer = millis();
            btn->state = B_WAIT_OFF;
        }
        break;
    case B_OFF:
        if(digitalRead(btn->pin) == !btn->logic) {
            btn->state = B_OFF;
            btn->event = B_RELIEZE;
        }
        else {
            btn->timer = millis();
            btn->state = B_WAIT_ON;
        }
        break;
    case B_WAIT_ON:
        if(digitalRead(btn->pin) == btn->logic) {
            if(millis() - btn->timer > wait1_ms) {
                btn->state = B_ON;
            }
            else {
                btn->state = B_WAIT_ON;
            }
        }
        else {
            btn->state = B_OFF;
        }
        break;
    case B_WAIT_OFF:
        if(digitalRead(btn->pin) == !btn->logic) {
            if(millis() - btn->timer > wait1_ms) {
                btn->state = B_OFF;
            }
            else {
                btn->state = B_WAIT_OFF;
            }
        }
        else {
            btn->state = B_ON;
        }
        break;
    default:
        break;
    }
}


bool buttonPush(button_t *btn) {
    return btn->event == B_PUSH;
}
bool buttonRelieze(button_t *btn) {
    return btn->event == B_RELIEZE;
}
bool buttonClickR(button_t *btn) {
    bool res = buttonPush(btn) & !btn->last_value1;
    btn->last_value1 = buttonPush(btn);
    return res;
}

bool buttonClickF(button_t *btn) {
    bool res = !buttonPush(btn) & !btn->last_value2;
    btn->last_value2 = !buttonPush(btn);
    return res;
}


bool buttonHold(button_t *btn, uint32_t hold_ms) {
    bool res = false;
    if(buttonPush(btn) && millis() - btn->timer > hold_ms) 
        res = true;
    return res;
}

bool buttonToggle(button_t *btn) {
    return buttonClickR(btn) ||  buttonClickF(btn);
}