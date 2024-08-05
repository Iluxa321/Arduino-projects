#ifndef __MAIN__

#define __MAIN__

#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <EEPROM.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include "button.h"


#define CONTROLLER_DEFAULT_INIT {DONE, 0, 0, 10, IDLE, 0, 0, 0}

typedef enum {
    RELE_NONE,
    RELE_ON = 1,
    RELE_OFF
} rele_state_t;

typedef enum {
    eRELE_NONE,
    eRELE_HIGH,
    eRELE_LOW,
}rele_event_t;

typedef struct {
    volatile uint32_t timer_us;
    volatile rele_event_t event;
} rele_t;


typedef union {
    uint8_t all;
    struct {
        unsigned eStart : 1;
        unsigned eStop : 1;
        unsigned eReset : 1;
        unsigned eReleOn : 1;
        unsigned eReleOff : 1;
    };
}controller_event_t;

typedef enum {
    IDLE,
    INIT,
    MEAS_RELE_ON_TIME,
    WAIT1,
    MEAS_RELE_OFF_TIME,
    UPDATE,
    WAIT2,
    STOP,
    RESET,
    ERROR
}controller_state;

typedef enum {
    DONE=1,
    WORK,
    GO_STOP,
    STOPED
} controller_action;

typedef union controller_t {
    struct {
        // Save Eeprom
        controller_action action;
        uint8_t test_num;      
        uint16_t current_i;
        uint16_t all_i;
        // Don't Save
        controller_state state;
        uint32_t timer;
        uint32_t time_turn_on;
        uint32_t time_turn_off;
    };
    struct {
        controller_action action;
        uint8_t test_num;      
        uint16_t current_i;
        uint16_t all_i;
    }eeprom_data;
} controller_t;


typedef enum {
    RGB_OFF,
    RGB_RED,
    RGB_GREEN,
    RGB_BLUE,

}rgb_color_t;

void controllerHandler(controller_t *c, controller_event_t *e, uint32_t rele_time);
controller_state idleHandler(controller_t *c, controller_event_t *e);
controller_state initHadler(controller_t *c);
controller_state releOnHandler(controller_t *c, controller_event_t *e, uint32_t rele_time);
controller_state releOffHandler(controller_t *c, controller_event_t *e, uint32_t rele_time);
controller_state updateHandler(controller_t *c);
controller_state stopHandler(controller_t *c, controller_event_t *e);
controller_state errorHandler(controller_t *c);
controller_state resetHandler(controller_t *c, controller_event_t *e);
controller_state wait1Handler(controller_t *c);
controller_state wait2Handler(controller_t *c);

rele_state_t releState(rele_t *rele, uint32_t wait_us);

void timerUpdate_us(uint32_t *tm);
uint32_t timerRead_us(uint32_t *tm);

void printEeprom(controller_t *c);

void rgbSet(rgb_color_t color, bool level);
bool rgbRead(rgb_color_t color);
void rgbHandler(controller_action action);


void rele_isr(void);


#endif