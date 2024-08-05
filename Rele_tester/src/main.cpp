#include "main.h"

#define MAX_CYCLE           1000
#define MAX_TIME_WAIT_us    5000000 // 5s    
#define WAIT1_us               100000 // 2s
#define WAIT2_us               1000000 // 2s
#define SIG_WAIT_us        3500 // 3.5 ms         

#define RELE_IN_PIN     2
#define RELE_HIGH       0
#define RELE_LOW        1

#define RELE_220_PIN    A1

#define B_START    0
#define B_STOP     1
#define B_RESTART  2

#define COIL_PIN    6

#define SD_SELECT 10

#define R_LED       9
#define G_LED       8
#define B_LED       7







String file_name;
// String name = "Test_";
File myFile;
LiquidCrystal_I2C lcd(0x27,16,2);
controller_event_t controller_event = {0};
controller_t controller = CONTROLLER_DEFAULT_INIT;
button_t buttons[3] = {BUTTON_DEFAULT_INIT, BUTTON_DEFAULT_INIT, BUTTON_DEFAULT_INIT};
rele_t rele = {0, eRELE_NONE};

controller_state tmp;

void controllerHandler(controller_t *c, controller_event_t *e, uint32_t rele_time) {
    if(e->eStop) {
        c->action = GO_STOP;
    }
    switch (c->state) {
    case IDLE:
        c->state = idleHandler(c, e);
        break;
    case INIT:
        c->state = initHadler(c);
        break;
    case MEAS_RELE_ON_TIME:
        c->state = releOnHandler(c, e, rele_time);
        break;
    case WAIT1:
        c->state = wait1Handler(c);
        break;
    case MEAS_RELE_OFF_TIME:
        c->state = releOffHandler(c, e, rele_time);
        break;
    case UPDATE:
        c->state = updateHandler(c);
        break;
    case WAIT2:
        c->state = wait2Handler(c);
        break;
    case STOP:
        c->state = stopHandler(c, e);
        break;
    case RESET:
        c->state = resetHandler(c, e);
        break;
    case ERROR:
        c->state = errorHandler(c);
        break;
    default:
        break;
    }
}

controller_state idleHandler(controller_t *c, controller_event_t *e) {
    controller_state next = IDLE;
    c->action = DONE;
    digitalWrite(COIL_PIN, LOW);
    if(e->eReset) {
        next = RESET;
    }
    else if(e->eStart) {
        next = INIT;
    }
    return next;
}

controller_state initHadler(controller_t *c) {
    noInterrupts();
    EEPROM.get(0, c->eeprom_data);
    // Serial.println("INIT STATE: READ EEPROM 1");
    // printEeprom(c);
    c->action = WORK;
    c->test_num = c->test_num + 1;
    c->current_i = 0;
    c->all_i = MAX_CYCLE;
    file_name = String("File_") + c->test_num + ".txt";
    if(SD.exists(file_name)) {
        Serial.println("File exists. Remove it and create new");
        SD.remove(file_name);
    }
    myFile = SD.open(file_name, FILE_WRITE);
    myFile.println("TIME ON; TIME OFF");
    myFile.close();
    EEPROM.put(0, c->eeprom_data);
    // Serial.println("INIT STATE: READ EEPROM 2");
    // printEeprom(c);
    interrupts();
    digitalWrite(RELE_220_PIN, HIGH);
    delay(20);
    return MEAS_RELE_ON_TIME;
}

controller_state releOnHandler(controller_t *c, controller_event_t *e, uint32_t rele_time) {
    controller_state next = MEAS_RELE_ON_TIME;
    if(digitalRead(COIL_PIN) == LOW) {
        digitalWrite(COIL_PIN, HIGH);
        timerUpdate_us(&c->timer);
    }
    else {
        if(timerRead_us(&c->timer) < MAX_TIME_WAIT_us) {
            if(e->eReleOn) {
                c->time_turn_on = rele_time - c->timer;
                timerUpdate_us(&c->timer);
                next = WAIT1;
            }
        }
        else {
            digitalWrite(COIL_PIN, LOW);
            digitalWrite(RELE_220_PIN, LOW);
            next = ERROR;
        }
    }
    return next;
}

controller_state releOffHandler(controller_t *c, controller_event_t *e, uint32_t rele_time) {
    controller_state next = MEAS_RELE_OFF_TIME;
    if(digitalRead(COIL_PIN) == HIGH) {
        digitalWrite(COIL_PIN, LOW);
        timerUpdate_us(&c->timer);
    }
    else {
        if(timerRead_us(&c->timer) < MAX_TIME_WAIT_us) {
            if(e->eReleOff) {
                c->time_turn_off = rele_time - c->timer;
                next = UPDATE;
            }
        }
        else {
            digitalWrite(COIL_PIN, LOW);
            digitalWrite(RELE_220_PIN, LOW);
            next = ERROR;
        }
    }
    return next;
}

controller_state updateHandler(controller_t *c) {
    controller_state next = WAIT2;
    noInterrupts();
    myFile = SD.open(file_name, FILE_WRITE);
    myFile.print(c->time_turn_on);
    myFile.print(";");
    myFile.println(c->time_turn_off);
    

    c->current_i++;

    
    if(c->current_i == c->all_i) {
        next = IDLE;
        c->action = DONE;
        myFile.print("Yspeh: ");
        myFile.print(c->current_i);
        myFile.print(" iz ");
        myFile.println(c->all_i);
    }
    else if(c->action == GO_STOP) {
        next = STOP;
        c->action = STOPED;
        digitalWrite(RELE_220_PIN, LOW);
    }
    myFile.close();
    EEPROM.put(0, c->eeprom_data);
    interrupts();
    timerUpdate_us(&c->timer);
    return next;

}

controller_state stopHandler(controller_t *c, controller_event_t *e) {
    controller_state next = STOP;
    if(e->eReset) {
        next = RESET;
    }
    else if(e->eStart) {
        digitalWrite(RELE_220_PIN, HIGH);
        next = MEAS_RELE_ON_TIME;
        delay(20);
    }
    return next;
}

controller_state errorHandler(controller_t *c) {
    noInterrupts();
    myFile = SD.open(file_name, FILE_WRITE);
    myFile.print("Ne yspeh: ");
    myFile.print(c->current_i);
    myFile.print(" iz ");
    myFile.println(c->all_i);
    myFile.close();
    c->action = DONE;
    c->current_i = 0;
    EEPROM.put(0, c->eeprom_data);
    interrupts();
    digitalWrite(RELE_220_PIN, LOW);
    return IDLE;
}

controller_state resetHandler(controller_t *c, controller_event_t *e) {
    uint8_t btn_yes = e->eStart;
    uint8_t btn_no = e->eStop;
    digitalWrite(RELE_220_PIN, LOW);
    controller_state next = RESET;
    if(btn_yes) {
        c->test_num = 0;
        c->current_i = 0;
        c->action = DONE;
        EEPROM.put(0, c->eeprom_data);
        next = IDLE;
    }
    else if(btn_no) {
        c->action = DONE;
        c->current_i = 0;
        EEPROM.put(0, c->eeprom_data);
        next = IDLE;
    }
    
    return next;
}

controller_state wait1Handler(controller_t *c) {
    controller_state next = WAIT1;
    if(timerRead_us(&c->timer) > WAIT1_us) {
        next = MEAS_RELE_OFF_TIME;
    }
    return next;
}

controller_state wait2Handler(controller_t *c) {
    controller_state next = WAIT2;
    if(timerRead_us(&c->timer) > WAIT2_us) {
        next = MEAS_RELE_ON_TIME;
    }
    return next;
}



void timerUpdate_us(uint32_t *tm) {
    *tm = micros();
}

uint32_t timerRead_us(uint32_t *tm) {
    return micros() - *tm;
}

rele_state_t releState(rele_t *rele, uint32_t wait_us) {
    rele_state_t res = RELE_NONE;
    detachInterrupt(0);

    if(micros() - rele->timer_us > wait_us) {
        if(rele->event == eRELE_HIGH)
            res = RELE_ON;
        else if(rele->event == eRELE_LOW)
            res =  RELE_OFF;
    }

    attachInterrupt(0, rele_isr, CHANGE);
    return res;
}

bool releOn(rele_t *rele) {
    return (releState(rele, SIG_WAIT_us) == RELE_ON);
}

bool releOff(rele_t *rele) {
    return (releState(rele, SIG_WAIT_us) == RELE_OFF);
}

void lcdUpdate(LiquidCrystal_I2C *lcd, controller_t *c) {
    static uint32_t timer = 0;
    if(millis() - timer > 1000) {
        timer = millis();
        if(c->state == RESET) {
            lcd->clear();
            lcd->home();
            lcd->print("Sbrosit' koli-");
            lcd->setCursor(0,1);
            lcd->print("chstvo testov?");
        }
        else {
            lcd->clear();
            lcd->home();
            lcd->print("N:");
            lcd->print(c->current_i);
            lcd->setCursor(9, 0);
            switch (c->action)
            {
            case DONE:
                lcd->print("DONE");
                break;
            case WORK:
            lcd->print("WORK");
                break;
            case GO_STOP:
                lcd->print("GO_STOP");
                break;
            case STOPED:
                lcd->print("STOPED");
                break;
            
            default:
                break;
            }
            lcd->setCursor(0,1);
            lcd->print("ON:");
            lcd->print((c->time_turn_on / 1000));
            lcd->print(" OFF:");
            lcd->print((c->time_turn_off / 1000));
        }
        

       

        
    }
}

void rgbSet(rgb_color_t color, bool level) {
    switch (color)
    {
    case RGB_RED:
        digitalWrite(R_LED, level);
        digitalWrite(G_LED, LOW);
        digitalWrite(B_LED, LOW);
        break;
    case RGB_GREEN:
        digitalWrite(R_LED, LOW);
        digitalWrite(G_LED, level);
        digitalWrite(B_LED, LOW);
        break;
    case RGB_BLUE:
        digitalWrite(R_LED, LOW);
        digitalWrite(G_LED, LOW);
        digitalWrite(B_LED, level);
        break;

    case RGB_OFF:
        digitalWrite(R_LED, LOW);
        digitalWrite(G_LED, LOW);
        digitalWrite(B_LED, LOW);
        break;
    
    
    default:
        break;
    }
}

bool rgbRead(rgb_color_t color) {
    bool res = false;
    switch (color)
    {
    case RGB_RED:
        res = (bool) digitalRead(R_LED);
        break;
    case RGB_GREEN:
        res = (bool) digitalRead(G_LED);
        break;
    case RGB_BLUE:
        res = (bool) digitalRead(B_LED);
        break;    
    default:
        break;
    }
    return res;
}

void rgbHandler(controller_action action) {
    static uint32_t timer = 0;
    switch (action)
    {
    case DONE:
        rgbSet(RGB_GREEN, HIGH);
        break;
    case WORK:
        if(millis() - timer > 500) {
            timer = millis();
            rgbSet(RGB_GREEN, !rgbRead(RGB_GREEN));
        }
        break;
    case GO_STOP:
        if(millis() - timer > 500) {
            timer = millis();
            rgbSet(RGB_BLUE, !rgbRead(RGB_BLUE));
        }
        break;
    case STOPED:
        rgbSet(RGB_BLUE, HIGH);
        break;
    
    default:
        break;
    }
}

bool printState(controller_state st) {
    static controller_state last_state = IDLE;
    if(last_state != st) {
        last_state = st;
        switch (st) {
        case IDLE:
            Serial.println("IDLE");
            break;
        case INIT:
            Serial.println("INIT");
            break;
        case MEAS_RELE_ON_TIME:
            Serial.println("MEAS_RELE_ON_TIME");
            break;
        case WAIT1:
            Serial.println("WAIT1");
            break;
        case MEAS_RELE_OFF_TIME:
            Serial.println("MEAS_RELE_OFF_TIME");
            break;
        case UPDATE:
            Serial.println("UPDATE");
            break;
        case WAIT2:
            Serial.println("WAIT2");
            break;
        case STOP:
            Serial.println("STOP");
            break;
        case RESET:
            Serial.println("RESET");
            break;
        case ERROR:
            Serial.println("ERROR");
            break;
        default:
            break;
        }
        return true;
    }
    return false;
}

void printEeprom(controller_t *c) {
    Serial.print("ACTION: ");
    Serial.println(c->action);
    Serial.print("I: ");
    Serial.println(c->current_i);
    Serial.print("NUM: ");
    Serial.println(c->test_num);
    Serial.print("ALL: ");
    Serial.println(c->all_i);
}

uint32_t tt = 0;

void setup() {
    Serial.begin(115200);
    // while (!Serial);
    pinMode(RELE_IN_PIN, INPUT_PULLUP);
    pinMode(COIL_PIN, OUTPUT);
    pinMode(R_LED, OUTPUT);
    pinMode(G_LED, OUTPUT);
    pinMode(B_LED, OUTPUT);
    pinMode(RELE_220_PIN, OUTPUT);
    digitalWrite(RELE_220_PIN, LOW);
    digitalWrite(COIL_PIN, LOW);
    digitalWrite(R_LED, HIGH);
    lcd.init();
    // lcd.backlight();
    SD.begin(SD_SELECT);
    
    buttonInit(&buttons[B_START], 3, INPUT_PULLUP);
    buttonInit(&buttons[B_STOP], 4, INPUT_PULLUP);
    buttonInit(&buttons[B_RESTART], 5, INPUT_PULLUP);
    noInterrupts();
    
    EEPROM.get(0, controller.eeprom_data);
    if(controller.action != DONE) {
        Serial.print("ACTION: ");
        Serial.println(controller.action);
        Serial.print("I: ");
        Serial.println(controller.current_i);
        Serial.print("NUM: ");
        Serial.println(controller.test_num);
        Serial.print("ALL: ");
        Serial.println(controller.all_i);
        controller.state = STOP;
        controller.action = STOPED;
        file_name = String("File_") + controller.test_num + ".txt";

        
    }
    interrupts();
    
    attachInterrupt(0, rele_isr, CHANGE);
    
    
}

void loop() {
    // Serial.println("Hello");
    controller_event.eStart = buttonClickR(&buttons[B_START]);
    controller_event.eStop = buttonClickR(&buttons[B_STOP]);
    controller_event.eReset = buttonHold(&buttons[B_RESTART], 1500);
    controller_event.eReleOn = releOn(&rele);
    controller_event.eReleOff = releOff(&rele);
    
    
    // if(releOn(&rele)) {
    //     if(millis()- tt > 1000) {
    //         tt = millis();
    //         Serial.println("RELE ON");
    //     }
    // }

    // if(releOff(&rele)) {
    //     if(millis()- tt > 1000) {
    //         tt = millis();
    //         Serial.println("RELE OFF");
    //     }
    // }

    controllerHandler(&controller, &controller_event, rele.timer_us);

    for(int i = 0; i < 3; i++) {
        buttonControler(&buttons[i], 100);
    }

    printState(controller.state);
    lcdUpdate(&lcd, &controller);
    rgbHandler(controller.action);
    // Serial.println(controller.state);    
    // delay(100);
    
}


void rele_isr(void) {
    uint16_t rele_sig = digitalRead(RELE_IN_PIN);
    if(rele_sig == RELE_HIGH) {
        rele.timer_us = micros();
        rele.event = eRELE_HIGH;
    }
    else if(rele_sig == RELE_LOW) {
        rele.timer_us = micros();
        rele.event = eRELE_LOW;
    }
}

