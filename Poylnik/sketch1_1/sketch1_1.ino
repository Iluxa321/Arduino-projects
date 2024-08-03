#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

/////////////////////////////////////////////////////////////
// Фиксированная точка
/////////////////////////////////////////////////////////////
/*typedef int16_t _iq;
#define _Q            (5)
#define _IQ5(A)       (int16_t)((A) * 32.0)
#define _IQ(A)        _IQ5(A)
#define _IQmpy(A, B)  (int16_t)(((int32_t)A * (int32_t)B) >> _Q)*/

typedef int32_t _iq;
#define _Q            (16)
#define _IQ16(A)       (int32_t)((A) * 65536.0)
#define _IQ(A)        _IQ16(A)
#define _IQmpy(A, B)  (int32_t)(((int64_t)A * (int64_t)B) >> _Q)
///////////////////////////////////////////////////////////////

#define temp A7                                             // ножка для считывания температуры термопары
#define deg_const     _IQ(5.0 / (1024.0 * 101.0 * 39e-6))   // коэффициент преобразования числа в температуру
#define kT            (310.0/185.0)                         // температурный коэффициент


#define CLK 8
#define DT 7 
#define SW 6
#define RIGHT 225
#define LEFT 210


long timer1 = 0; 
uint16_t disp_t = 185;

///////////////////////////////////////////////////////////////
// Переменные ПИД регулятора
///////////////////////////////////////////////////////////////
#define limMax    _IQ(2500.0)
#define limMin    _IQ(0.0)

#define kp  35.5
#define ki  2.5    // 77.16; 58.5; 12.5
#define kd  0.25    // 0.14; 0.25
#define T   40e-3


_iq Kp[2] = {_IQ(kp), _IQ(kp * 4)};
_iq Ki = _IQ(ki * T);
_iq Kd = _IQ(kd / T);


_iq pre_error = _IQ(0.0);
_iq setpoint = _IQ(disp_t * kT);
_iq derivative = _IQ(0.0);
_iq integral = _IQ(0.0);
_iq control = _IQ(0.0);
_iq proportional = _IQ(0.0);
/////////////////////////////////////////////////////////////////



/////////////////////////////////////////////////////////////////
// Фильтр среднего
/////////////////////////////////////////////////////////////////
typedef struct{
  uint8_t n; // порядок фильтра
  uint8_t i;
  uint32_t *arr; 
  uint32_t sum;
}filter_avg;

uint32_t arr[8] = {0};
filter_avg flt = {8, 0, arr, 0};

uint32_t arr2[32] = {0};
filter_avg flt_u = {32, 0, arr2, 0};

uint16_t fileter_handler(filter_avg *f, uint16_t x){
  f->sum -= f->arr[f->i];
  f->arr[f->i] = x;
  f->sum += f->arr[f->i];
  f->i = (f->i + 1) % f->n;
  return f->sum / f->n;
}
//////////////////////////////////////////////////////////////////



void encoderInit(){
  pinMode(CLK,INPUT_PULLUP);
  pinMode(DT,INPUT_PULLUP);
  pinMode(SW,INPUT_PULLUP);
}

uint8_t encoderRoll() {
  static uint8_t encoderState = 0;
  uint8_t currentState = 0;
  if(digitalRead(CLK) == HIGH) currentState |= (1<<0);
  if(digitalRead(DT) == HIGH) currentState |= (1<<1);
  if((encoderState & 0x03) == currentState) return 0;
  encoderState = (encoderState << 2) | currentState;
  if(encoderState == RIGHT) return 1;
  if(encoderState == LEFT) return 2;
  return 0;
}

uint16_t updateNum(uint16_t x, uint8_t dir, int16_t digital){
  if(dir == 1) x += digital;
  else if(dir == 2) x -= digital;
  return x;
}

void pwm1_init(void){
  // Инициализвация ШИМ
  pinMode(9, OUTPUT);
  // OC1A PWM
  // Fast PWM
  TCCR1B = 0;
  TCCR1A = 0;
  TCNT1 = 0;
  TCCR1B |= (1 << WGM13) | (1 << WGM12); 
  TCCR1A |= (1 << WGM11);
  TCCR1A |= (1 << COM1A1); // pwm non-inverting mode

  // 16М / 625 / 25 == 2500
  ICR1 = 2500;
  // Задем коэффициент заполенния
  OCR1A = 0;
  // Прерывание по совпадению с ICR1
  TIMSK1 |= (1 << 5);
  TCCR1B |= (1 << CS12) ; // 16M/256 = 62500
}
void fix_print(_iq a){
  Serial.print(a >> _Q);
  Serial.print(".");
  _iq tmp = ((_iq)(a & 0x1F) * 1250) >> 2;
  Serial.print(tmp / 1000);
  Serial.print(tmp % 1000 / 100);
  Serial.print(tmp % 100 / 10);
  Serial.println(tmp % 10);
  
}
void setup() {
  Serial.begin(9600);
  pwm1_init();
  encoderInit();
  lcd.init();                      // Инициализация LCD и шины I2C 
  lcd.backlight(); 
  lcd.clear();
  //lcd.setCursor(0,0);
  //lcd.print(5.23);
  //pinMode(9, OUTPUT);
  //digitalWrite(9, LOW);
  //fix_print(setpoint);
}

void loop() {
  if(millis() - timer1 > 500){
    //Serial.println(((float)(flt.sum / flt.n) * 5.0 / 1024.0 / 101 / 39e-6 / kT));
    //int16_t e = analogRead(temp);
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("tT: ");
    lcd.print((int)((float)(flt.sum / flt.n) * 5.0 / 1024.0 / 101 / 39e-6 / kT));
    //lcd.print(" C");
    lcd.print(", tY: ");
    lcd.print(disp_t);
    //lcd.print(" C");
    float i = flt_u.sum / flt_u.n;
    i = i / 2500.0;
    i = i * 24.0;
    lcd.setCursor(0,1);
    lcd.print("Tok: ");
    lcd.print(i / 20.0);
    lcd.print(" A");
    timer1 = millis();
    /*lcd.setCursor(0,1);
    lcd.print(proportional >> _Q);
    lcd.print(" ");
    lcd.print(integral >> _Q);
    lcd.print(" ");
    lcd.print(derivative >> _Q);
    lcd.print(" ");
    lcd.print(control >> _Q);
    timer1 = millis();*/
    
  }
  /*uint8_t dir = encoderRoll();
  if(dir != 0){
    int32_t tmp = disp_t; // приводим к целому числу
    tmp = updateNum(tmp, dir, 0x05);
    //Serial.println(tmp);
    if((tmp <= 400) && (tmp >= 185)){
      disp_t = tmp;
      setpoint = _IQ((float)tmp * kT);
    }
  }*/
}



ISR (TIMER1_CAPT_vect){ // Обрабдчик прерывания
  _iq current = _IQ(fileter_handler(&flt, analogRead(temp)));
  current = _IQmpy(current, deg_const);
  //fix_print(current);
    
  _iq error = setpoint - current;
  if(abs(error) > 100){
    proportional = _IQmpy(error, Kp[0]);
  }
  else{
    proportional = _IQmpy(error, Kp[1]);
  }
  //fix_print(proportional);
  
  //integral = integral + _IQmpy(((error + pre_error) / 2), Ki);
  //fix_print(integral);
  if(abs(error) < _IQ(50.0)){
    integral = integral + _IQmpy(((error + pre_error) / 2), Ki);
    if(abs(integral) > (limMax)) integral = limMax;
  }
  else{
    integral = 0;
  }
  //if(abs(integral) > (limMax / 2)) integral = limMax / 2;
    
    
  //fix_print(integral);
  derivative = _IQmpy((error - pre_error), Kd);
    
  control = proportional + integral + derivative;
    
  if(control > limMax) control = limMax;
  if(control < limMin) control = limMin;
  fileter_handler(&flt_u, control >> _Q);
  //fix_print(control);
  pre_error = error;
  OCR1A = uint16_t(control >> _Q);

}
