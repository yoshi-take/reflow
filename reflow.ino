#include <SPI.h>
#include <Adafruit_MAX31855.h>
#include <LiquidCrystal.h>

/* 基本的に触らないパラメータ */  
#define   TEMP_PROTECT      260     // 保護温度[℃]
float     ACC_PREHEAT   =   1;      // 予備加熱加速度[℃/sec]
float     ACC_REFLOW;               // リフロー加熱加速度[℃/sec]
#define   MAX_DUTY          100     // DUTY比（0~100）
#define   CTRL_INT          20000   // 割り込みの閾値（制御）
#define   SENCE_INT         5000    // 割り込みの閾値（センス）

/* ここに必要なパラメータを記述 */
#define   TEMP_PREHEAT      50  // 予備加熱温度[℃]
#define   TIME_PREHEAT      60  // 予備加熱時間[sec] 
#define   TEMP_REFLOW       90  // リフローステージ温度[℃]
#define   TIME_REFLOW       10  // リフローステージ時間[sec]
#define   Kp                1.4   // Pゲイン
#define   Ki                0.05  // Iゲイン

/*=== ピン定義 ===*/
/* 熱電対アンプ */
#define MAXDO   3
#define MAXCS   4
#define MAXCLK  5
/* LCD */
#define RS      11
#define RW      10
#define ENABLE  8
#define DB4     15
#define DB5     14
#define DB6     17
#define DB7     16
/* リレー */
#define CONTROL 9
#define CTRL_VALUE  OCR1A
/* スイッチ */
#define   START   ~PD6   // D6
#define   RESET   ~PD7   // D7


/*=== モード一覧 ===*/
typedef enum{
  MODE_SET  = 0,
  MODE_ACC_PREHEAT,
  MODE_PREHEAT,
  MODE_ACC_REFLOW,
  MODE_REFLOW,
  MODE_SHUTDOWN,
  MODE_ANALOG
}en_MODE;


/*=== 変数 ===*/
float   f_Temp        = 25;   // 現在温度[℃]
float   f_Time        = 0;    // 経過時間[s]
float   f_TImeSum     = 0;    // 合計経過時間[s]
float   f_TempErr     = 0;    // 温度偏差
float   f_TempErrSum  = 0;    // 温度偏差の積分値
float   f_TrgtTemp    = 0;    // 目標温度[℃]
en_MODE state         = MODE_SET;   // システムの状態
boolean flag_ctrl     = false;      // 制御許可
int     count_ctrl    = 0;
int     count_sence   = 0;

Adafruit_MAX31855 thermocouple(MAXCLK, MAXCS, MAXDO);
LiquidCrystal lcd = LiquidCrystal(RS,RW,ENABLE,DB4,DB5,DB6,DB7);


void LCD_init(){
  lcd.begin(16,2);
  lcd.print("initialize");
}

void LCD_TempView(double d_temp){
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print("en_MODE");  // モード表示
  lcd.setCursor(0,1);
  lcd.print("TMP=");
  lcd.print(d_temp); 
}

void LCD_View(double d_temp){
  lcd.clear();
  lcd.setCursor(0,0);
  
  switch(state){
    case  MODE_SET:
      lcd.print("SET");
    case  MODE_ACC_PREHEAT:
      lcd.print("ACC_PREHEAT");
    case  MODE_PREHEAT:
      lcd.print("PREHEAT");
    case  MODE_ACC_REFLOW:
      lcd.print("ACC_REFLOW");
    case  MODE_REFLOW:
      lcd.print("REFLOW");
    case  MODE_SHUTDOWN:
      lcd.print("SHUTDOWN");
    case  MODE_ANALOG:
      lcd.print("ANALOG");
  }
  
  lcd.setCursor(0,1);
  lcd.print("TMP=");
  lcd.print(d_temp); 
}

void Serial_init(){
  Serial.begin(9600);
  while (!Serial) delay(1); // wait for Serial on Leonardo/Zero, etc
  Serial.println("MAX31855 test");
  // wait for MAX chip to stabilize
  delay(500);
  Serial.print("Initializing sensor...");
  if (!thermocouple.begin()) {
    Serial.println("ERROR.");
    while (1) delay(10);
  }
}

void Serial_View(){
  Serial.print("Internal Temp = ");
  Serial.println(f_Temp);
  Serial.print("Target Temp = ");
  Serial.println(f_TrgtTemp);

}

void Control_init(){
  /*Timer1を使用*/
  TCCR1A  = 0b10000010;
  TCCR1B  = 0b00011101;
  ICR1    = 31249;      // 0.5Hzに設定
  CTRL_VALUE    = 0;          // duty
  pinMode(CONTROL,OUTPUT);
  Control_STOP();
}

void Timer_init(){
  /* Timer0を使用(センス用)*/
  // 0.1ミリ秒ごとに割り込みが入るように設定
  TCCR0A  = 0b10000010;
  TCCR0B  = 0b00000010;   // 8分周
  TIMSK0  = 0b00000010;   // 割り込みを設定
  OCR0A   = 199;   

  /* Timer2を使用(制御用)*/
  // 0.1ミリ秒ごとに割り込みが入るように設定
  TCCR2A  = 0b10000010;
  TCCR2B  = 0b00000010;
  TIMSK2  = 0b00000010;
  OCR2A   = 199;
}

ISR(TIMER0_COMPA_vect){
  if(count_sence == SENCE_INT){
    interrupt();    // 0.5秒ごとに割り込みが入る
    count_sence = 0;
  }else{
    count_sence++;
  }
}


ISR(TIMER2_COMPA_vect){
  if(count_ctrl == CTRL_INT){
    CTRL_Pol();   // 2秒ごとに割込みが入る
    count_ctrl  = 0;
  }else{
    count_ctrl++;
  }
}

void Control_STOP(){
  CTRL_VALUE  = 0;
  flag_ctrl   = false;
}

void Control_START(){
  flag_ctrl = true;
}

void System_Protect(){
  if( (RESET==1) || (f_Temp>=TEMP_PROTECT) ){
    Control_STOP();
    state = MODE_SHUTDOWN;
  }
}

void IO_init(){
  pinMode(START,INPUT_PULLUP);
  pinMode(RESET,INPUT_PULLUP);
}

void CTRL_getTemp(){
  f_Temp  = thermocouple.readCelsius();
}

void CTRL_refTarget(){
  
  switch(state){
    case  MODE_SET:
      f_TrgtTemp  = f_Temp;
      
    case  MODE_ACC_PREHEAT:
      if(f_TrgtTemp < TEMP_PREHEAT){
        f_TrgtTemp  = f_Temp + ACC_PREHEAT*f_Time;
      }
      break;
      
    case  MODE_PREHEAT:
      f_TrgtTemp  = TEMP_PREHEAT;
    
    case  MODE_ACC_REFLOW:
      if(f_TrgtTemp < TEMP_REFLOW){
        f_TrgtTemp  = f_Temp + ACC_REFLOW*f_Time;
      }
    case  MODE_REFLOW:
      f_TrgtTemp  = TEMP_REFLOW;

    case  MODE_SHUTDOWN:
      f_TrgtTemp  = 25;
  }
}

void CTRL_getTempFB(float *p_err){
  
  f_TempErr    = f_TrgtTemp - f_Temp;
  f_TempErrSum += f_TempErr * Ki;

  // アンチワインドアップ入れる

  *p_err       = f_TempErrSum + f_TempErr * Kp;
}

void CTRL_outPWM(float f_duty){
  if(f_duty > MAX_DUTY){
    f_duty  = MAX_DUTY;
  
  }else if(f_duty < 0){
    f_duty = 0;
  }

  // PWM出力
  CTRL_VALUE  = f_duty * 312;
}

void CTRL_Pol(){
  
  float f_TempCtrl  = 0;
  
  if( (RESET != 1) || (state != MODE_SET) || (state != MODE_SHUTDOWN) ){

    /* 制御を行うかチェック */
    if(flag_ctrl != true){
      return;
    }

    CTRL_refTarget();               // 目標値設定
    CTRL_getTempFB(&f_TempCtrl);    // 制御量取得
    CTRL_outPWM(f_TempCtrl);        // PWM出力
    
  }else{
    Control_STOP();   // 出力を切る
  }

  f_Time += 2;
    
}

//割り込みでの処理
void interrupt(){
  CTRL_getTemp();   // 温度取得
  System_Protect(); // 保護判定
  LCD_View(f_Temp); // LCD表示
  Serial_View();    // シリアル伝送
}

void setup(){
  IO_init();        // IO周りの設定
  LCD_init();       // LCDの設定
  Serial_init();    // シリアルの設定
  Timer_init();     // 割り込みの設定
  Control_init();   // PWMの設定
  Serial.println("DONE.");
  lcd.print("DONE");
}

void loop() {

  /* 予備加熱温度まで加熱中 */
  if( (START==1) && (state == MODE_SET) && (RESET != 1) ){
    if(f_Temp > 50){
      state = MODE_SET;
    }else{
      state = MODE_ACC_PREHEAT; // 次のモードに遷移
      Control_START();          // 制御許可
    }
  }

  if(state == MODE_ACC_PREHEAT){
    f_Time  = 0;
    while(f_Temp < TEMP_PREHEAT);
    state   = MODE_PREHEAT;
  }else{
  }

  /* 予備加熱温度でキープ */
  f_Time  = 0;
  while(f_Time == TIME_PREHEAT);
  state   = MODE_ACC_REFLOW;

  /* リフローステージ温度まで加熱中 */
  f_Time  = 0;
  while(f_Temp < TEMP_REFLOW);
  state   = MODE_REFLOW;

  /* リフローステージ温度でキープ */
  f_Time  = 0;
  while(f_Time == TIME_PREHEAT);
  state   = MODE_SHUTDOWN;

}

/*
void loop() {
  // basic readout test, just print the current temp
   //Serial.print("Internal Temp = ");
   //Serial.println(thermocouple.readInternal());

   double c = thermocouple.readCelsius();
   if (isnan(c)) {
     Serial.println("Something wrong with thermocouple!");
   } else {
     Serial.print("C = ");
     Serial.println(c);
     LCD_TempView(c);
   }
   //Serial.print("F = ");
   //Serial.println(thermocouple.readFahrenheit());

   delay(1000);
  
}
*/
