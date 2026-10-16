#include <Arduino.h>
#define M1_EN 5
#define M1_IN1 7
#define M1_IN2 8
#define M2_EN 6
#define M2_IN1 9
#define M2_IN2 10
#define BTN_LEFT 2
#define BTN_RIGHT 3
#define BTN_UP 4
#define BTN_DOWN 11
#define BTN_RECORD 12
#define BTN_PLAY 13
#define M1_SDA A2
#define M1_SCL A3
#define M2_SDA A4
#define M2_SCL A5
#define AS5600_ADDR 0x36
#define M1_SPEED 75
#define M2_SPEED 75
#define PLAY_MIN_SPEED 45
#define PLAY_MAX_SPEED 75
#define HOME_SPEED 75
#define POSITION_TOLERANCE 2.0
#define MAX_POSITIONS 20
#define LONG_PRESS_TIME 1500
#define BUTTON_DEBOUNCE 50
#define M1_RAW_0 1661
#define M1_RAW_90 3177
#define M1_RAW_180 120
#define M2_RAW_0 531
#define M2_RAW_90 3700
#define M2_RAW_180 2613
float recordedM1[MAX_POSITIONS];
float recordedM2[MAX_POSITIONS];
int recordedCount=0;
bool playing=false;
bool returningHome=false;
int homeStage=0;
int playIndex=0;
unsigned long recordPressTime=0;
bool recordPressed=false;
bool playLastReading=HIGH;
bool playStableState=HIGH;
unsigned long playLastChange=0;
void motor1Stop(){
analogWrite(M1_EN,0);
digitalWrite(M1_IN1,LOW);
digitalWrite(M1_IN2,LOW);
}
void motor1Forward(int speedValue=M1_SPEED){
digitalWrite(M1_IN1,HIGH);
digitalWrite(M1_IN2,LOW);
analogWrite(M1_EN,speedValue);
}
void motor1Reverse(int speedValue=M1_SPEED){
digitalWrite(M1_IN1,LOW);
digitalWrite(M1_IN2,HIGH);
analogWrite(M1_EN,speedValue);
}
void motor2Stop(){
analogWrite(M2_EN,0);
digitalWrite(M2_IN1,LOW);
digitalWrite(M2_IN2,LOW);
}
void motor2Forward(int speedValue=M2_SPEED){
digitalWrite(M2_IN1,HIGH);
digitalWrite(M2_IN2,LOW);
analogWrite(M2_EN,speedValue);
}
void motor2Reverse(int speedValue=M2_SPEED){
digitalWrite(M2_IN1,LOW);
digitalWrite(M2_IN2,HIGH);
analogWrite(M2_EN,speedValue);
}
void i2cDelay(){
delayMicroseconds(5);
}
void i2cStart(uint8_t sda,uint8_t scl){
pinMode(sda,OUTPUT);
pinMode(scl,OUTPUT);
digitalWrite(sda,HIGH);
digitalWrite(scl,HIGH);
i2cDelay();
digitalWrite(sda,LOW);
i2cDelay();
digitalWrite(scl,LOW);
}
void i2cStop(uint8_t sda,uint8_t scl){
pinMode(sda,OUTPUT);
pinMode(scl,OUTPUT);
digitalWrite(sda,LOW);
digitalWrite(scl,HIGH);
i2cDelay();
digitalWrite(sda,HIGH);
i2cDelay();
}
bool i2cWriteByte(uint8_t sda,uint8_t scl,uint8_t data){
pinMode(sda,OUTPUT);
for(int i=7;i>=0;i--){
digitalWrite(sda,(data>>i)&1);
digitalWrite(scl,HIGH);
i2cDelay();
digitalWrite(scl,LOW);
}
pinMode(sda,INPUT_PULLUP);
digitalWrite(scl,HIGH);
i2cDelay();
bool ack=!digitalRead(sda);
digitalWrite(scl,LOW);
return ack;
}
uint8_t i2cReadByte(uint8_t sda,uint8_t scl,bool ack){
uint8_t data=0;
pinMode(sda,INPUT_PULLUP);
for(int i=7;i>=0;i--){
digitalWrite(scl,HIGH);
i2cDelay();
if(digitalRead(sda)) data|=(1<<i);
digitalWrite(scl,LOW);
}
pinMode(sda,OUTPUT);
digitalWrite(sda,ack?LOW:HIGH);
digitalWrite(scl,HIGH);
i2cDelay();
digitalWrite(scl,LOW);
digitalWrite(sda,HIGH);
return data;
}
int readAS5600Raw(uint8_t sda,uint8_t scl){
i2cStart(sda,scl);
if(!i2cWriteByte(sda,scl,(AS5600_ADDR<<1)|0)){
i2cStop(sda,scl);
return -1;
}
if(!i2cWriteByte(sda,scl,0x0C)){
i2cStop(sda,scl);
return -1;
}
i2cStart(sda,scl);
if(!i2cWriteByte(sda,scl,(AS5600_ADDR<<1)|1)){
i2cStop(sda,scl);
return -1;
}
uint8_t high=i2cReadByte(sda,scl,true);
uint8_t low=i2cReadByte(sda,scl,false);
i2cStop(sda,scl);
return ((high&0x0F)<<8)|low;
}
float calculateM1Angle(int raw){
if(raw<0) return 0;
float continuousRaw=raw;
if(raw<M1_RAW_0) continuousRaw=raw+4096.0;
float raw0=M1_RAW_0;
float raw90=M1_RAW_90;
if(raw90<raw0) raw90+=4096.0;
float raw180=M1_RAW_180;
if(raw180<raw0) raw180+=4096.0;
float angle;
if(continuousRaw<=raw90){
angle=(continuousRaw-raw0)*90.0/(raw90-raw0);
}else{
angle=90.0+(continuousRaw-raw90)*90.0/(raw180-raw90);
}
if(angle<0) angle=0;
if(angle>180) angle=180;
return angle;
}
float wrappedDistance(int startRaw,int raw){
float distance=(float)startRaw-(float)raw;
if(distance<0) distance+=4096.0;
return distance;
}
float calculateM2Angle(int raw){
if(raw<0) return 0;
float distance=wrappedDistance(M2_RAW_0,raw);
float d90=wrappedDistance(M2_RAW_0,M2_RAW_90);
float d180=wrappedDistance(M2_RAW_0,M2_RAW_180);
float angle;
if(distance<=d90){
angle=distance*90.0/d90;
}else{
angle=90.0+(distance-d90)*90.0/(d180-d90);
}
if(angle<0) angle=0;
if(angle>180) angle=180;
return angle;
}
int calculatePlaybackSpeed(float error){
error=fabs(error);
if(error<=5.0) return PLAY_MIN_SPEED;
if(error>=30.0) return PLAY_MAX_SPEED;
return PLAY_MIN_SPEED+(int)((error-5.0)*(PLAY_MAX_SPEED-PLAY_MIN_SPEED)/25.0);
}
void manualControl(){
bool left=digitalRead(BTN_LEFT)==LOW;
bool right=digitalRead(BTN_RIGHT)==LOW;
bool up=digitalRead(BTN_UP)==LOW;
bool down=digitalRead(BTN_DOWN)==LOW;
if(left&&!right){
motor1Reverse(M1_SPEED);
}else if(right&&!left){
motor1Forward(M1_SPEED);
}else{
motor1Stop();
}
if(up&&!down){
motor2Reverse(M2_SPEED);
}else if(down&&!up){
motor2Forward(M2_SPEED);
}else{
motor2Stop();
}
}
void handleRecordButton(float angle1,float angle2){
bool state=digitalRead(BTN_RECORD)==LOW;
if(state&&!recordPressed){
recordPressed=true;
recordPressTime=millis();
}
if(!state&&recordPressed){
unsigned long pressDuration=millis()-recordPressTime;
recordPressed=false;
if(pressDuration>=LONG_PRESS_TIME){
playing=false;
returningHome=false;
homeStage=0;
playIndex=0;
recordedCount=0;
motor1Stop();
motor2Stop();
Serial.println("RECORD RESET");
Serial.println("POSITIONS=0");
}else{
if(!playing&&!returningHome){
if(recordedCount<MAX_POSITIONS){
recordedM1[recordedCount]=angle1;
recordedM2[recordedCount]=angle2;
recordedCount++;
Serial.print("POSITION ");
Serial.print(recordedCount);
Serial.println(" SAVED");
Serial.print("M1=");
Serial.print(angle1,1);
Serial.print(" M2=");
Serial.println(angle2,1);
Serial.print("TOTAL POSITIONS=");
Serial.println(recordedCount);
}else{
Serial.println("MEMORY FULL");
}
}else{
Serial.println("CANNOT RECORD WHILE PLAYING");
}
}
}
}
void handlePlayButton(){
bool reading=digitalRead(BTN_PLAY);
if(reading!=playLastReading){
playLastChange=millis();
playLastReading=reading;
}
if(millis()-playLastChange>BUTTON_DEBOUNCE){
if(reading!=playStableState){
playStableState=reading;
if(playStableState==LOW){
Serial.println("PLAY BUTTON PRESSED");
Serial.print("RECORDED POSITIONS=");
Serial.println(recordedCount);
if(!playing&&!returningHome){
if(recordedCount>0){
playing=true;
returningHome=true;
homeStage=1;
playIndex=0;
motor1Stop();
motor2Stop();
Serial.println("RETURNING MOTOR 1 TO 0 DEG");
}else{
Serial.println("NO POSITIONS RECORDED");
}
}
}
}
}
}
void goToHome(float angle1,float angle2){
if(homeStage==1){
motor2Stop();
if(fabs(angle1)<=POSITION_TOLERANCE){
motor1Stop();
homeStage=2;
Serial.println("MOTOR 1 AT 0 DEG");
Serial.println("RETURNING MOTOR 2 TO 0 DEG");
delay(200);
return;
}
if(angle1>0){
motor1Reverse(HOME_SPEED);
}else{
motor1Stop();
homeStage=2;
Serial.println("MOTOR 1 AT 0 DEG");
Serial.println("RETURNING MOTOR 2 TO 0 DEG");
}
return;
}
if(homeStage==2){
motor1Stop();
if(fabs(angle2)<=POSITION_TOLERANCE){
motor2Stop();
returningHome=false;
homeStage=0;
playIndex=0;
Serial.println("MOTOR 2 AT 0 DEG");
Serial.println("0 DEG POSITION REACHED");
Serial.println("STARTING PLAYBACK");
delay(500);
return;
}
if(angle2>0){
motor2Forward(HOME_SPEED);
}else{
motor2Stop();
returningHome=false;
homeStage=0;
playIndex=0;
Serial.println("MOTOR 2 AT 0 DEG");
Serial.println("0 DEG POSITION REACHED");
Serial.println("STARTING PLAYBACK");
delay(500);
}
}
}
void playback(float angle1,float angle2){
if(returningHome){
goToHome(angle1,angle2);
return;
}
if(playIndex>=recordedCount){
motor1Stop();
motor2Stop();
playing=false;
Serial.println("PLAYBACK FINISHED");
return;
}
float target1=recordedM1[playIndex];
float target2=recordedM2[playIndex];
float error1=target1-angle1;
float error2=target2-angle2;
bool reached1=fabs(error1)<=POSITION_TOLERANCE;
bool reached2=fabs(error2)<=POSITION_TOLERANCE;
if(reached1){
motor1Stop();
}else{
int speed1=calculatePlaybackSpeed(error1);
if(error1>0){
motor1Forward(speed1);
}else{
motor1Reverse(speed1);
}
}
if(reached2){
motor2Stop();
}else{
int speed2=calculatePlaybackSpeed(error2);
if(error2>0){
motor2Reverse(speed2);
}else{
motor2Forward(speed2);
}
}
if(reached1&&reached2){
motor1Stop();
motor2Stop();
Serial.print("POSITION ");
Serial.print(playIndex+1);
Serial.println(" REACHED");
delay(300);
playIndex++;
if(playIndex<recordedCount){
Serial.print("MOVING TO POSITION ");
Serial.println(playIndex+1);
}
}
}
void setup(){
Serial.begin(9600);
pinMode(M1_EN,OUTPUT);
pinMode(M1_IN1,OUTPUT);
pinMode(M1_IN2,OUTPUT);
pinMode(M2_EN,OUTPUT);
pinMode(M2_IN1,OUTPUT);
pinMode(M2_IN2,OUTPUT);
pinMode(BTN_LEFT,INPUT_PULLUP);
pinMode(BTN_RIGHT,INPUT_PULLUP);
pinMode(BTN_UP,INPUT_PULLUP);
pinMode(BTN_DOWN,INPUT_PULLUP);
pinMode(BTN_RECORD,INPUT_PULLUP);
pinMode(BTN_PLAY,INPUT_PULLUP);
pinMode(M1_SDA,INPUT_PULLUP);
pinMode(M1_SCL,INPUT_PULLUP);
pinMode(M2_SDA,INPUT_PULLUP);
pinMode(M2_SCL,INPUT_PULLUP);
motor1Stop();
motor2Stop();
Serial.println("2-DOF ARM READY");
Serial.println("D12=RECORD/RESET");
Serial.println("D13=PLAY");
Serial.println("SHORT D12=SAVE");
Serial.println("LONG D12=RESET");
}
void loop(){
int raw1=readAS5600Raw(M1_SDA,M1_SCL);
int raw2=readAS5600Raw(M2_SDA,M2_SCL);
float angle1=calculateM1Angle(raw1);
float angle2=calculateM2Angle(raw2);
handleRecordButton(angle1,angle2);
handlePlayButton();
if(playing){
playback(angle1,angle2);
}else{
manualControl();
}
static unsigned long lastPrint=0;
if(millis()-lastPrint>=500){
lastPrint=millis();
Serial.print("M1=");
Serial.print(angle1,1);
Serial.print(" M2=");
Serial.print(angle2,1);
Serial.print(" POSITIONS=");
Serial.print(recordedCount);
Serial.print(" HOME=");
Serial.print(returningHome?"ON":"OFF");
Serial.print(" STAGE=");
Serial.print(homeStage);
Serial.print(" PLAY=");
Serial.println(playing?"ON":"OFF");
}
delay(10);
}
