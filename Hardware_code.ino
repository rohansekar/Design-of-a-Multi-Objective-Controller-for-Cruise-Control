#include <LiquidCrystal.h>
#include<Servo.h>
#define massSensor        A0
#define cruiseSpeedDial     A2
#define cruiseDistanceDial    A3
Servo myservo1,myservo2;
LiquidCrystal lcd(12, 5, 4, 2, 1, 0);

const int echoPin=13;      
const int trigPin=6;      
const int servo1=9;       
const int servo2=10;      
const int start=8;        
const int engineRPM=11;     
const int speedSensor=3;    
int distance=0;              
int distanceSet=0;      
int speed=0;          
int speedSet=0;       
int tempSpeedSet=0;   
int mass=0;           
int servoValue=0;     
long duration=0;      
double error=0;       
double lastError=0;    
double cumError=0;    
double rateError=0;   
double outputPWM=0;   
double kp=27.7593;         
double ki=3.9352;          
double kd=24.0508;        
double force=0;        
double friction=0;        
double drag=0;          
int brakeForce=0;     
double acceleration=0;      
void defaultPos(void);      
void startACC(void);      
void getDistance(void);     
void getSpeed(void);      
void checkRadar(void);
void getMass(void);
void getSimulatedSpeed(void);
void PID(void);
void simulateCar(void);
void cutSpeed(void);
void brake(void);
void setup()
{
 Serial.begin(9600);
  lcd.begin(16, 2);
  pinMode(cruiseSpeedDial,INPUT);
  pinMode(cruiseDistanceDial,INPUT);
  pinMode(speedSensor,INPUT);
  pinMode(start,INPUT);
  pinMode(engineRPM,OUTPUT);
  myservo1.attach(9);
  myservo2.attach(10);
  myservo1.write(90);
  delay(15);
  myservo2.write(90);
  delay(15);
  lcd.write("Adaptive Cruise");
  lcd.setCursor(0,1);
  lcd.write(" Control System");
  delay(1500);
  myservo1.write(0);
  delay(15);
  myservo2.write(0);
  delay(15);
  delay(1500);
  lcd.clear();
}

void defaultPos(void)
{
    lcd.setCursor(7,1);
    lcd.write("OFF ");
    lcd.setCursor(11,0);
    lcd.print(speed);
    lcd.write("Km ");
    getSpeed();
    getDistance();
    checkRadar();
    getMass();
}

void getDistance(void)
{
  distanceSet = (analogRead(cruiseDistanceDial)*0.1953125)+100;
    lcd.setCursor(0,1);
    lcd.print(distanceSet);
    lcd.print("m ");
}
 
void getSpeed(void)
{
  speedSet = (analogRead(cruiseSpeedDial)*0.1171875)+30;  
    lcd.setCursor(0,0);
    lcd.print(speedSet);
    lcd.print("Km ");
}

void getMass(void)
{
  mass = (analogRead(massSensor)*0.48828125)+1500;  
    lcd.setCursor(6,0);
    lcd.print(mass);
}

void cutSpeed(void)
{
  tempSpeedSet=speedSet-((300-distance)*0.003*speedSet);  
    if(speedSet!=tempSpeedSet)
      speedSet=tempSpeedSet;
}

void brake(void)
{
  brakeForce=(mass*((speed*speed)+1))/(2*distance);
    outputPWM=0;
}

void PID(void)
{
  error=speedSet-speed;
    cumError+=error * 0.8;    //0.8 is the simulation time
    rateError=(error - lastError)/0.8;
    outputPWM=(kp*error) + (ki*cumError) + (kd*rateError);
    lastError=error;
    analogWrite(engineRPM,outputPWM);
}

void simulateCar(void)
{
    friction=0.7*mass*9.8;        
    drag=(0.3*1.225*(speed*speed)*4)/2;
  force=(outputPWM*78.125)-brakeForce-friction-drag;
    acceleration=force/mass;
    speed+=acceleration*0.8;
    if(speed<0)
      speed=0;
    lcd.setCursor(11,0);
    lcd.print(speed);
    lcd.print("Km ");
    servoValue=(outputPWM*0.703125);
    myservo1.write(servoValue);
    delay(15);
    servoValue=brakeForce;
    myservo2.write(servoValue);
}

void checkRadar(void)
{
  pinMode(trigPin, OUTPUT);
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);

    // convert the time into a distance
    distance = duration/29/2;

    lcd.setCursor(11,1);
    lcd.print(distance);
    lcd.print("m ");
    
}

void startACC(void)
{
    getSpeed();
    getDistance();
    getMass();
    lcd.setCursor(7,1);
    lcd.print("ON ");
    checkRadar();
  
    if(distance>=distanceSet)
    {
      brakeForce=0;
      PID();
    simulateCar();
    }
    else if((distance<distanceSet)&&(distance>=30))
    {
      brakeForce=0;
      cutSpeed();
      PID();
    simulateCar();
    }
    else
    {
      brake();
      brakeForce+=30;
      simulateCar();
    }

}

void loop()
{
  if(digitalRead(start)==HIGH) 
      startACC();
    else
        defaultPos();
}
