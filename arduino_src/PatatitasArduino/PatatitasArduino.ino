/* Read the values from the encoders and the ultrasound sensors and send them to the srial port
*
* Author: Team Patatitas
* Date: May 2016
*/

#define ULTRASOUND_1_IN 4
#define ULTRASOUND_1_OUT 5
#define ULTRASOUND_2_IN 6
#define ULTRASOUND_2_OUT 7
#define ULTRASOUND_3_IN 8
#define ULTRASOUND_3_OUT 9
#define ULTRASOUND_4_IN 10
#define ULTRASOUND_4_OUT 11
#define ULTRASOUND_5_IN 12
#define ULTRASOUND_5_OUT 13

//Arduino UNO (328) and Nano (168)
#define INTERRUPT_REG PIND
#define INTERRUPT_0_BIT 2
#define INTERRUPT_1_BIT 3

volatile uint32_t left_encoder_count = 0;
volatile uint32_t right_encoder_count = 0;


unsigned long distancia;
unsigned long tiempo;
int i = 0;

void setup() {
    Serial.begin(115200);
    
    delay(100);
    pinMode(2,INPUT);
    pinMode(3,INPUT);
    attachInterrupt(0, &left_interrupt, CHANGE);
    attachInterrupt(1, &right_interrupt, CHANGE);
        
    pinMode(ULTRASOUND_1_OUT, OUTPUT);
    pinMode(ULTRASOUND_1_IN, INPUT);  
    pinMode(ULTRASOUND_2_OUT, OUTPUT);
    pinMode(ULTRASOUND_2_IN, INPUT);
    pinMode(ULTRASOUND_3_OUT, OUTPUT);
    pinMode(ULTRASOUND_3_IN, INPUT);
    pinMode(ULTRASOUND_4_OUT, OUTPUT);
    pinMode(ULTRASOUND_4_IN, INPUT);
    pinMode(ULTRASOUND_5_OUT, OUTPUT);
    pinMode(ULTRASOUND_5_IN, INPUT);
}

void left_interrupt(){
    // Just increment counter
    left_encoder_count ++;
}

void right_interrupt(){
    // Just increment counter
    right_encoder_count ++;
}

void loop() {
    
    Serial.print(left_encoder_count);
    Serial.print(",");
    Serial.print(right_encoder_count);
    Serial.print(",");
    
    if (i==0){
        digitalWrite(ULTRASOUND_1_OUT,LOW);
        delayMicroseconds(5);
        digitalWrite(ULTRASOUND_1_OUT,HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRASOUND_1_OUT,LOW);
        tiempo=pulseIn(ULTRASOUND_1_IN,HIGH);
        distancia=(0.017*tiempo);
        //Serial.print("DistanciaHastaEncoder1: ");
        Serial.print(distancia);
        //Serial.println(" cm");
        //delay(100);
        i=1;
    }
    
    if (i==1){
        digitalWrite(ULTRASOUND_2_OUT,LOW);
        delayMicroseconds(5);
        digitalWrite(ULTRASOUND_2_OUT,HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRASOUND_2_OUT,LOW);
        tiempo=pulseIn(ULTRASOUND_2_IN,HIGH);
        distancia=(0.017*tiempo);
        Serial.print(",");
        Serial.print(distancia);
        //Serial.println(" cm");
        //delay(100);
        i=2;
    }

    if (i==2){
        digitalWrite(ULTRASOUND_3_OUT,LOW);
        delayMicroseconds(5);
        digitalWrite(ULTRASOUND_3_OUT,HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRASOUND_3_OUT,LOW);
        tiempo=pulseIn(ULTRASOUND_3_IN,HIGH);
        distancia=(0.017*tiempo);
        Serial.print(",");
        Serial.print(distancia);
        //Serial.println(" cm");
        //delay(100);
        i=3;
    }

    if (i==3){
        digitalWrite(ULTRASOUND_4_OUT,LOW);
        delayMicroseconds(5);
        digitalWrite(ULTRASOUND_4_OUT,HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRASOUND_4_OUT,LOW);
        tiempo=pulseIn(ULTRASOUND_4_IN,HIGH);
        distancia=(0.017*tiempo);
        Serial.print(",");
        Serial.print(distancia);
        //Serial.println(" cm");
        //delay(100);
        i=3;
    }

    if (i==3){
        digitalWrite(ULTRASOUND_5_OUT,LOW);
        delayMicroseconds(5);
        digitalWrite(ULTRASOUND_5_OUT,HIGH);
        delayMicroseconds(10);
        digitalWrite(ULTRASOUND_5_OUT,LOW);
        tiempo=pulseIn(ULTRASOUND_5_IN,HIGH);
        distancia=(0.017*tiempo);
        Serial.print(",");
        Serial.println(distancia);
        //Serial.println(" cm");
        //delay(100);
        i=0;
    }
}
