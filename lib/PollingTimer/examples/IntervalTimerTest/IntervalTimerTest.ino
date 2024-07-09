#include <PollingTimer.h>

#define INTERVAL1   500
#define INTERVAL2   1000

IntervalTimer interval1;
IntervalTimer interval2;
ModuloCounter modulo(5);

void setup()
{
    Serial.begin(9600);
    
    interval1.set(INTERVAL1);
    interval2.set(INTERVAL2);
}

void loop()
{
    if(interval1.elapsed()){
        static int cnt = 0;
        cnt++;
        Serial.print("interval1: ");
        Serial.println(cnt);
    }
    delay(100);
    if(interval2.elapsed()){
        static int cnt = 0;
        cnt++;
        Serial.print("interval2: ");
        Serial.println(cnt);
        
        if(modulo.count()){
            static int cnt = 0;
            cnt++;
            Serial.print("modulo: ");
            Serial.println(cnt);
        }
    }
}
