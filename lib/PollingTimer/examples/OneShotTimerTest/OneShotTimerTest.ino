#include <PollingTimer.h>

#define TIMEOUT     3000

OneShotTimer oneshot;

void setup()
{
    Serial.begin(9600);
}

void loop()
{
    oneshot.set(3000);
    while(!oneshot.elapsed()){
        Serial.print(".");
        delay(200);
    }
    Serial.println("");
    Serial.println("timeout!");
}
