/*
    arduino firmware source code
    detect the distance between ultrasonic sensor and obstacle
*/

// declare trig/echo port number
int trig[4] = {2, 4, 6, 8};
int echo[4] = {3, 5, 7, 9};
int i=0;

void setup() {
    // serial communication, 9600 baudrate
    Serial.begin(9600);
    for (i=0; i<4; i++) {
        pinMode(trig[i],OUTPUT);
        pinMode(echo[i],INPUT);
    }
}

void loop() {
    long dur[4]={0.0,};
    long dist[4]={0.0,};

    for(i=0; i<4; i++) {
        digitalWrite(trig[i],LOW);
        delayMicroseconds(2);
        digitalWrite(trig[i],HIGH);
        delayMicroseconds(10);
        degitalWrite(trig[i],LOW);

        dur[i]=pulseIn(echo[i],HIGH);
        // convert pulsetime to distance
        dist[i]=dur[i]*170/1000;
        if(dist[i]>=2000 || dist[i]<0) {
            dist[i]=0;
        }
    }

    // print the data of distance
    Serial.print(dist[0]);
    Serial.print("mm ");
    Serial.print(dist[1]);
    Serial.print("mm ");
    Serial.print(dist[2]);
    Serial.print("mm ");
    Serial.print(dist[3]);
    Serial.println("mm");

    delay(50);
}