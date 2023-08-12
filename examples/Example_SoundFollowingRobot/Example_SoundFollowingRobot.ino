//  variables for Microphones
const int arrayMax  = 100.0;
short aveL[arrayMax];
short aveR[arrayMax];
//short aveC[arrayMax];
int aveIndexNewest = 0;
int aveIndexOldest = 1;
float aveRi = 0;
float accLe = 0;
float accRi = 0;
float level = 0;
float levelAcc = 0;
float levelL = 0;
float levelR = 0;
float l = 0;
float r = 0;
float c = 0;
////   variables for drive
//const int LDE = 10;
const int LD1 = 8;
const int LD2 = 9;
//const int RDE = 11;
const int RD1 = 10;
const int RD2 = 11;
const int RPul = 3;
const int LPul = 2;
const int LEDW = 13;
const int LEDR = 12;

void setup() {
    //Serial.begin(38400);       // for debugging
    for( int i = 0 ; i < arrayMax ; i++ ){
         aveL[i] = 0;
         aveR[i] = 0;    
    }
   
   // drive pins
   // pinMode(LDE, OUTPUT);
    pinMode(LD1, OUTPUT);
    pinMode(LD2, OUTPUT);
   // pinMode(RDE, OUTPUT);
    pinMode(RD1, OUTPUT);
    pinMode(RD2, OUTPUT);
    pinMode(LEDW, OUTPUT);
    pinMode(LEDR, OUTPUT);
    //analogWrite(LDE , 255);
    digitalWrite(LD1 , LOW);
    digitalWrite(LD2 , LOW);
    //analogWrite(RDE , 255);
    digitalWrite(RD1 , LOW);
    digitalWrite(RD2 , LOW);    
}

void loop() {       
        aveL[aveIndexNewest] = analogRead(A0);
        accLe += aveL[aveIndexNewest];
        accLe -= aveL[aveIndexOldest];       
        
        aveR[aveIndexNewest] = analogRead(A1);
        accRi += aveR[aveIndexNewest];
        accRi -= aveR[aveIndexOldest];
                
        r =   (accRi/arrayMax) -  aveR[aveIndexNewest];
        r = abs(r);
        
        l =   (accLe/arrayMax) -  aveL[aveIndexNewest];
        l = abs(l);
      
        c += r;
        c -= (l * 1.1) ;  // Letf * 1.1 as my left hand microphone seems at little deaf // left is not as responsive as right microphone
        c /= 1.25;     // dived by 1.25 so C returns to 0 when it is quite : there is no sound input 
        //Serial.println(c);
    
        // increament Array Index
        aveIndexNewest++; 
        if (aveIndexNewest == arrayMax)  aveIndexNewest = 0; 
        aveIndexOldest++;        
        if (aveIndexOldest == arrayMax)  aveIndexOldest = 0; 
        ///////////////
        if (c > 7){// right            // C > 7 not 0 ? ideally C would = 0 // its a bias to keep in straight line
            digitalWrite(LD1 , LOW);
            digitalWrite(LD2 , HIGH);
            digitalWrite(RD1 , LOW);
            digitalWrite(RD2 , LOW);                   
        }else{  //  left
            digitalWrite(LD1 , LOW);
            digitalWrite(LD2 , LOW);
            digitalWrite(RD1 , LOW);
            digitalWrite(RD2 , HIGH);
        }
        
}
