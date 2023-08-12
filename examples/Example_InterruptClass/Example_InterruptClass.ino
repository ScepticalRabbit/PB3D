#define MAX_ISR 8
 
class cCounter
{
public:
  cCounter(uint8_t pinInt) : pinInt(pinInt) {}
 
  ~cCounter(void)
  {
    detachInterrupt(digitalPinToInterrupt(pinInt));
    ISRUsed &= ~_BV(myISRId);   // free up the ISR slot for someone else
  }
 
  bool begin(void)
  {
    int8_t irq = digitalPinToInterrupt(pinInt);
   
    if (irq != NOT_AN_INTERRUPT)
    {
      pinMode(pinInt, INPUT);
   
      // assign ourselves a ISR ID ...
      myISRId = UINT8_MAX;
      for (uint8_t i = 0; i < MAX_ISR; i++)
      {
        if (!(ISRUsed & _BV(i)))    // found a free ISR Id?
        {
          myISRId = i;                 // remember who this instance is
          myInstance[myISRId] = this; // record this instance
          ISRUsed |= _BV(myISRId);    // lock this in the allocations table
          break;
        }
      }
      // ... and attach corresponding ISR callback from the lookup table
      {
        static void((*ISRfunc[MAX_ISR])(void)) =
        {
          globalISR0, globalISR1, globalISR2, globalISR3,
          globalISR4, globalISR5, globalISR6, globalISR7,
        };
   
        if (myISRId != UINT8_MAX)
          attachInterrupt(irq, ISRfunc[myISRId], CHANGE);
        else
          irq = NOT_AN_INTERRUPT;
      }
      reset();
    }
    return(irq != NOT_AN_INTERRUPT);
  }
 
  inline void reset(void) { count = 0; }
 
private:
  // Define the class variables
  uint8_t pinInt;            // The interrupt pin used
  uint8_t myISRId;           // This is my instance ISR Id for myInstance[x] and encoderISRx
  volatile uint16_t count; // Encoder interrupt counter
 
  static uint8_t ISRUsed;        // Keep track of which ISRs are used (global bit field)
  static cCounter* myInstance[]; // Callback instance for the ISR to reach instanceISR()
 
  void instanceISR(void) { count++; }   // Instance ISR handler called from static ISR globalISRx
 
  // declare all the [MAX_ISR] encoder ISRs
  static void globalISR0(void);
  static void globalISR1(void);
  static void globalISR2(void);
  static void globalISR3(void);
  static void globalISR4(void);
  static void globalISR5(void);
  static void globalISR6(void);
  static void globalISR7(void);
};
 
// Interrupt handling declarations required outside the class
uint8_t cCounter::ISRUsed = 0;           // allocation table for the globalISRx()
cCounter* cCounter::myInstance[MAX_ISR]; // callback instance handle for the ISR
 
// ISR for each myISRId
void cCounter::globalISR0(void) { cCounter::myInstance[0]->instanceISR(); }
void cCounter::globalISR1(void) { cCounter::myInstance[1]->instanceISR(); }
void cCounter::globalISR2(void) { cCounter::myInstance[2]->instanceISR(); }
void cCounter::globalISR3(void) { cCounter::myInstance[3]->instanceISR(); }
void cCounter::globalISR4(void) { cCounter::myInstance[4]->instanceISR(); }
void cCounter::globalISR5(void) { cCounter::myInstance[5]->instanceISR(); }
void cCounter::globalISR6(void) { cCounter::myInstance[6]->instanceISR(); }
void cCounter::globalISR7(void) { cCounter::myInstance[7]->instanceISR(); }
 
cCounter c1(2); 
cCounter c2(3);
 
void setup(void)
{
  Serial.begin(57600);
   
  if (!c1.begin()) Serial.println("Can't start c1");
  if (!c2.begin()) Serial.println("Can't start c2");
}
 
void loop(void)
{
  // do whatever
}
