
//#define DISPLAY_OOTC_RAW_TIME
//#define DISPLAY_OOTC
//#define DISPLAY_OOTC_DATA
#define DISPLAY_ANGLES
//#define DISPLAY_RAW_ANGLE
//#define DISPLAY_SIGNAL_TIMINGS
//#define DISPLAY_RAW_SIGNAL_DURATIONS
//#define DISPLAY_NEWLINE_AFTER_Y

#define RINGBUFF_MAX 200
#define TICKS_PER_US 84 //84 ticks per microsecond

// the high order bit is used to indicate if the entry is a rising or falling edge.
#define RISING_EDGE 0x00000000
#define FALLING_EDGE 0x80000000
//#define MILLIS_MULTIPLIER 10000
//#define MAX_COUNTER (MILLIS_MULTIPLIER * 84000)
#define MILLIS_MULTIPLIER 1
#define MAX_COUNTER (MILLIS_MULTIPLIER * 8400000)

class IrReceiver
{
  public:
  short readerPos; // last place read from
  short writerPos; // next place to write
  // note: if readerPos == writerPos, that is the state we hit when
  // the ring buffer is full.  We do this so that the check for a full
  // ring buffer is as simple as possible in the interrupt routine
  // i.e. all it has to do is a quick comparison to see if it's safe to write.
  unsigned int buff[RINGBUFF_MAX];
  
  bool data;
  bool rotor;
  bool skip;

  bool lastPulseWasOotc;
  bool lastWasRising;
  int OotcPulseStartTime;

  float X,Y;

  IrReceiver()
  {
    readerPos=0;
    writerPos=1;
    X=0;
    Y=0;
    data=false;
    rotor=false;
    skip=false;
    lastPulseWasOotc = false;
    lastWasRising = false;
    OotcPulseStartTime=0;
    
  }  
  
};

#define MAX_RING_BUFFERS 10
volatile IrReceiver gReceiver[MAX_RING_BUFFERS];

long int down=0, up=0;
// the setup function runs once when you press reset or power the board
void setup() 
{
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  //pinMode(3, INPUT_PULLUP);
  pinMode(20, INPUT);
  pinMode(21, INPUT);
  pinMode(22, INPUT);
  pinMode(23, INPUT);
  pinMode(24, INPUT);
  pinMode(25, INPUT);
  pinMode(26, INPUT);
  pinMode(27, INPUT);
  pinMode(28, INPUT);
  pinMode(29, INPUT);
  pinMode(30, INPUT);
  pinMode(31, INPUT);
  Serial.begin(115200);

  Serial.print("SysTick->LOAD: ");
  Serial.println(SysTick->LOAD);

  SysTick->LOAD = 8399999;

  Serial.print("SysTick->LOAD: ");
  Serial.println(SysTick->LOAD);

  // flip the FALLING and RISING here because the input signal is active low
  attachInterrupt(digitalPinToInterrupt(22), rising0, FALLING);
  attachInterrupt(digitalPinToInterrupt(23), falling0, RISING);
  attachInterrupt(digitalPinToInterrupt(24), rising1, FALLING);
  attachInterrupt(digitalPinToInterrupt(25), falling1, RISING);
  attachInterrupt(digitalPinToInterrupt(26), rising2, FALLING);
  attachInterrupt(digitalPinToInterrupt(27), falling2, RISING);
  attachInterrupt(digitalPinToInterrupt(28), rising3, FALLING);
  attachInterrupt(digitalPinToInterrupt(29), falling3, RISING);
  attachInterrupt(digitalPinToInterrupt(30), rising4, FALLING);
  attachInterrupt(digitalPinToInterrupt(31), falling4, RISING);
  attachInterrupt(digitalPinToInterrupt(32), rising5, FALLING);
  attachInterrupt(digitalPinToInterrupt(33), falling5, RISING);
}



// the loop function runs over and over again forever
void loop() 
{

  delay (1);  


  ProcessSensor(0);
  ProcessSensor(1);
  ProcessSensor(2);
  ProcessSensor(3);
  ProcessSensor(4);
  ProcessSensor(5);

}

void ProcessSensor(int sensor)
{
           

  int lastTime = 0;
  // yeah, this is a hairy conditional.  
  // We're checking to see if there's anything in the ring buffer for us to process.
  // We've traded off some complexity here to make the interrupt routines simpler/ faster
  while (((gReceiver[sensor].writerPos - 1 - gReceiver[sensor].readerPos) + RINGBUFF_MAX) % RINGBUFF_MAX != 0)
  {
    gReceiver[sensor].readerPos = (gReceiver[sensor].readerPos+1) % RINGBUFF_MAX;
    
    if (gReceiver[sensor].buff[gReceiver[sensor].readerPos] & FALLING_EDGE)
    {
#ifdef DISPLAY_SIGNAL_TIMINGS      
      Serial.print("F ");
//      Serial.print("Falling ");
      Serial.print(gReceiver[sensor].buff[gReceiver[sensor].readerPos] & ~FALLING_EDGE);
//      Serial.print(gReceiver[sensor].buff[gReceiver[sensor].readerPos] & ~FALLING_EDGE);
      Serial.print(" ");
#endif
      if (gReceiver[sensor].lastWasRising)
      {
        int durationTicks = ((lastTime - (gReceiver[sensor].buff[gReceiver[sensor].readerPos] & ~FALLING_EDGE))+ MAX_COUNTER)%MAX_COUNTER;
        float duration = durationTicks / 84.0; //the clock we're using has 84 ticks per us
#ifdef DISPLAY_RAW_SIGNAL_DURATIONS         
//        Serial.print(", ");
        Serial.print(duration);
      
        Serial.print(" us ");
#endif



        // 10.416667 is a bit of a magic number  
        // Derived from data at: 
        // https://github.com/nairol/LighthouseRedox/blob/master/docs/Light%20Emissions.md
        float durationDecoded = duration / 10.41667; 

        
        if (durationDecoded > 5.5 && durationDecoded < 14.5)
        {
          
#ifdef DISPLAY_OOTC_RAW_TIME
           Serial.print(durationTicks);
           Serial.print("  ");
//           Serial.print(durationTicks);
//           Serial.print("  ");
#endif
            gReceiver[sensor].OotcPulseStartTime = lastTime;
           // okay, looks like a OOTX frame, let's decode it
           int pulseVal = (durationDecoded + 0.5); // add .5 so we round to the nearest instead of truncating
           pulseVal -= 6; // subtract 6 because the first value (0 index) starts at a pulse value of 6

           gReceiver[sensor].rotor = pulseVal & 0x01;
           gReceiver[sensor].data =  pulseVal & 0x02;
           gReceiver[sensor].skip =  pulseVal & 0x04;

           ProcessOotcBit(gReceiver[sensor].data);

#ifdef DISPLAY_OOTC
           Serial.print("  data: ");
           Serial.print(data);
           Serial.print("  rotor: ");
           Serial.print(rotor);
           Serial.print("  skip: ");
           Serial.println(skip);
#endif

           gReceiver[sensor].lastPulseWasOotc = true;
        }

        
      }

      lastTime = gReceiver[sensor].buff[gReceiver[sensor].readerPos] & ~FALLING_EDGE;
      gReceiver[sensor].lastWasRising=false;
      
    }
    else
    {
#ifdef DISPLAY_SIGNAL_TIMINGS      
      Serial.print("R ");
//      Serial.print("Rising  ");
      Serial.print(gReceiver[sensor].buff[gReceiver[sensor].readerPos]);
      Serial.print("  ");
#endif

      if (0)//(!lastWasRising)
      {
        Serial.print("               ");
        Serial.println(lastTime - (gReceiver[sensor].buff[gReceiver[sensor].readerPos] & ~FALLING_EDGE));
      }

      lastTime = gReceiver[sensor].buff[gReceiver[sensor].readerPos];
      gReceiver[sensor].lastWasRising=true;

      if (gReceiver[sensor].lastPulseWasOotc)
      {
          // This is the rising edge of our sweep pulse
          int durationTicks = ((gReceiver[sensor].OotcPulseStartTime - (gReceiver[sensor].buff[gReceiver[sensor].readerPos]))+ MAX_COUNTER)%MAX_COUNTER;
          float duration = durationTicks / 84.0;

#ifdef DISPLAY_ANGULAR_DURATIONS          
          Serial.print(" ad:");
          Serial.print(durationTicks);
          Serial.print(" ");
#endif

#define CYCLES_PER_SECOND 60.0
#define DEGREES_PER_CYCLE 360.0
#define S_PER_MS (1.0/1000.0)
#define CYCLES_PER_SECOND (1.0/60.0)

// durationTicks is in units of 1/84,000,000 s
          OnNewAngle(sensor,gReceiver[sensor].rotor, durationTicks * 0.000257143);

// TODO: Why does the below not give the same results as above.  Must be some rounding or something?  figure it out later...
//          OnNewAngle(0,rotor, (float)durationTicks / 84000000.0 * CYCLES_PER_SECOND * DEGREES_PER_CYCLE);


//          OnNewAngle(0,rotor, duration * S_PER_MS * CYCLES_PER_SECOND * DEGREES_PER_CYCLE);
       
          gReceiver[sensor].lastPulseWasOotc = false;
      }

      
    }
  }
}



void OnNewAngle(int irReceiver, bool rotor, float angle)
{
#ifdef DISPLAY_RAW_ANGLE
  Serial.print(irReceiver);
  Serial.print(".");
  if (rotor)
  {
    Serial.print("Y:");
  }
  else
  {
    Serial.print("X:");
  }
  Serial.print(angle,3);
  Serial.print("  ");
#endif

#ifdef DISPLAY_NEWLINE_AFTER_Y
  if (rotor && irReceiver == 5)
  {
    Serial.println("");
  }
#endif

  if (rotor)
  {
    gReceiver[irReceiver].Y = angle;
  }
  else
  {
    gReceiver[irReceiver].X = angle;
  }
#ifdef DISPLAY_ANGLES
//          Serial.print(angle);
if (rotor && irReceiver==5)
{
  for (int a=0; a<6; a++)
  {
    Serial.print(a);
    Serial.print(".X:");
    Serial.print(gReceiver[a].X,3);
    Serial.print(" ");
    Serial.print(a);
    Serial.print(".Y:");
    Serial.print(gReceiver[a].Y,3);
    Serial.print("   ");
  }
  Serial.println("");
}
#endif 
}

void ProcessOotcBit(bool ootcBit)
{
#ifdef DISPLAY_OOTC_DATA
  static int zeroBitCount = 0;
  
  Serial.print(ootcBit);
  
  if (zeroBitCount == 17 && ootcBit == 1)
  {
    Serial.println("");
  }
  
  if (ootcBit == 1)
  {
    zeroBitCount = 0;
  }
  else
  {
    zeroBitCount++;
  }
#endif
}

// note: if it's ever found that we're spending too long in the interrupt service routines, we can simplify
// these quite a bit by eliminating the call to millis() and instead raising the value of SysTick->LOAD
// The reason we're using millis() here is that if we change SysTick->LOAD, that will change
// other timing calculations done by the Arduino code.  
void rising0_OLD()
{
  if (gReceiver[0].readerPos != gReceiver[0].writerPos)
  {
//    gReceiver[0].buff[gReceiver[0].writerPos] = SysTick->VAL;
    gReceiver[0].buff[gReceiver[0].writerPos] = SysTick->VAL  + ((10000-(millis()%10000)) * 84000);

    // increment the writer pos and use modulo to wrap back to the beginning
    // of the ring buffer iff we're past the end of the buffer.
    gReceiver[0].writerPos = (gReceiver[0].writerPos+1) % RINGBUFF_MAX;
  }
}

void falling0_OLD()
{
  if (gReceiver[0].readerPos != gReceiver[0].writerPos)
  {
    // tack on the FALLING_EDGE bit to indicate this is a falling edge.
//    gReceiver[0].buff[gReceiver[0].writerPos] = SysTick->VAL | FALLING_EDGE;
    gReceiver[0].buff[gReceiver[0].writerPos] = (SysTick->VAL + ((10000-(millis()%10000)) * 84000)) | FALLING_EDGE;

    // increment the writer pos and use modulo to wrap back to the beginning
    // of the ring buffer iff we're past the end of the buffer.
    gReceiver[0].writerPos = (gReceiver[0].writerPos+1) % RINGBUFF_MAX;
  }
}

//    gReceiver[receiver].buff[gReceiver[receiver].writerPos] = SysTick->VAL  + ((10000-(millis()%10000)) * 84000);\

#define RISING_INTERRUPT_BODY(receiver) \
  if (gReceiver[receiver].readerPos != gReceiver[receiver].writerPos)\
  {\
    gReceiver[receiver].buff[gReceiver[receiver].writerPos] = SysTick->VAL;\
    gReceiver[receiver].writerPos = (gReceiver[receiver].writerPos+1) % RINGBUFF_MAX;\
  }

//    gReceiver[receiver].buff[gReceiver[receiver].writerPos] = (SysTick->VAL + ((10000-(millis()%10000)) * 84000)) | FALLING_EDGE;\

#define FALLING_INTERRUPT_BODY(receiver)\
  if (gReceiver[receiver].readerPos != gReceiver[receiver].writerPos)\
  {\
    gReceiver[receiver].buff[gReceiver[receiver].writerPos] = (SysTick->VAL) | FALLING_EDGE;\
    gReceiver[receiver].writerPos = (gReceiver[receiver].writerPos+1) % RINGBUFF_MAX;\
  }

void rising0()
{
  RISING_INTERRUPT_BODY(0)
}
void falling0()
{
  FALLING_INTERRUPT_BODY(0)
}
void rising1()
{
  RISING_INTERRUPT_BODY(1)
}
void falling1()
{
  FALLING_INTERRUPT_BODY(1)
}
void rising2()
{
  RISING_INTERRUPT_BODY(2)
}
void falling2()
{
  FALLING_INTERRUPT_BODY(2)
}
void rising3()
{
  RISING_INTERRUPT_BODY(3)
}
void falling3()
{
  FALLING_INTERRUPT_BODY(3)
}
void rising4()
{
  RISING_INTERRUPT_BODY(4)
}
void falling4()
{
  FALLING_INTERRUPT_BODY(4)
}
void rising5()
{
  RISING_INTERRUPT_BODY(5)
}
void falling5()
{
  FALLING_INTERRUPT_BODY(5)
}

