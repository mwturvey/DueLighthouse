

//#define DISPLAY_OOTC
#define DISPLAY_OOTC_DATA
//#define DISPLAY_ANGLES

#define RINGBUFF_MAX 200
#define TICKS_PER_US 84 //84 ticks per microsecond

// the high order bit is used to indicate if the entry is a rising or falling edge.
#define RISING_EDGE 0x00000000
#define FALLING_EDGE 0x80000000

class ringBuffer
{
  public:
  short readerPos; // last place read from
  short writerPos; // next place to write
  // note: if readerPos == writerPos, that is the state we hit when
  // the ring buffer is full.  We do this so that the check for a full
  // ring buffer is as simple as possible in the interrupt routine
  // i.e. all it has to do is a quick comparison to see if it's safe to write.
  unsigned int buff[RINGBUFF_MAX];

  ringBuffer()
  {
    readerPos=0;
    writerPos=1;
  }  
  
};

volatile ringBuffer ring1;

long int down=0, up=0;
// the setup function runs once when you press reset or power the board
void setup() 
{
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  pinMode(3, INPUT_PULLUP);
  Serial.begin(115200);

  Serial.print("SysTick->LOAD: ");
  Serial.println(SysTick->LOAD);

  SysTick->LOAD = 8399999;

  Serial.print("SysTick->LOAD: ");
  Serial.println(SysTick->LOAD);

  // flip the FALLING and RISING here because the input signal is active low
  attachInterrupt(digitalPinToInterrupt(4), rising1, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), falling1, RISING);
}

// the loop function runs over and over again forever
void loop() 
{

/*
  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delayMicroseconds(1);              // wait for a second
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);              // wait for a second


  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delayMicroseconds(10);              // wait for a second
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);              // wait for a second


  digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
  delayMicroseconds(100);              // wait for a second
  digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
  delay(100);              // wait for a second
*/
delay (1);  

bool data;
bool rotor;
bool skip;
           
static bool lastPulseWasOotc = false;
  static bool lastWasRising = false;
  static int OotcPulseStartTime = 0;
//  Serial.println("");
  int lastTime = 0;
  // yeah, this is a hairy conditional.  
  while (((ring1.writerPos - 1 - ring1.readerPos) + RINGBUFF_MAX) % RINGBUFF_MAX != 0)
  {
    ring1.readerPos = (ring1.readerPos+1) % RINGBUFF_MAX;
    
    if (ring1.buff[ring1.readerPos] & FALLING_EDGE)
    {
//      Serial.print(",F, ");
//      Serial.print("Falling ");
//      Serial.println(ring1.buff[ring1.readerPos] & ~FALLING_EDGE);
//      Serial.print(ring1.buff[ring1.readerPos] & ~FALLING_EDGE);
//      Serial.print(", ");

      if (lastWasRising)
      {
        int durationTicks = ((lastTime - (ring1.buff[ring1.readerPos] & ~FALLING_EDGE))+ 83999)%83999;
        float duration = durationTicks / 84.0; //the clock we're using has 84 ticks per us
//        Serial.print(", ");
//        Serial.println(duration);
        //Serial.println(" us");


        // 10.416667 is a bit of a magic number  
        // Derived from data at: 
        // https://github.com/nairol/LighthouseRedox/blob/master/docs/Light%20Emissions.md
        float durationDecoded = duration / 10.41667; 
        
        if (durationDecoded > 5.5 && durationDecoded < 14.5)
        {

            OotcPulseStartTime = lastTime;
           // okay, looks like a OOTX frame, let's decode it
           int pulseVal = (durationDecoded + 0.5); // add .5 so we round to the nearest instead of truncating
           pulseVal -= 6; // subtract 6 because the first value (0 index) starts at a pulse value of 6

           rotor = pulseVal & 0x01;
           data =  pulseVal & 0x02;
           skip =  pulseVal & 0x04;

#ifdef DISPLAY_OOTC
           Serial.print("  data: ");
           Serial.print(data);
           Serial.print("  rotor: ");
           Serial.print(rotor);
           Serial.print("  skip: ");
           Serial.println(skip);
#endif
#ifdef DISPLAY_OOTC_DATA
          static int zeroBitCount = 0;

          Serial.print(data);

          if (zeroBitCount == 17 && data == 1)
          {
            Serial.println("");
          }

          if (data == 1)
          {
            zeroBitCount = 0;
          }
          else
          {
            zeroBitCount++;
          }

#endif

           lastPulseWasOotc = true;
        }

        
      }

      lastTime = ring1.buff[ring1.readerPos] & ~FALLING_EDGE;
      lastWasRising=false;
      
    }
    else
    {
//      Serial.print("R, ");
//      Serial.print("Rising  ");
//      Serial.print(ring1.buff[ring1.readerPos]);
//      Serial.print(", ");

      if (0)//(!lastWasRising)
      {
        Serial.print("               ");
        Serial.println(lastTime - (ring1.buff[ring1.readerPos] & ~FALLING_EDGE));
      }

      lastTime = ring1.buff[ring1.readerPos];
      lastWasRising=true;

      if (lastPulseWasOotc)
      {
          // This is the rising edge of our sweep pulse
          int durationTicks = ((OotcPulseStartTime - (ring1.buff[ring1.readerPos]))+ SysTick->LOAD)%SysTick->LOAD;
          float duration = durationTicks / 84.0;
//          Serial.print("       angular_duration: ");
//          Serial.print(duration);
//          Serial.print(" us  ");

#define CYCLES_PER_SECOND 60.0
#define DEGREES_PER_CYCLE 360.0
#define S_PER_MS (1.0/1000.0)
#define CYCLES_PER_SECOND (1.0/60.0)

#ifdef DISPLAY_ANGLES
          Serial.print(duration * S_PER_MS * CYCLES_PER_SECOND * DEGREES_PER_CYCLE,4);
          Serial.print("  ");
//          Serial.print(" degrees  ");
         
          if (rotor)
          {
            Serial.println("");
          }
#endif 
       
          lastPulseWasOotc = false;
      }

      
    }
  }
  
  
 

/*  
  int a, b, c, d, e, f, g;
  a=SysTick->VAL;
  b=SysTick->VAL;
  c=SysTick->VAL;
  d=SysTick->VAL;
  e=SysTick->VAL;
  f=SysTick->VAL;
  g=SysTick->VAL;
  Serial.print(a-b);
  Serial.print("  ");
  Serial.print(b-c);
  Serial.print("  ");
  Serial.print(c-d);
  Serial.print("  ");
//  Serial.print(d-e);
//  Serial.print("  ");
//  Serial.print(e-f);
//  Serial.print("  ");
//  Serial.print(f-g);
//  Serial.print("  ");
//  Serial.print(d);
  Serial.println("");
  */
}


void rising1()
{
  if (ring1.readerPos != ring1.writerPos)
  {
    ring1.buff[ring1.writerPos] = SysTick->VAL;

    // increment the writer pos and use modulo to wrap back to the beginning
    // of the ring buffer iff we're past the end of the buffer.
    ring1.writerPos = (ring1.writerPos+1) % RINGBUFF_MAX;
  }
}

void falling1()
{
  if (ring1.readerPos != ring1.writerPos)
  {
    // tack on the FALLING_EDGE bit to indicate this is a falling edge.
    ring1.buff[ring1.writerPos] = SysTick->VAL | FALLING_EDGE;

    // increment the writer pos and use modulo to wrap back to the beginning
    // of the ring buffer iff we're past the end of the buffer.
    ring1.writerPos = (ring1.writerPos+1) % RINGBUFF_MAX;
  }
}


