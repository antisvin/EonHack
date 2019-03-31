/*      Written by Dr. Crash, version 1.000 (2016.11.24)

      THIS CODE IS LICENSED UNDER THE GPL VERSION 2.  SHARE ALL!!!

      Test code to explore the Qu-Bit EON eurorack synth module which uses an
      ATMEL ATmega 328P processor (same as most Arduinos!)  You can either use
      an ATMEL six-pin programmer, or wire an Arduino to _be_ an ATMEL programmer
      (the code to do this comes with the Arduino environment)

      DANGER DANGER DANGER!   There is NO PROVISION HERE for going back to the
      as-shipped code for the Qu-Bit EON.  Once you reprogram your EON, it's like the
      Alanis Morissette song- "the only way out is through".  You MIGHT have a brick;
      it's up to you to make the most of it.

      THERE IS NO GOING BACK until and unless Qu-Bit releases a .HEX upgrade or
      suchlike.  On the other-other hand, "freedom is truly having nothing else
      left to lose."  Make the most of your freedom.

      I TAKE NO RESPONSIBILITY FOR YOU GETTING THIS WRONG!!!  ALTHOUGH YOU CAN
      ALWAYS UNBRICK THE EON'S CPU WITH ANOTHER ARDUINO OR THE ATMEL ICSP PROGRAMMER,
      YOU CAN NEVER GO BACK TO THE ORIGINAL EON SOFTWARE!

      I _INTENTIONALLY_ did not try to suck the code out of the Qu-bit EON, so I
      cannot be accused of plaigarism, nor of DMCA violation since no copyrighted
      code was permitted to exist nor do my actions allow anyone to circumvent a
      copyright technical control (if anything, I merely destroyed previously copyrighted
      information - and the DMCA does not forbid the burning of books).  Everything
      I learned, I learned with a scope and spec sheets published by the CPU manufacturer.

      YOU WILL ALSO NEED TO ADD ONE RESISTOR.  For some unexplained reason, there's a
      huge amount of crosstalk between the E-O-N frontpanel switch and the CV Input.  I
      fixed it with a 10K ohm resistor soldered between pin 27 of the microprocessor and
      ground (actually, I used the via attached to pin 27.  This is the via closest to the
      east (right) corner of the microprocessor, if you hold the module with the E-O-N
      switch at the top and the microprocessor facing you.  The nearest convenient ground
      is the bottommost pin in the 4-pin + 3-pin mainboard-to-front panel connection all
      the way to the west (left) from the via.

      Put the bias jumper in the lower (+-2.5 volt) position.  You could hack the code below
      to use the +-5V position, but it will cost you a bit of resolution.

      Then adjust all three pots to approximately the midpoint of rotation (you can adjust
      the topmost one +- 45 degrees to center the the N-mode voltage to +-2.5 volts.)

      HOW TO USE IT ONCE YOU'VE COMMITTED AND REFLASHED THE CPU AND ADDED THE RESISTOR,
      YOU ARE READY TO ROCK.

      Here's how to use it.   I suggest you add a sticker to help you remember.

      E-mode - E-mode is a sample/hold/quantize.

          NORMAL mode - samples on TRIG, quantizes CVin to 2 / 4 / 8 / 16... # of levels is
           controlled by KNOB1, spread over 0-5 volts.  Not your average quantizer.
           No concept of major/minor/chord.  You want those, buy a Penrose.  KNOB2 adds a
           displacement to the output value.
          ALT mode - KNOB2 becomes an LFO which triggers sampling and quantization,
           and a TRIG pulse (long enough to kick a MATHS into operation.
          PB1 - toggles "auto-trig" mode - sampling is continuous, and TRIG fires
           whenever the value changes enough to move to another quantization bucket.

      O-mode - Arpeggitron.
          NORMAL mode - CVin is the base pitch; KNOB1 controls the increment between
           arpeggiated notes.  KNOB2 controls how many notes in the arpeggiation.
           TRIG steps from one note to the next.
          ALT mode - CVin is base pitch, KNOB1 controls increment, KNOB2 is an LFO that
           acts like TRIG (which still works!  Try ratcheting!).  Number of notes in the
           arpeggiation is what you set in NORMAL mode.
          PB1 - switches the arpeggiation from up, to down, to pendulum, to random(ish).

      N-mode - N-mode is an oscillator
          NORMAL mode - CVin and KNOB1 controls the pitch of a supersine (TZPMed) oscillator.
           Amount of TZPM (a.k.a. timbre) is controlled by KNOB2.  TRIG is hard sync
          ALT mode - CVin and KNOB1 control the pitch of a supersaw (multiple detuned saw osc.
           Amount of saw detune is controlled by KNOB2.   Setting KNOB2 to zero will cause
           the saws to reconverge and stay converged.  TRIG is hard sync.
          PB1 - kicks it into "low gear" - oscillations are now LFO.  Very interesting to
           use as AM or FM on another VCO.

  You may ask why N-mode (oscillator) isn't O-mode (oscillator) - that's because E and O are 0-5 volt
  outputs, while N mode switches the outputs in hardware to be +-2.5 volts.


      Note that the EON module has six pins in line for in-circuit software
      programming (a.k.a. ICSP) - the pins, from the top of the module down toward the
      power socket, are VCC, SCK, MISO, MOSI, RESET*, and GND.   You can easily program
      an Arduino to become an ICSP programmer and drive these pins to reprogram the EON
      to be anything you like.

      Current EONs come with a 20 MHz crystal, so there's no real room left to overclock;
      it's a pretty slow processor so there's not a lot-lot you can do.

      The three pots on the EON are, from the top:
        1) (top) DC value read on the EON switch pin in when in the N position,
        2) (middle) DC offset (centering) of OUT1 - but only sometimes!
        3) (bottom) unknown.

      Outputs: there are two on the board:  OUT2 (the lower jack) is an R-2R ladder
      driven by data port PD (8 pins, low order bits) and Port B bits 0, 1, 2, and 3.)
      PD0 is the least significant bit, and PB3 is the most significant.

      Compared to most R-2R ladders it's kinda slow (10KHz at -3dB, but it starts rounding
      a triangle wave at about 2 KHz.   Still, that's not absolutely horrible... 2KHz is
      still just barely on a piano keyboard.   It'll be down about 1 or 2 dB by D8.  The
      same R-2R A-D ladder that drives the OUT2 jack also drives the LED inside the
      pushbutton.

      OUT1 (the upper jack on the front panel) is a single output, driven by port C
      bit 1 (2nd LSB, counting from 0, a.k.a. bit PC1). It also drives the small yellow LED.

      Note that this is not a very fast CPU- only 8 bits, and has a slow ADC as well.
      If you do NOTHING other than read the 5 analog ins and the digital in (the EON switch is
      actually analog - high/low/medium!) and toggle the OUT1 digital output, you can go about
      11 KHz.  So, it's a 22 KHz loop (you go through it twice to toggle it).

      To get dependable switching on the EON switch itself I added a 100K resistor (red-black-yellow)
      between CPU pin 27 (EON IN) and ground.   There's probably a dozen other places you
      could add it (like the switch itself) but I can only say what I know works.

      Here's a quick chart of what goes where, for you Arduino types:

      Arduino pin       ATMEL 328P pin   Used here as:
      0 (digital)        PD0              Low order bit of the R-2R DAC
      1...7              PD1-PD7          middle bits of R-2R DAC
      8... 10            PB0-2            higher middle bits of R-2R DAC
      11                 PB3              high order bit of R-2R DAC
      12                 PB4              UNKNOWN!
      13 (the "LED" pin) PB5              Pushbutton contacts (it's an input!)
      14                                  UNKNOWN!
      A0    (23)         PC0              UNKNOWN!
      A1    (24)         PC1              OUT1 (the jack with the yellow LED; digital out)
      A2    (25)         PC2              KNOB1 input
      A3    (26)         PC3              TRIG Jack input (yes, it's analog!)
      A4    (27)         PC4 (SDA)        EON switch input (also analog)
      A5    (28)         PC5 (SCL)        NORM/ALT switch (also analog) - 1=NORM, 0=ALT
      A6    (19)                          CV Jack input
      A7    (22)                          KNOB2 input

*/

//     Define cbi and sbi to do bitwhacking when needed.
//
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif
#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif


#include <avr/pgmspace.h>
#include "prescaler.h"
#include "number_table.h"
// #include <BigNumber.h>

//  These are analog reads:
#define UNK1 A0
#define OUT1 A1
#define KNOB1 A2
#define TRIGIN A3
#define EON A4
#define NORMALT A5
#define CVIN A6
#define KNOB2 A7
//   These are digital reads:
#define UNK0 14
#define PUSHSWITCH 13
#define TRIGSWITCH 17
#define EONSWITCH 18
#define NORMALTSWITCH 19
//   Digital Writes  (OUT2 is an R2R network)
#define OUT1 A1

// the setup routine runs once when you press reset:
void setup()
{
  // initialize the digital pin as an output.
  int i;
  for ( i = 0; i < 25; i++)
  {
    // DO NOT SET ALL PINS AS OUTPUTS!   YOU MIGHT BURN UP
    //   THE 328P IO DRIVERS AND HI-Z ADC ISOLATORS - DAMHIK!
    //pinMode(i, OUTPUT);
    //digitalWrite (i, 1);  //  Turn on the bits (which are also the pullups)
    pinMode(i, INPUT);
    digitalWrite (i, 0);  //  Turn off the bits (which are also the pullups)
  }
  //   Pins 0 thru 11 are the 12 bits of R-2R converter input
  for ( i = 0; i < 12; i++)
  {
    pinMode (i, OUTPUT);    // digitals to all R-2R outputs
    digitalWrite (i, 0);
  }


  //   Now mangle a few things that aren't obvious
  //  A1 is the OUT1 jack and LED, not an input.
  pinMode (OUT1, OUTPUT);
  digitalWrite (OUT1, 0);

  DDRC = DDRC & 0x02 | 2;
  //PORTC = PORTC | 32 ;      // activate pullup resistor on C4 (a.k.a. A4, EON switch)

  //   and set the system oscillator to go as fast as we can...
  setClockPrescaler (0);

  //    The mystery EON switch explained (sorta)
  //
  //    So, how can an ON-OFF-ON switch get read?  The obvious thing is to put a voltage divider on
  //    one side and full V++ on the other, but that's only _sorta_ what Qu-Bit did - the voltage divider
  //    is the pot on the very end of the PCB, but the pot is shared with the voltage offset on OUT1
  //    when the switch is thrown.  WTF????  Even more amusing is that the voltage goes thru some more
  //    mumbo jumbo, since it ALSO gets crosstalk from the voltage out from the R-2R ladder feeding OUT2,
  //    so it' not like they used one of the eight TL074-style op-amps on the board to actually do the
  //    adding properly (and why would they in the first place???  Some things about the board still
  //    make no sense to me...) .
  //
  //    To solve what goes where, once I had the R-2R scoped out, I put in simple programs that did an
  //    AnalogRead on each of the analog pins and outputted the value both on the R-2R and via duty cycle
  //    of the digital out on OUT1.
  //
  //    Don't get confused by what an oscilloscope will show you from DACOUT, since the EON switch
  //    and the pots will mess around with the gain.  Instead, use duty cycle on OUT1 to read out
  //    what the EON analog port is returning, since even if OUT1 is DC-shifted, the duty cycle is
  //    still easy to read.
  //
  //    If you do that and the rest of the pot are set to mid-rotation, then the following setup will
  //    let you set the EON analog reading to around 50% of full scale with the topmost pot.
  //    The other pots do something; we just don't know what yet!
  //
  digitalWrite (EON, 0);     //   seems to work better to turn off the pullup
  pinMode (EON, INPUT);

  //    Mucking around to try to set offsets of the R-2R when in "N" on the EON switch
  //    Normal EON switch and R2R seems to happen when the following config is set up:
  //    A0:in,1, D12:in,0, D14,in,1, A6,in,0... then adjust the topmost pot to get 0-5 sweep
  //    in calibration mode ("ticker ramps") in E and O and -2.5 to +2.5 in N and the jumper
  //    in the lower (closer to power-in pins) position

  //   Don't do these as output, it possibly could blow the pin isolators! (or maybe not...)
#ifdef SELF_DESTRUCT
  pinMode (A0, OUTPUT);
  digitalWrite (A0, 0);      //  Setting this to live output with a 1 seems to move OUT1 to -6 volt base!  WTF?
  pinMode (A0, INPUT);
  pinMode (12, OUTPUT);      //   Pin 12 output is weird too.
  digitalWrite (12, 0);      //  AS OUTPUT: 1--> 3.0v,2,4V,1.5V ; 0--> .5V, 0V, -1V
  pinMode (12, INPUT);     //  AS INPUT : 1 --> +1v,0v,-1v
  pinMode (14, OUTPUT);      //   Pin 12 output is weird too.
  digitalWrite (14, 0);      //  AS OUTPUT: 1--> 3.0v,2,4V,1.5V ; 0--> .5V, 0V, -1V
  pinMode (14, INPUT);     //  AS INPUT : 1 --> +1v,0v,-1v
  pinMode (A6, OUTPUT);    //  A6 is also the reset pin
  digitalWrite (A6, 0);
  pinMode (A6, INPUT);
#endif

  //  Set the ADC to go as fast as it can.
  ADCSRA &= ~7;           // remove prescaler bits set by Arduino library
  ADCSRA |= 6;            //  Set prescaler to 2 or 3 (i.e. divide by 2 or 4; was 128 in standard Arduino)

  cli();//disable interrupts
  //set timer0 interrupt at 40kHz
  TCCR0A = 0;// set entire TCCR0A register to 0
  TCCR0B = 0;// same for TCCR0B
  TCNT0  = 0;//initialize counter value to 0
  // set compare match register for 40khz increments
  OCR0A = 52;// = (20*10^6) / (48000*8) - 1 (must be <256)
  //OCR0A = 49;// = (16*10^6) / (40000*8) - 1 (must be <256)
  // turn on CTC mode
  TCCR0A |= (1 << WGM01);
  // Set CS11 bit for 8 prescaler
  TCCR0B |= (1 << CS11);
  // enable timer compare interrupt
  TIMSK0 |= (1 << OCIE0A);
  sei();//enable interrupts

}

//   Declare the globals that we sample on every pass thru anyway, or output on
//   every pass anyway:
//      Global Inputs:
int knob1, knob2, trigin, cvin, eon, estate, normaltswitch, pushswitch;
int unk1;
//      Global Outputs:
unsigned long dacout, digout;
//      Global States:
volatile unsigned long pushed, triggered, pbholdcounter, trigcounter;
unsigned pbheld = 0;
volatile unsigned pblongheld = 0;
unsigned long ticker = 0;
int ecount;

// the loop routine runs over and over again forever:
void loop()
{
  //   Our three options are:
  //   E - was Envelope, now is S/H/quant
  //   O - oscillator (sine distortion and supersaw)
  //   N - arpeggitron
  estate = 0;
  pbheld = 0;
  while (1)
  {
    //   Do the reads
    knob1  = analogRead (KNOB1);                   // VERIFIED
    knob2  = analogRead (KNOB2);                   // VERIFIED
    cvin   = analogRead (CVIN);                    // VERIFIED
    eon = analogRead (EON);                       // VERIFIED
    trigin = digitalRead (TRIGIN);                      // VERIFIED
    normaltswitch = digitalRead (NORMALTSWITCH);   // VERIFIED
    pushswitch = digitalRead (PUSHSWITCH);         // VERIFIED

    //    Deal with the global states of pushbuttons and triggers
    pushed = (pushswitch && (pbholdcounter == 0)) ? 1 : 0 ;
    if (pushed) pbholdcounter = 0;
    pbholdcounter = (pushswitch) ? pbholdcounter + 1 : 0 ;
    if (pbholdcounter == 1000) {
      pbheld = 1 - pbheld;
    }
    //if (pbholdcounter == 10000) {pblongheld = 1 - pblongheld;}

    triggered = (trigin && (trigcounter == 0)) ? 1 : 0;
    if (triggered) trigcounter = 0;
    trigcounter = (trigin) ? trigcounter + 1 : 0;
    //trigcounter = (trigin) ? trigcounter + 1 : 0;
  }
}

////  EXAMPLE sine oscillator.   Not Used in this code but it's good example code
//void example_sineoscillator ()
//{
//  static unsigned long phaseaccum;
//  unsigned long phaseout;
//  unsigned long increment;
//#define SCALESHIFT 7
//#define PHASEMAX  ((0x00000001L<<(SCALESHIFT+12))-1)
//
//  //     sine wave oscillator
//  increment = (knob1 + cvin) >> 1;
//  increment = pgm_read_word_near (exp_table + increment);
//  phaseaccum += increment;
//  phaseaccum = phaseaccum & PHASEMAX;
//  phaseout =  phaseaccum >> SCALESHIFT;
//  phaseout = phaseout & 0x00000FFFL;
//  dacout = pgm_read_word_near (sinetable + phaseout); // note C-style punning an array name
//  digout = dacout > 0x7FF;
//}

//      SuperSine Oscillator - actually a TZ phase modulated sine
//
void supersineoscillator ()
{
  static unsigned long phaseaccum;
  unsigned long phaseout;
  long exp_knob1, exp_cvin;
  long increment;
  //  int phaserem;
#define SCALESHIFT 8
#define TZFM_SCALEDOWN 7
#define SINE_PHASEMAX  ((0x00000001L<<(SCALESHIFT+12))-1)
  //#define INTERPOLATE 4

  //     TZFM sine wave oscillator
  exp_cvin  = pgm_read_dword_near (exp_table + cvin) >> 5;
  exp_knob1 = pgm_read_dword_near (exp_table + knob1) >> 3;
  //    Yes, I know I should have been able to do the following multiply
  //    post-exponentiation with an add pre-exponentiation, but I was
  //    not able to get it to work.  Shame on me!
  increment = exp_cvin * exp_knob1;
  //increment = pgm_read_dword_near (exp_table + cvin + knob1);
  if (pbheld) increment = increment >> 5;
  if (triggered) phaseaccum = 0;          // Hard synch
  phaseaccum += increment;
  phaseaccum = phaseaccum & SINE_PHASEMAX;
  phaseout =  phaseaccum >> SCALESHIFT;
  phaseout = phaseout & 0x0000FFFL;
  dacout = read_sine_quadrant(phaseout); // note C-style punning an array name
  //    And now the super-sine (phase distortion)

  phaseout = phaseout + (((dacout - 2048) * knob2 ) >> TZFM_SCALEDOWN);
  //phaseout = phaseout + (((dacout-2048) * knob2 ) >> (TZFM_SCALEDOWN - INTERPOLATE));
  //phaserem = (phaseout << (16 - INTERPOLATE)) >> (16 - INTERPOLATE); // Last INTERPOLATE bits are stored here
  //phaseout = phaseout >> INTERPOLATE;
  phaseout = phaseout & 0x00000FFFL;
  dacout = read_sine_quadrant(phaseout); // >> 4;

}

//    A three-saw supersaw (phase1 = 50%, phase2 = 25%, phase3 = 25%
void supersawoscillator()
{
  static unsigned long phaseaccum, phaseaccum1, phaseaccum2, phaseaccum3;
  unsigned long phaseout;
  long exp_knob1, exp_cvin;
  long increment;
#define SCALESHIFT 8
#define PHASEMAX  ((0x00000001L<<(SCALESHIFT+12))-1)
#define KNOBSHIFT 1

  if (knob2 == 0)
  { phaseaccum2 += (phaseaccum2 > phaseaccum1) ? -50 : +51;
    phaseaccum3 += (phaseaccum3 > phaseaccum1) ? -50 : +51;
  }
  else
  { phaseaccum2 += (phaseaccum2 > phaseaccum1) ? -1 : 0;
    phaseaccum3 += (phaseaccum3 > phaseaccum1) ? 0 : +1;
  }

  if (triggered) phaseaccum1 = phaseaccum2 = phaseaccum3 = 0;          // Hard synch

  //     supersaw oscillator
  exp_cvin  = pgm_read_dword_near (exp_table + cvin);
  exp_knob1 = pgm_read_dword_near (exp_table + knob1);
  increment = (exp_cvin * exp_knob1) >> SCALESHIFT;
  if (pbheld) increment = increment >> 7;

  //increment = pgm_read_word_near (exp_table + increment);

  phaseaccum1 += increment;
  phaseaccum1 = phaseaccum1 & PHASEMAX;

  phaseaccum2 += increment + (knob2 >> KNOBSHIFT);
  phaseaccum2 = phaseaccum2 & PHASEMAX;

  phaseaccum3 += increment - (knob2 >> (KNOBSHIFT + 1));
  phaseaccum3 = phaseaccum3 & PHASEMAX;

  phaseaccum = (phaseaccum1 + phaseaccum1 + phaseaccum2 + phaseaccum3) >> 2;
  //phaseaccum = phaseaccum & PHASEMAX;
  phaseout =  phaseaccum >> SCALESHIFT;
  phaseout = phaseout & 0x00000FFFL;
  dacout = phaseout;
}

#define QUANTIZER
#ifdef QUANTIZER
void samp_quant_slide_hold ()
{
  //  Sample/Hold/Quantizer

  //    Sample and hold the value on valin, as clocked by clockin
  //    All modes - CVin is the value
  //    NORM mode:
  //       CVin is value to be quantized
  //       TRIGin is sample/hold/quantize trigger
  //       KNOB1 is the quantization granularity (out of 10 bits) - CW = more levels; full CW = no quant
  //       KNOB2 is the slew limiter - CW = faster tracking, full CW - 1 cycle limit.
  //       PUSHBUTTON - triggers a sample as well
  //       OUT1 is the "Slew Complete" value.  Loop into TRIG for self triggering
  //       OUT2 is the SHQed value
  //    ALT mode:
  //       KNOB2 becomes the frequency of self-clocking.
  //    Pushbutton pushed:
  //       Use an internal high speed ramp as the signal input.
  static long capturedcv, quantizedcv;
  static long quantizemask, quantizeoffset, knobadd;
  static long olddacout;
  static unsigned long internallfo;

  //  if NORMALTSWITCH then use a built-in LFO to create triggers
  if (! normaltswitch)
  {
    internallfo += pgm_read_word_near (exp_table + knob2 );
    if (internallfo > 10000000)
    {
      internallfo = 0;
      triggered = 1;
    }
  }

  //   Handle trigger ( captures cvin to capturedcv)
  //   Trigger on TRIGIN or if we're in PBHELD
  if (triggered || pbheld)
  {
    capturedcv = cvin << 3;
  }
  //    Handle quantization
  //    The quantize mask is the bits we keep.
  quantizemask = 0xfffff000 >> (knob1 >> 7);
  quantizedcv = (capturedcv & quantizemask) ;
  //   the quantize offset is half the value of the remaining bits
  quantizeoffset = 0x0000800 >> (knob1 >> 7);
  quantizedcv += quantizeoffset;

  //   Use knob2 as an adder only if we aren't using the LFO in alt mode
  knobadd = normaltswitch ? knob2 << 2 : 0;
  dacout = quantizedcv + knobadd;
  if (dacout > 0x0FFF) dacout = 0x0FFF;

  //     OUT1 triggers if we had a change in the output;
  if (triggered || (olddacout != dacout) )
  {
    digout = 1;
  }
  else
  {
    digout = 0;
  }
  olddacout = dacout;
  return;
}
#endif

#define ARPEGGITRON
#ifdef ARPEGGITRON
//
//     Arpeggitron
//       NORM -  CVin is the base pitch;
//         KNOB1 controls the increment between arpeggiated notes.
//         KNOB2 controls how many notes in the arpeggiation.
//         TRIG steps from one note to the next.
//       ALT mode - CVin is base pitch, KNOB1 controls increment,
//         KNOB2 is an LFO that acts like TRIG (which still works!  Try ratcheting!).
//         Number of notes in the arpeggiation is what you set in NORMAL mode.
//      PB1 - switches the arpeggiation from up, to down, to pendulum, to random(ish).

void arpeggitron ()
{
  static long pitch_incr, pitch_count, now_count;
  static int arp_dir;
  static long lfo_accum;
  static long unsigned internallfo;
#define MAX_ARPEGGIOMODES 4
  static int arpeggiomode;

  //   pitch_incr always changes...
  pitch_incr = knob1;

  //  normaltswitch - switch between note counts or using a local LFO to trigger
  if (normaltswitch)
  {
    pitch_count = knob2 >> 5;  //  up to a 32-note arpeggio!
  }
  else
  {
    internallfo += (pgm_read_dword_near (exp_table + (knob2) ));
    if (internallfo > 10000000)
    {
      internallfo = 0;
      triggered = 1;
    }
  }

  if (pushed)  arpeggiomode++;
  if (arpeggiomode > MAX_ARPEGGIOMODES) arpeggiomode = 0;

  //     If we're not triggered, we do nothing!
  if (!triggered)
  {
    digout = 0;
    return;
  }
  //     We are triggered so we select based on arpeggiomode.
  if (arpeggiomode == 0)  // 0 is arp-up
  {
    now_count++;
    if (now_count > pitch_count) now_count = 0;
    dacout = (cvin << 2) + (now_count * pitch_incr);
  }
  if (arpeggiomode == 1)  // 1 is arp-down
  {
    now_count--;
    if (now_count < 0) now_count = pitch_count;
    dacout = (cvin << 2) + (now_count * pitch_incr);
  }
  if (arpeggiomode == 2)  //  2 is arp-pendulum
  {
    if (arp_dir > 1) arp_dir = 1;
    if (arp_dir < -1) arp_dir = -1;
    if (arp_dir == 0) arp_dir = 1;
    now_count = now_count + arp_dir;
    if (now_count > pitch_count) {
      arp_dir = -1;
      now_count = pitch_count;
    };
    if (now_count < 0) {
      arp_dir = +1;
      now_count = 0;
    }
    dacout = (cvin << 2) + (now_count * pitch_incr);
  }
  if (arpeggiomode == 3)  //  3 is fairly random
  {
    now_count = (ticker ^ ( ticker >> 5) ^ (ticker >> 11)) % pitch_count;
    dacout = (cvin << 2) + (now_count * pitch_incr);
  }
  if (arpeggiomode == 4)  //  4 is semirandom textured
  {
    now_count = ticker % pitch_count;
    dacout = (cvin << 2) + (now_count * pitch_incr);
  }
  digout = 1;
}
#endif

#ifdef COPYME
{
  increment = (analogRead (KNOB1) << (scaleshift)) + (analogRead(KNOB2));
  phaseaccum += increment;
  phaseaccum = phaseaccum & phasemax;
  phaseout =  phaseaccum >> scaleshift;
  phaseout = phaseout & 0x00000FFFL;
  dacout = pgm_read_word_near (sinetable + phaseout); // note C-style punning an array name)
  PORTB = (PORTB & 0xF0) + ((dacout >> 8) & 15);
  PORTD = dacout & 255;
  PORTC = PORTC & ~0x02 | (phaseout > 2000L) ? 0 : 2;
}
#endif


ISR(TIMER0_COMPA_vect) { //interrupt routine
  //   Ticker just keeps... on... ticking.   Highly useful.
  ticker++;

  //    Switch to the proper EON E-state function mode.based
  //    on the E-O-N switch analog value
  if ( eon > 950 )
  {
    estate = 0;
  }
  else
  {
    if (eon < 100)
    {
      estate = 1;
    }
    else
    {
      estate = 2;
    }
  }
  //    Now, branch on estate
  //    Each function MUST set the desired values of the globals
  //    named dacout and digout.
  if (estate == 0)  // "E" mode   0-5 volts in hardware
  {
    //    Do the sample/quantize/slide/hold
    samp_quant_slide_hold ();
  }
  if (estate == 2)   // "N" mode - balanced +-2.5V
  {
    //dacout = 4095 - ticker;
    if (normaltswitch)
    {
      supersineoscillator();
    }
    else
    {
      supersawoscillator();
    }
    digout = dacout > 0x7FF;

  }
  if (estate == 1)   // "O" mode - 0-5 volts in hardware
  {
    //dacout = ticker<<5;
    //digout = dacout > 0x7FF;
    //    Do an arpeggitron
    arpeggitron();
  }

  // Turn this on to calibrate the top pot and switches
  // by PWMing OUT1 (top jack) and the yellow LED.
  //#define CALIBRATION_TESTING
#ifdef CALIBRATION_TESTING
  dacout = eon << 2;
  digout = dacout > (0x3FF & ticker);
#endif

  //   Do the actual output to the R-2R ladder:
  PORTB = (PORTB & 0xF0) + ((dacout >> 8) & 15);
  // 
  PORTD = dacout & 255;
  //   Do the actual write to the OUT1 data latch:
  //digitalWrite (OUT1, digout);
  //PORTC = (PORTC & ~0x02) | ((digout > 0) ? 0x02 : 0x0);
  if (digout) {
    sbi(PORTC, 1);
  }
  else {
    cbi(PORTC, 1);
  }
}
