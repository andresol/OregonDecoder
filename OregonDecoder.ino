// Oregon V2 decoder added - Dominique Pierre
// Oregon V3 decoder revisited - Dominique Pierre
// New code to decode OOK signals from weather sensors, etc.
// 2010-04-11 <jcw@equi4.com> http://opensource.org/licenses/mit-license.php
// $Id: ookDecoder.pde 5331 2010-04-17 10:45:17Z jcw $


int UVNow=-999;
int UVBat=100;
int MaxUV24=-999;
char data[25];
long pos;
int OutTempNow=-999;
int OutHumNow=-999;
int GustNow=-999;
int AverageNow=-999;
int DirectionNow=-999;
int BarrometerNow=-999;
//String Forecast="";
int InTempNow=-999;
int InHumNow=-999;
String Comfort="";
String Forecast="";
int CH1TempNow=-999;
int CH1HumNow=-999;
int CH2TempNow=-999;
int CH2HumNow=-999;
int CH3TempNow=-999;
int CH3HumNow=-999;
int RainRateNow=-999;
int TotalRainFrom0000=0;
int TotalRainHour=0;
int PowerNow=0;
int RainDaysMonth=-999;
int RainTotalMonth=-999;
int OutTempBat=100;
int WindBat=100;
int RainBat=100;
int InTempBat=100;
int CH1Bat=100;
int CH2Bat=100;
int CH3Bat=100;
float PF=0.71;
float TotalPower24 = 0;
float TotalPowerHr = 0;
int MaxTemp24=-999;
int MinTemp24=999;
int MaxGust24=-999;
int Direction24=-999;
int MaxPower24=-999;
long PreviousTime;
int TempTriggerMax=150; //temp * 10
int TempTriggerMin=0;
int WindTrigger=15; //mph
boolean WindTriggerFlag = false;
boolean TempTriggerMaxFlag = false;
boolean TempTriggerMinFlag = false;

int OldRainTotal=0;
boolean WriteFlag = false;
boolean PowerFlag = false;
boolean Extreme24Flag = false;
boolean ExtremeYearFlag = false;
boolean RainNewFlag = true;
int celsius = 0;
const char* lastValue = "";


// Grab nibbile from packet and reverse it
byte GetNibble(int nibble, const byte packet[])
{
  int pos = nibble / 2;
  int nib = nibble % 2;
  byte b = packet[pos];
  if (nib == 1)
    b = (byte)((byte)(b) >> 4);
  else
    b = (byte)((byte)(b) & (byte)(0x0f));

  return b;
}

// Reverse the bits
byte Reverse(byte b) 
{ 
  int rev = (b >> 4) | ((b & 0xf) << 4); 
  rev = ((rev & 0xcc) >> 2) | ((rev & 0x33) << 2);
  rev = ((rev & 0xaa) >> 1) | ((rev & 0x55) << 1); 
  return (byte)rev; 
}

class DecodeOOK {
protected:
  byte bits, flip, state, pos, data[25];
  int counter;
  virtual char decode (word width) =0;

public:
 byte total_bits;
  enum { 
    UNKNOWN, T0, T1, T2, T3, OK, DONE   };

  DecodeOOK () { 
    resetDecoder(); 
  }

  bool nextPulse (word width) {
    if (state != DONE)

      switch (decode(width)) {
      case -1: 
        resetDecoder(); 
        break;
      case 1:  
        done(); 
        break;
      }
    return isDone();
  }

  bool isDone () const { 
    return state == DONE; 
  }

  const byte* getData (byte& count) const {
    count = pos;
    return data; 
  }

  void resetDecoder () {
    total_bits = bits = pos = flip = counter = 0 ;
    state = UNKNOWN;
  }

  // add one bit to the packet data buffer

  virtual void gotBit (char value) {
    total_bits++;
    byte *ptr = data + pos;
    *ptr = (*ptr >> 1) | (value << 7);

    if (++bits >= 8) {
      bits = 0;
      if (++pos >= sizeof data) {
        resetDecoder();
        return;
      }
    }
    state = OK;
  }

  // store a bit using Manchester encoding
  void manchester (char value) {
    flip ^= value; // manchester code, long pulse flips the bit
    gotBit(flip);
  }

  // move bits to the front so that all the bits are aligned to the end
  void alignTail (byte max =0) {
    // align bits
    if (bits != 0) {
      data[pos] >>= 8 - bits;
      for (byte i = 0; i < pos; ++i)
        data[i] = (data[i] >> bits) | (data[i+1] << (8 - bits));
      bits = 0;
    }
    // optionally shift bytes down if there are too many of 'em
    if (max > 0 && pos > max) {
      byte n = pos - max;
      pos = max;
      for (byte i = 0; i < pos; ++i)
        data[i] = data[i+n];
    }
  }

  void reverseBits () {
    for (byte i = 0; i < pos; ++i) {
      byte b = data[i];
      for (byte j = 0; j < 8; ++j) {
        data[i] = (data[i] << 1) | (b & 1);
        b >>= 1;
      }
    }
  }

  void reverseNibbles () {
    for (byte i = 0; i < pos; ++i)
      data[i] = (data[i] << 4) | (data[i] >> 4);
  }

  void done () {
    while (bits)
      gotBit(0); // padding
    state = DONE;
  }
};

// 433 MHz decoders


class OregonDecoderV2 : 
public DecodeOOK {
public:
  OregonDecoderV2() {
  }

  // add one bit to the packet data buffer
  virtual void gotBit (char value) {
    if(!(total_bits & 0x01))
    {
      data[pos] = (data[pos] >> 1) | (value ? 0x80 : 00);
    }
    total_bits++;
    pos = total_bits >> 4;
    if (pos >= sizeof data) {
      resetDecoder();
      return;
    }
    state = OK;
  }

  virtual char decode (word width) {
    if (200 <= width && width < 1200) {
      byte w = width >= 700;
      switch (state) {
      case UNKNOWN:
        if (w != 0) {
          // Long pulse
          ++flip;
        } 
        else if (32 <= flip) {
          // Short pulse, start bit
          flip = 0;
          state = T0;
        } 
        else {
          // Reset decoder
          return -1;
        }
        break;
      case OK:
        if (w == 0) {
          // Short pulse
          state = T0;
        } 
        else {
          // Long pulse
          manchester(1);
        }
        break;
      case T0:
        if (w == 0) {
          // Second short pulse
          manchester(0);
        } 
        else {
          // Reset decoder
          return -1;
        }
        break;
      }
    } 
    else {
      return -1;
    }
    return total_bits == 160 ? 1: 0;
  }
};


class OregonDecoderV3 : 
public DecodeOOK {
public:
  OregonDecoderV3() {
  }

  // add one bit to the packet data buffer
  virtual void gotBit (char value) {
    data[pos] = (data[pos] >> 1) | (value ? 0x80 : 00);
    total_bits++;
    pos = total_bits >> 3;
    if (pos >= sizeof data) {
      resetDecoder();
      return;
    }
    state = OK;
  }

  virtual char decode (word width) {
    if (200 <= width && width < 1200) {
      byte w = width >= 700;
      switch (state) {
      case UNKNOWN:
        if (w == 0)
          ++flip;
        else if (32 <= flip) {
          flip = 1;
          manchester(1);
        } 
        else
          return -1;
        break;
      case OK:
        if (w == 0)
          state = T0;
        else
          manchester(1);
        break;
      case T0:
        if (w == 0)
          manchester(0);
        else
          return -1;
        break;
      }
    } 
    else {
      return -1;
    }
    return  total_bits == 80 ? 1: 0;
  }
};

class CrestaDecoder : 
public DecodeOOK {
public:
  CrestaDecoder () {
  }

  virtual char decode (word width) {
    if (200 <= width && width < 1300) {
      byte w = width >= 750;
      switch (state) {
      case UNKNOWN:
        if (w == 1)
          ++flip;
        else if (2 <= flip && flip <= 10)
          state = T0;
        else
          return -1;
        break;
      case OK:
        if (w == 0)
          state = T0;
        else
          gotBit(1);
        break;
      case T0:
        if (w == 0)
          gotBit(0);
        else
          return -1;
        break;
      }
    } 
    else if (width >= 2500 && pos >= 7) 
      return 1;
    else
      return -1;
    return 0;
  }
};

class KakuDecoder : 
public DecodeOOK {
public:
  KakuDecoder () {
  }

  virtual char decode (word width) {
    if (180 <= width && width < 450 || 950 <= width && width < 1250) {
      byte w = width >= 700;
      switch (state) {
      case UNKNOWN:
      case OK:
        if (w == 0)
          state = T0;
        else
          return -1;
        break;
      case T0:
        if (w)
          state = T1;
        else
          return -1;
        break;
      case T1:
        state += w + 1;
        break;
      case T2:
        if (w)
          gotBit(0);
        else
          return -1;
        break;
      case T3:
        if (w == 0)
          gotBit(1);
        else
          return -1;
        break;
      }
    } 
    else if (width >= 2500 && 8 * pos + bits == 12) {
      for (byte i = 0; i < 4; ++i)
        gotBit(0);
      alignTail(2);
      return 1;
    } 
    else
      return -1;
    return 0;
  }
};

class XrfDecoder : 
public DecodeOOK {
public:
  XrfDecoder () {
  }

  // see also http://davehouston.net/rf.htm
  virtual char decode (word width) {
    if (width > 2000 && pos >= 4)
      return 1;
    if (width > 5000)
      return -1;
    if (width > 4000 && state == UNKNOWN)
      state = OK;
    else if (350 <= width && width < 1800) {
      byte w = width >= 720;
      switch (state) {
      case OK:
        if (w == 0)
          state = T0;
        else
          return -1;
        break;
      case T0:
        gotBit(w);
        break;
      }
    } 
    else
      return -1;
    return 0;
  }
};

class HezDecoder : 
public DecodeOOK {
public:
  HezDecoder () {
  }

  // see also http://homeeasyhacking.wikia.com/wiki/Home_Easy_Hacking_Wiki
  virtual char decode (word width) {
    if (200 <= width && width < 1200) {
      byte w = width >= 600;
      gotBit(w);
    } 
    else if (width >= 5000 && pos >= 5 /*&& 8 * pos + bits == 50*/) {
      for (byte i = 0; i < 6; ++i)
        gotBit(0);
      alignTail(7); // keep last 56 bits
      return 1;
    } 
    else
      return -1;
    return 0;
  }
};

class NexaDecoder : 
public DecodeOOK { 
  private:
    byte prevBit;
    int i;
public:
  NexaDecoder () {
    prevBit = 0;
    i = 0;
  }

  virtual char decode (word width) {
    if (200 <= width && width < 1800) {
      byte w = width <= 900;
      if (i % 2 == 1) {
         if ((prevBit + w) != 1) { // Error check. Must Be 01 or 10
                i = 0;
                //return -1;
            }
       gotBit(w);
      }
      prevBit = w;
      i++;
    } 
    //else if (width >= 5000 && pos >= 5 /*&& 8 * pos + bits == 50*/) {
      else if (width >= 5000 && pos == 12 ) {
        //for (byte i = 0; i < 6; ++i)
      //    gotBit(0);
      //alignTail(7); // keep last 56 bits
      return 1;
    } 
    else if (width >= 1800) {
      state = OK;
    } 
    else
      return -1;
    return 0;
  }
};

/**

  Message is sent 7 times in repeate. 36x9. 
  9 nibles: 
  
  95 40 0e 3c c : 10010101 01000000 00001110 00111100 1100
  1001: Always 1001. ID for sensor 9
  0101 0100 = RandomID 5 4
  0000: 0 battery. 1 ok, 1 pressed button, 00= channel from 1-3 
  temp: 12 bits. 0e3 = 227 = 22.7C if 1e3 = -22.7c
  hum0: 1100 = no humidity c
  hum1: 1100 = no humidity  c
*/
class Prologue : 
public DecodeOOK { 
  private:
    byte prevBit;
    int i;
public:
  Prologue () {
    prevBit = 0;
    i = 0;
  }
  
   virtual void gotBit (char value) {
    total_bits++;
    bitWrite(data[pos], 7-bits, value);
    if (++bits >= 8) {
      bits = 0;
      if (++pos >= sizeof data) {
        resetDecoder();
        return;
      }
    }
    state = OK;
  }

  virtual char decode (word width) {
    if (1800 <= width && width < 4200) {
      if (width > 1900 && width < 2100) {
        gotBit(0);
      } else if (width > 3800  && width < 4000) {
        gotBit(1);
      }
    } else if (width >= 8000 && width <= 11000 && 8 * pos + bits == 37) {
      return 1;
    } else if (width >= 300 && width < 1000) {
      state = OK;
    } else {
      return -1;
    }
    return 0;
  }
};

// 868 MHz decoders

class VisonicDecoder : 
public DecodeOOK {
public:
  VisonicDecoder () {
  }

  virtual char decode (word width) {
    if (200 <= width && width < 1000) {
      byte w = width >= 600;
      switch (state) {
      case UNKNOWN:
      case OK:
        state = w == 0 ? T0 : T1;
        break;
      case T0:
        gotBit(!w);
        if (w)
          return 0;
        break;
      case T1:
        gotBit(!w);
        if (!w)
          return 0;
        break;
      }
      // sync error, flip all the preceding bits to resync
      for (byte i = 0; i <= pos; ++i)
        data[i] ^= 0xFF; 
    } 
    else if (width >= 2500 && 8 * pos + bits >= 36 && state == OK) {
      for (byte i = 0; i < 4; ++i)
        gotBit(0);
      alignTail(5); // keep last 40 bits
      // only report valid packets
      byte b = data[0] ^ data[1] ^ data[2] ^ data[3] ^ data[4];
      if ((b & 0xF) == (b >> 4))
        return 1;
    } 
    else
      return -1;
    return 0;
  }
};

class EMxDecoder : 
public DecodeOOK {
public:
  EMxDecoder () {
  }

  // see also http://fhz4linux.info/tiki-index.php?page=EM+Protocol
  virtual char decode (word width) {
    if (200 <= width && width < 1000) {
      byte w = width >= 600;
      switch (state) {
      case UNKNOWN:
        if (w == 0)
          ++flip;
        else if (flip > 20)
          state = OK;
        else
          return -1;
        break;
      case OK:
        if (w == 0)
          state = T0;
        else
          return -1;
        break;
      case T0:
        gotBit(w);
        break;
      }
    } 
    else if (width >= 1500 && pos >= 9)
      return 1;
    else
      return -1;
    return 0;
  }
};

class KSxDecoder : 
public DecodeOOK {
public:
  KSxDecoder () {
  }

  // see also http://www.dc3yc.homepage.t-online.de/protocol.htm
  virtual char decode (word width) {
    if (200 <= width && width < 1000) {
      byte w = width >= 600;
      switch (state) {
      case UNKNOWN:
        gotBit(w);
        bits = pos = 0;
        if (data[0] != 0x95)
          state = UNKNOWN;
        break;
      case OK:
        state = w == 0 ? T0 : T1;
        break;
      case T0:
        gotBit(1);
        if (!w)
          return -1;
        break;
      case T1:
        gotBit(0);
        if (w)
          return -1;
        break;
      }
    } 
    else if (width >= 1500 && pos >= 6) 
      return 1;
    else
      return -1;
    return 0;
  }
};

class FSxDecoder : 
public DecodeOOK {
public:
  FSxDecoder () {
  }

  // see also http://fhz4linux.info/tiki-index.php?page=FS20%20Protocol
  virtual char decode (word width) {
    if (300 <= width && width < 775) {
      byte w = width >= 500;
      switch (state) {
      case UNKNOWN:
        if (w == 0)
          ++flip;
        else if (flip > 20)
          state = T1;
        else
          return -1;
        break;
      case OK:
        state = w == 0 ? T0 : T1;
        break;
      case T0:
        gotBit(0);
        if (w)
          return -1;
        break;
      case T1:
        gotBit(1);
        if (!w)
          return -1;
        break;
      }
    } 
    else if (width >= 1500 && pos >= 5)
      return 1;
    else
      return -1;
    return 0;
  }
};

OregonDecoderV2 orscV2;
OregonDecoderV3 orscV3;
CrestaDecoder cres;
KakuDecoder kaku;
XrfDecoder xrf;
HezDecoder hez;
NexaDecoder nexa;
VisonicDecoder viso;
EMxDecoder emx;
KSxDecoder ksx;
FSxDecoder fsx;
Prologue pro;

#define PORT 2

volatile word pulse;

void rupt (void) {

  static word last;
  // determine the pulse length in microseconds, for either polarity
  pulse = micros() - last;
  last += pulse;
}

void reportSerial (const char* s, class DecodeOOK& decoder) {
  byte pos;
  unsigned int watts;
  const byte* data = decoder.getData(pos);
  Serial.print(s);
  lastValue = s;
  Serial.print(" Pos:");
  Serial.print(pos);
  Serial.print(' ');
  Serial.print(" Total:");
  Serial.print(decoder.total_bits);
  Serial.print(' ');
  for (byte i = 0; i < pos; ++i) {
     Serial.print(data[i] >> 4, HEX);
    //Serial.print(GetNibble(i, data), HEX);
    //Serial.print(GetNibble(i+1, data), HEX);
    //Serial.print(data[i]);
    Serial.print(data[i] & 0x0F, HEX);
    Serial.print(' ');
  }
  //if (pos > 5) {
  //  Serial.print('\t');
  //  watts = ((data[4] * 256) + (data[3] & 0xF0)) * 1.006196884; 
  //  Serial.print(watts);
  //}
  // Serial.print(' ');
  // Serial.print(millis() / 1000);

  Serial.println();
  //General note where possible, values multiplied by 10 to give integers to 1 dcimal point when divided by 10 later.  Less memory hungry.

  // OWL Electricty Meter
  //  if (data[0] == 0x06 && data[1] == 0xC8) {
  //    Serial.print("Current ");
  //    PowerNow = (data[3] + ((data[4] & 0x03)*256));
  //    Serial.print(float(PowerNow)/10,1);
  //    Serial.println("amps"); 
  //    float ActualPower = Duration * ((PowerNow * 240 * PF)/36000.0);
  //    if (ActualPower < 0) ActualPower = -ActualPower;
  //    if (ActualPower > 0) {  
  //      TotalPowerHr += ActualPower; //W/s
  //      TotalPower24 += ActualPower; //W/s
  //      YearData.TotalPowerY += ActualPower;
  //      PreviousTime = now.unixtime();
  //      PowerTime = now.unixtime();
  //     Serial.print("Added... ");
  //      Serial.print(ActualPower);
  //      Serial.print("W (over ");
  //    }
  //    Serial.print(Duration);
  //    Serial.print(" seconds). Total Today ");
  //    Serial.print(TotalPower24,0);
  //    Serial.println("W/hs ");
  //Check Extremes
  //    if (PowerNow > MaxPower24 && PowerNow != -999) {
  //      MaxPower24 = PowerNow;
  //    }
  //  }

  //UV138 is 0xEA && 0X7C

  if (data[0] == 0xEA && data[1] == 0x7C) {
    Serial.print("UV ");
    UVNow = ((data[5] & 0x0F) * 10)  + (data[4] >> 4);

    Serial.print(UVNow);
    Serial.print("  Battery ");
    if ((data[4] & 0x0F) >= 4){
      UVBat=0;
      Serial.println("Low"); 
    }
    else
    {
      UVBat=100;
      Serial.println("OK"); 
    }  
    //Check Extremes
    if (UVNow > MaxUV24 && UVNow != -999) {
      MaxUV24 = UVNow;
    }    
  }    
  decoder.resetDecoder();




  //(THGR228N, THGR122X,  )Inside Temp-Hygro


  if (data[0] == 0x1A && data[1] == 0x2D || data[0] == 0x1D && data[1] == 0x20 || data[0] == 0xFA && data[1] == 0x28) 

  {

    int battery=0;
    celsius= ((data[5]>>4) * 100)  + ((data[5] & 0x0F) * 10) + ((data[4] >> 4)); //11..8 11 is flag bit. 10..8 temp.
    if ((data[6] & 0x0F) >= 8) celsius=-celsius;
    int hum = ((data[7] & 0x0F)*10)+ (data[6] >> 4);
    if ((data[4] & 0x0F) == 4)
    {
      battery=0;
    }
    else
    {
      battery=100;
    }   
    Serial.print("THGR228N found ");

    switch (data[2] & 0x0F) { //Nibbel 5
    case 1:
      CH1TempNow=celsius;
      CH1HumNow=hum;
      CH1Bat=battery;

      Serial.print("CH1  ");
      break;
    case 2:
      CH2TempNow=celsius;
      CH2HumNow=hum;
      CH2Bat=battery;

      Serial.print("CH2  ");
      break;
    case 4:
      CH3TempNow=celsius;
      CH3HumNow=hum;
      CH3Bat=battery;

      Serial.print("CH3  ");
      break;
    } 
    Serial.print ("ID") ;
    Serial.print(data[2] >> 4); 
    Serial.print(data[3] & 0x0F); 
    Serial.print (" ") ;
    //Serial.print(" ");
    //Serial.print(data[2]);
    //Serial.print(" ");
    //Serial.print("Temp: ");
    Serial.print(float(celsius)/10,1);
    //Serial.println(" c");
    //Serial.print(hum);
    // Serial.print("Temp: ");
    // Serial.print(float(CH3TempNow)/10,1);
    Serial.print("C  Humidity ");
    Serial.print(CH3HumNow);
    Serial.print("%  Battery ");  
    Serial.print(battery);
    Serial.println("%");      
  }


  //THN122N Inside Temp


  if (data[0] == 0xEA && data[1] == 0x4c) 

  {

    int battery=0;
    int celsius= ((data[5]>>4) * 100)  + ((data[5] & 0x0F) * 10) + ((data[4] >> 4));
    if ((data[6] & 0x0F) >= 8) celsius=-celsius;

    if ((data[4] & 0x0F) == 4)
    {
      battery=0;
    }
    else
    {
      battery=100;
    }   
    Serial.print("THN122N found ");
    switch (data[2]) {
    case 0x10:
      CH1TempNow=celsius;
      CH1Bat=battery;

      Serial.print("CH1  ");
      break;
    case 0x20:
      CH2TempNow=celsius;
      CH2Bat=battery;

      Serial.print("CH2  ");
      break;
    case 0x40:
      CH3TempNow=celsius;
      CH3Bat=battery;

      Serial.print("CH3  ");
      break;
    }    
    Serial.print ("ID") ;
    Serial.print(data[2] >> 4); 
    Serial.print(data[3] & 0x0F); 
    Serial.print (" ") ;
    Serial.print(float(celsius)/10,1);
    Serial.print("%  Battery ");  
    Serial.print(battery);
    Serial.println("%");      
  }


  //  Annometer
  if (data[0] == 0x3A && data[1] == 0x0D || data[0] == 0x19 && data[1] == 0x84 || data[0] == 0x19 && data[1] == 0x94 || data[0] == 0x1A && data[1] == 0x89) 
  {
    //Checksum - add all nibbles from 0 to 8, subtract A and compare to byte 9, should = 0
    int cs = 0;
    for (byte i = 0; i < pos-1; ++i) 
    { 
      //all but last byte
      cs += data[i] >> 4;
      cs += data[i] & 0x0F;
    }
    int csc = ((data[9] >> 4)*16) + (data[9] & 0x0F);
    cs -= 10;
    Serial.print("Annometer found ");
    //    Serial.print(csc); 
    //    Serial.print(" vs ");   
    //    Serial.println(cs); 
    if (cs == csc)
    { 
      //if checksum is OK
      Serial.print ("ID") ;
      Serial.print(data[2] >> 4); 
      Serial.print(data[3] & 0x0F); 
      Serial.print (" ") ;   
      Serial.print("Direction ");
      DirectionNow = ((data[4]>>4) * 22.5);
      //DirectionNow = GetNibble(9, data) * 22.5;
      Serial.print(DirectionNow);
      Serial.print(" degrees  Current Speed (Gust) ");
      GustNow = ((data[7] & 0x0F) * 100)  + ((data[6]>>4) * 10)  + ((data[6] & 0x0F)) ;
      Serial.print(float(GustNow)/10,1);   
      Serial.print("m/s  Average Speed ");
      AverageNow = ((data[8]>>4) * 100)  + ((data[8] & 0x0F) * 10)+((data[7] >>4)) ;      
      Serial.print(float(AverageNow)/10,1); 
      Serial.print("m/s  Battery ");      
      WindBat=(10-(data[4] & 0x0F))*10;
      Serial.print(WindBat);
      Serial.println("%");

    }  
  }


  // Rain Guage 
  if (data[0] == 0x2A && data[1] == 0x1D || data[0] == 0x2D && data[1] == 0x10 || data[0] == 0x29 && data[1] == 0x14 || data[0] == 0x2A && data[1] == 0x19) 
  {
    //Checksum - add all nibbles from 0 to 8, subtract 9 and compare, should = 0
    //    Serial.print(" - ");
    //   int cs = 0;
    //    for (byte i = 0; i < pos-1; ++i) 
    //    { 
    //all but last byte
    //        cs += data[i] >> 4;
    //        cs += data[i] & 0x0F;
    //    }
    //    int csc = (data[8] >> 4) + ((data[9] & 0x0F));    
    //    cs -= 10;      //my version as A fails?
    //    Serial.print(csc); 
    //    Serial.print(" vs ");   
    //    Serial.println(cs);
    //    if (cs == csc)
    //   { 
    //if checksum is OK      
    Serial.print("Rain Guage found ");
    Serial.print ("ID") ;
    Serial.print(data[2] >> 4); 
    Serial.print(data[3] & 0x0F);
    Serial.print (" ") ;
    Serial.print("Rain ");
    RainRateNow = ((data[5]>>4) * 100)  + ((data[5] & 0x0F) * 10) + (data[4] >> 4);
    Serial.print(RainRateNow);
    Serial.print("mm/hr  Total ");
    int RainTotal = ((data[7]  >> 4) * 10)  + (data[6]>>4);

    //     if (RainTotal != OldRainTotal)
    //     {
    //        if (RainNewFlag == false)
    //        {  
    //Stops 1st reading going through and giving additonal values
    //          TotalRainFrom0000 += 1;
    //          TotalRainHour += 1;
    //          YearData.TotalRainY += 1;
    //          SendEmail(2);
    //        }
    //       OldRainTotal=RainTotal;
    //       RainNewFlag=false;
    //      }
    Serial.print(TotalRainFrom0000);   
    Serial.print(" ");
    Serial.print(RainTotal); 
    Serial.print(" ");
    Serial.print(OldRainTotal); 
    Serial.print("mm  Battery ");
    if ((data[4] & 0x0F) >= 4)
    {
      RainBat=0;
      Serial.println("Low"); 
    }
    else
    {
      RainBat=100;
      Serial.println("OK"); 
    }          
    //   }
  } 



  //(THGR918, THGR810v1, THGR810 ) Outside Temp-Hygro
  if (data[0] == 0x1A && data[1] == 0x3D|| data[0] == 0x3A && data[1] == 0x0D || data[0] == 0xF8 && data[1] == 0x24 || data[0] == 0x1D && data[1] == 0x20) 
  {
    //Checksum - add all nibbles from 0 to 8, subtract 9 and compare, should = 0
    Serial.print(" - ");
    int cs = 0;
    for (byte i = 0; i < pos-2; ++i) 
    { 
      //all but last byte
      cs += data[i] >> 4;
      cs += data[i] & 0x0F;
    }
    int csc = ((data[8] >> 4)*16) + (data[8] & 0x0F);    
    cs -= 10; 
    Serial.print(csc); 
    Serial.print(" vs ");   
    Serial.println(cs);
    if (cs == csc)
    { 
      //if checksum is OK 
      Serial.print("Outdoor temperature ");
      OutTempNow= ((data[5]>>4) * 100)  + ((data[5] & 0x0F) * 10) + ((data[4] >> 4));
      if ((data[6] & 0x0F) >= 8) OutTempNow=-OutTempNow;
      Serial.print ("ID") ;
      Serial.print(data[2] >> 4); 
      Serial.print(data[3] & 0x0F); 
      Serial.print (" ") ;
      Serial.print(float(OutTempNow)/10,1);
      Serial.print("C  Humidity ");
      OutHumNow = ((data[7] & 0x0F)*10)+ (data[4] >> 4);
      Serial.print(OutHumNow);
      Serial.print("%  Battery ");
      OutTempBat=(10-(data[4] & 0x0F))*10;
      Serial.print(OutTempBat);
      Serial.println("%");
      //      OutTime = now.unixtime();

    }
  }



  //(BTHR918, BTHR968) Temp-Hygro-Baro
  if (data[0] == 0x5A && data[1] == 0x5D || data[0] == 0x5D && data[1] == 0x60  || data[0] == 0x5A && data[1] == 0x6D) 
  {
    Serial.print("Indoor temperature ");
    Serial.print ("ID") ;
    Serial.print(data[2] >> 4); 
    Serial.print(data[3] & 0x0F); 
    Serial.print (" ") ;
    InTempNow= ((data[5]>>4) * 100)  + ((data[5] & 0x0F) * 10) + ((data[4] >> 4));
    if ((data[6] & 0x0F) >= 8) InTempNow=-InTempNow;
    Serial.print(float(InTempNow)/10,1);
    Serial.print("C  Humidity ");
    InHumNow = ((data[7] & 0x0F)*10)+ (data[6] >> 4);
    Serial.print(InHumNow);
    Serial.print("%  Pressure ");
    BarrometerNow = (data[8])+856;
    Serial.print(BarrometerNow);
    Serial.print("hPa  ");
    switch (data[7] & 0xC0) 
    {
    case 0x00:
      Comfort="Normal";
      break;
    case 0x40:
      Comfort="Comfortable";
      break;
    case 0x80:
      Comfort="Dry";
      break;
    case 0xC0:
      Comfort="Wet";
      break;
    }
    Serial.print(Comfort);
    Serial.print("  ");
    switch (data[9] >> 4) 
    {
    case 0x0C:
      Forecast="Sunny";
      break;
    case 0x06:
      Forecast="Partly Cloudy";
      break;
    case 0x02:
      Forecast="Cloudy";
      break;
    case 0x03:
      Forecast="Wet";
      break;
    }     

    Serial.print(Forecast);
    Serial.print("  Battery ");               
    InTempBat=(10-(data[4] & 0x0F))*10;
    Serial.print(InTempBat);
    Serial.println("%");      
  }  
  decoder.resetDecoder();
}



void setup () {
  Serial.begin(9600);
  Serial.println("\n[Starting]");
  pinMode(3, OUTPUT);
  digitalWrite(3,HIGH);
  pinMode(PORT, INPUT);  // use
  attachInterrupt (0, rupt, CHANGE);

}

void loop () {
  static int i = 0;
  noInterrupts();
  word p = pulse;

  pulse = 0;
  interrupts();

  //if (p != 0){ Serial.print(++i); Serial.print('\n');}

  if (p != 0) {
    if (orscV2.nextPulse(p))
      reportSerial("OSV2", orscV2);  
    if (orscV3.nextPulse(p))
      reportSerial("OSV3", orscV3);        
    if (cres.nextPulse(p))
      reportSerial("CRES", cres);        
    if (kaku.nextPulse(p))
      reportSerial("KAKU", kaku);        
    if (xrf.nextPulse(p))
      reportSerial("XRF", xrf);
    if (nexa.nextPulse(p))
        reportSerial("NEXA", nexa);        
    if (hez.nextPulse(p))
      reportSerial("HEZ", hez);
    if (viso.nextPulse(p))
      reportSerial("VISO", viso);        
    if (emx.nextPulse(p))
      reportSerial("EMX", emx);        
    if (ksx.nextPulse(p))
      reportSerial("KSX", ksx);        
    if (fsx.nextPulse(p))
      reportSerial("FSX", fsx);   
    if (pro.nextPulse(p))
      reportSerial("PRO", pro);        
    //    }
  }

  //if (p != 0) {
  //    if (viso.nextPulse(p))
  //        reportSerial("VISO", viso);        
  //   if (emx.nextPulse(p))
  //      reportSerial("EMX", emx);        
  //   if (ksx.nextPulse(p))
  //       reportSerial("KSX", ksx);        
  //  if (fsx.nextPulse(p))
  //          reportSerial("FSX", fsx);        
  //    }

}
