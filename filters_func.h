#define CALIB_V 15.5360

#ifdef SENSOR_15A
  #define CALIB_C10X 9.9300 // SENSOR 15A 10.0430 (Sensor Anterior)
  #define CALIB_C5X  9.8120 // SENSOR 15A 9.8580 (Sensor Anterior)
  #define CALIB_C1X  9.9530 // SENSOR 15A 9.8600 (Sensor Anterior)  
#endif

#ifdef SENSOR_30A
  #define CALIB_C10X 13.7430 // SENSOR 30A
  #define CALIB_C5X  13.5120 // SENSOR 30A
  #define CALIB_C1X  13.9530 // SENSOR 30A
#endif


void samplesADCraw(int32_t* smp_Volt, int32_t* smp_Curr1, int32_t* smp_Curr5, int32_t* smp_Curr10);
void filterRemoveDC(int32_t* inFilterHP, int32_t* outFilterHP, const int32_t PRECISION);
void filterRemoveNoise(int32_t* inFilterLP, int32_t* outFilterLP, const int32_t PRECISION);
void filterZeroCross(int32_t* inFilterZC, int32_t* outFilterZC, const int32_t PRECISION);
bool zeroCrossing(int32_t* inSignal);
uint32_t calcFrequency(int32_t* inSignal, uint16_t sampleCount, const uint16_t FREQ_SAMPLE);
uint32_t calcRound(uint64_t value, uint16_t Exp);
int32_t voltageInst(int32_t* inSignal);
int32_t currentInst(int32_t* inSignal1, int32_t* inSignal5, int32_t* inSignal10, uint8_t adcGain);
uint8_t configADC(int32_t* inSignal1, int32_t* inSignal5, int32_t* inSignal10, bool zeroCross);
uint32_t calcVoltRms(uint32_t sumSqrVolt, uint16_t sampleCount);
uint64_t calcCurrRms(uint64_t sumSqrCurr, uint8_t adcGain, uint16_t sampleCount);
uint64_t calcPwrReal(int32_t sumPwr, uint8_t adcGain, uint16_t sampleCount);
uint32_t calcPwrApparent(uint32_t VoltRms, uint64_t CurrRms);
uint32_t calcPwrReative(uint32_t pwrReal, uint32_t pwrApparent);
uint32_t calcFactorPwr(uint32_t pwrReal, uint32_t pwrApparent);
uint32_t calcEneCunsumed(int32_t sumPwr, uint8_t adcGain, const uint16_t FREQ_SAMPLE);



void samplesADCraw(int32_t* smp_Volt, int32_t* smp_Curr1, int32_t* smp_Curr5, int32_t* smp_Curr10)
{
  smp_Volt[1] = smp_Volt[0];  
  smp_Curr1[1] = smp_Curr1[0];
  smp_Curr5[1] = smp_Curr5[0];
  smp_Curr10[1] = smp_Curr10[0];  
  
  // Samples ADC raw
  smp_Volt[0] = adc1_get_raw(ADC1_CHANNEL_7); // Voltage
  smp_Curr10[0] = adc1_get_raw(ADC1_CHANNEL_6); // Current10x
  smp_Curr5[0] = adc1_get_raw(ADC1_CHANNEL_3); // Current5x
  smp_Curr1[0] = adc1_get_raw(ADC1_CHANNEL_0); // Current1x
}



void filterRemoveDC(int32_t* inFilterHP, int32_t* outFilterHP, const int32_t PRECISION)
{
/* Filter IIR 1ª Order - Band Pass (Remove Level DC)
*
*   Fc = 1Hz
*   Fs = 4000Hz
*
*
*                1 - z^(-1)
*   H(z) =  G ---------------
*              1 - C1*z^(-1)
*          
*   y[n] = G(x[n] - x[n-1]) + C1*y[n-1]     
*          
*          
*/

  // Coefficients 
  const int16_t C1_FilterHP = 32717; 
  // Gain
  const int16_t G_FilterHP = 32742;
  
  // Filter HighPass Remove DC Level
  for (byte i=20; i>0; i--)
  {
    outFilterHP[i] = outFilterHP[i-1]; 
  }

  outFilterHP[0] = (G_FilterHP * (inFilterHP[0] -  inFilterHP[1]) + (C1_FilterHP * outFilterHP[1])) / PRECISION;       
}



void filterRemoveNoise(int32_t* inFilterLP, int32_t* outFilterLP, const int32_t PRECISION)
{
/* Filter FIR 20ª Oeder - Low Pass (Remove Noise)
* 
*   Fc = 1300Hz
*   Fs = 4000Hz
* 
* 
*             C1 + C2*z^(-1) + C2*z^(-2) + ... + C21*z^(-20)
*   H(z) =  -------------------------------------------------
*                                  1   
*              
*   y[n] = C1*x[n] + C2*x[n-1] + C2*x[n-2] + ... + C21*x[n-20]            
*              
*              
*/
  // Coefficients
  const int16_t COEF_FilterLP[21] =   {46, -42, -104, 318, -174, -675, 1503,
                    -410, -3721, 9001, 21283, 9001, -3721, -410,
                    1503, -675, -174, 318, -104, -42, 46};

  // Filter LowPass Remove Noise
  for (byte i=20; i>0; i--)
  {
    outFilterLP[i] = outFilterLP[i-1];
  }
  outFilterLP[0] = 0;

  for (byte i=0; i<10; i++)
  {
    outFilterLP[0] += COEF_FilterLP[i] * (inFilterLP[i] + inFilterLP[20-i]);
  }
  outFilterLP[0] += COEF_FilterLP[10] * inFilterLP[10];
  outFilterLP[0] /= PRECISION;
}



void filterZeroCross(int32_t* inFilterZC, int32_t* outFilterZC, const int32_t PRECISION)
{
/* Filter FIR 20ª Oeder - Low Pass (Fundamental Harmonics for Zero-Crossing)
* 
*   Fc = 50Hz
*   Fs = 4000Hz
* 
* 
*             C1 + C2*z^(-1) + C2*z^(-2) + ... + C21*z^(-20)
*   H(z) =  -------------------------------------------------
*                                  1   
*              
*   y[n] = C1*x[n] + C2*x[n-1] + C2*x[n-2] + ... + C21*x[n-20]            
*              
*              
*/
  // Coefficients
  const int16_t COEF_FilterZC[21] =   {337, 503, 720, 987, 1298, 1636, 1977,
                    2290, 2544, 2710, 2767, 2710, 2544, 2290,
                                        1977, 1636, 1298, 987, 720, 503, 337};

  // Filter LowPass Zero-Crossing
  for (byte i=20; i>0; i--)
  {
    outFilterZC[i] = outFilterZC[i-1];
  }
  outFilterZC[0] = 0;

  for (byte i=0; i<10; i++)
  {
    outFilterZC[0] += COEF_FilterZC[i] * (inFilterZC[i] + inFilterZC[20-i]);
  }
  outFilterZC[0] += COEF_FilterZC[10] * inFilterZC[10];
  outFilterZC[0] /= PRECISION;
}



boolean zeroCrossing(int32_t* inSignal)
{
  static bool zeroCrossPositive = false;

  // Zero-Crossing Voltage
  if (!zeroCrossPositive && (inSignal[0] >= 0))
  {
    zeroCrossPositive = true;
    return true;
  }
  else if (zeroCrossPositive && (inSignal[0] < 0))
  {
    zeroCrossPositive = false;
  }
  
  return false;
}



uint32_t calcFrequency(int32_t* inSignal, uint16_t sampleCount, const uint16_t FREQ_SAMPLE)
{
  // Delta T Initial 
  static uint32_t DTi = 0;
  // Delta T Final
  static uint32_t DTf = 0;
  // Samples For Frequency 
  uint32_t nFreq = 0;
  
  // Calculate Frequency
  if ((abs(inSignal[1])+abs(inSignal[0])) != 0)
  {
    DTf = (abs(inSignal[1])*4096)/(abs(inSignal[1])+abs(inSignal[0]));
  }
  nFreq = (sampleCount-1)*4096 + DTi + DTf;
  if ((abs(inSignal[0])+abs(inSignal[1])) != 0)
  {
    DTi = (abs(inSignal[0])*4096)/(abs(inSignal[0])+abs(inSignal[1]));
  }
  if (nFreq > 0)
  {
    return (uint32_t)(FREQ_SAMPLE*4096*100/nFreq);
  }
  
  return 0;
}



uint32_t calcRound(uint64_t value, uint16_t Exp)
{
  if ( (value % (10*Exp) ) >= 5*Exp )
  {
    value = (int)(value / (10*Exp) ) + 1;
  }
  else
  {
    value = (int)(value / (10*Exp) );
  }
  
  return value;
}



int32_t voltageInst(int32_t* inSignal)
{
  return inSignal[10];
}



int32_t currentInst(int32_t* inSignal1, int32_t* inSignal5, int32_t* inSignal10, uint8_t adcGain)
{
  int32_t curr = 0;
  
  // Current Instantaneous
  if (adcGain == 10)
  {
    curr = inSignal10[10];
  }
  else if (adcGain == 5)
  {
    curr = inSignal5[10] * 2;
  }
  else if (adcGain == 1)
  {
    curr = inSignal1[10] * 10;
  }
  
  return curr;
  
}



uint8_t configADC(int32_t* inSignal1, int32_t* inSignal5, int32_t* inSignal10, bool zeroCross)
{
  static bool adc_10X = true;
  static bool adc_5X = false;
  static bool adc_1X = false;
  static int32_t peakCurr = 0;
  static int32_t peakCurr5X = 0;
  static int32_t peakCurr10X = 0;
  static uint8_t adcGain = 10;
  
  // Peaks Currents
  if (abs(inSignal1[10]) > peakCurr)
  {
    peakCurr = abs(inSignal1[10]);
  }

  if (abs(inSignal5[10]) > peakCurr5X)
  {
    peakCurr5X = abs(inSignal5[10]);
  }
  
  if (abs(inSignal10[10]) > peakCurr10X)
  {
    peakCurr10X = abs(inSignal10[10]);
  }
  
  if (zeroCross)
  {
    // ADC configure
    if (adc_10X && (peakCurr10X > 1800)) {
    adc_10X = false;
    adc_5X = true;
    adcGain = 5;
    }

    if (adc_5X) {
      if (peakCurr5X > 1800) {
      adc_5X = false;
      adc_1X = true;
      adcGain = 1;
      } else if (peakCurr5X < 750) {
      adc_5X = false;
      adc_10X = true;
      adcGain = 10;
      }
    }

    if (adc_1X && (peakCurr < 300)) {
    adc_1X = false;
    adc_5X = true;
    adcGain = 5;
    }
    
    peakCurr = 0;
    peakCurr5X = 0;
    peakCurr10X = 0;
  }
  
  return adcGain;
}



uint32_t calcVoltRms(uint32_t sumSqrVolt, uint16_t sampleCount)
{ 
  // Calculate Voltage RMS
  if (sampleCount > 0)
  {
    return (uint32_t)(CALIB_V * sqrt(sumSqrVolt / sampleCount));
  }
  return 0;
}

uint64_t calcCurrRms(uint64_t sumSqrCurr, uint8_t adcGain, uint16_t sampleCount)
{ 
  // Calculate Current RMS
  if (sampleCount > 0)
  {
    if (adcGain == 10)
    {
      return (uint64_t)(CALIB_C10X * sqrt(sumSqrCurr / sampleCount));
    }
    else if (adcGain == 5)
    {
      return (uint64_t)(CALIB_C5X * sqrt(sumSqrCurr / sampleCount));
    }
    else if (adcGain == 1)
    {
      return (uint64_t)(CALIB_C1X * sqrt(sumSqrCurr / sampleCount));
    }
  }
  return 0;
}



uint64_t calcPwrReal(int32_t sumPwr, uint8_t adcGain, uint16_t sampleCount)
{
  // Calculate Power Real
  if (sampleCount > 0 && sumPwr > 0)
  {
    if (adcGain == 10)
    {
      return (uint64_t)(CALIB_V * CALIB_C10X * (sumPwr / sampleCount));
    }
    else if (adcGain == 5)
    {
      return (uint64_t)(CALIB_V * CALIB_C5X * (sumPwr / sampleCount));
    }
    else if (adcGain == 1)
    {
      return (uint64_t)(CALIB_V * CALIB_C1X * (sumPwr / sampleCount));
    }
  }
  return 0;
}



uint32_t calcPwrApparent(uint32_t VoltRms, uint64_t CurrRms)
{
  // Calculate Power Apparent
  if (VoltRms > 5000 && CurrRms > 150)
  {
    return (uint32_t)(VoltRms * CurrRms);
  }
  return 0;
}



uint32_t calcPwrReative(uint32_t pwrReal, uint32_t pwrApparent)
{
  // Calculate Power Reative
  if (pwrApparent >= pwrReal)
  {
    return (uint32_t)sqrt((pwrApparent * pwrApparent * 100) - (pwrReal * pwrReal * 100));
  } 
  else
  {
    return (uint32_t)sqrt((pwrReal * pwrReal * 100) - (pwrApparent * pwrApparent * 100));
  }

}



uint32_t calcFactorPwr(uint32_t pwrReal, uint32_t pwrApparent)
{
  // Calculate Factor Power
  if (pwrApparent > 0)
  {
    return (uint32_t)((pwrReal * 10000) / pwrApparent);
  }
  return 0;
}



uint32_t calcEneCunsumed(int32_t sumPwr, uint8_t adcGain, const uint16_t FREQ_SAMPLE)
{
  // Calculate Energy Consumed per Cycle
  if (sumPwr > 0)
  {
    if (adcGain == 10)
    {
      return (uint32_t)((CALIB_V * CALIB_C10X * sumPwr) / (36 * FREQ_SAMPLE)); // (3600*FREQ_SAMPLE)
    }
    else if (adcGain == 5)
    {
      return (uint32_t)((CALIB_V * CALIB_C5X * sumPwr) / (36 * FREQ_SAMPLE)); // (3600*FREQ_SAMPLE)
    }
    else if (adcGain == 1)
    {
      return (uint32_t)((CALIB_V * CALIB_C1X * sumPwr)/ (36 * FREQ_SAMPLE)); // (3600*FREQ_SAMPLE) => 36 no lugar de 3600 para aumentar a precisão
    } 
  }
  return 0;
}
