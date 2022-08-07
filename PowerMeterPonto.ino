/*
 * 
 * CALCULO DOS DADOS A CADA CICLO DE FASE 60Hz
 * 
 * ENVIA PARA O GATEWAY A CADA 5 SEGUNDOS
 * 
 * 
 */

#include <math.h>
#include <stdio.h>

#include <esp_system.h>
#include <driver/adc.h>
#include <esp_adc_cal.h>
#include <esp_task_wdt.h>
#include "esp_spiffs.h"
#include "freertos/task.h"
#include "soc/rtc_wdt.h"
#include "time.h"

//#define DEBUG

//#define WIFIHOTSPOT
//#define WIFICASA
#define WIFIAPARTAMENTO

#define SENSOR_15A
//#define SENSOR_30A

#define POWERMETER "pm03"
#define LED 2

#include "module_wifi.h"
#include "filters_func.h"


// Tasks
TaskHandle_t serverComunicationHandle = NULL;
void ServerComunication(void *pvParameters);
TaskHandle_t signalHandle = NULL;
static void SignalProcess(void *pvParameters);

// Interrupt
void IRAM_ATTR onTimer();
// Attribute Timer
hw_timer_t* timer = NULL; 

#define LEN_BUF 60
#define LEN_PIPE 60

// Struct Queue Data
typedef struct {
  uint16_t _size = LEN_BUF;
  uint16_t _pos = 0;
  uint32_t rtc[LEN_BUF];
  uint16_t ms[LEN_BUF];
  int32_t v[LEN_BUF];
  int32_t i[LEN_BUF];
  int32_t p[LEN_BUF];
  int32_t q[LEN_BUF];
  int32_t s[LEN_BUF];
} DataInfo;


// Queue Data
QueueHandle_t dataPipe;

// NTP Server
const char NTP_SERVER[] = "a.st1.ntp.br"; //"pool.ntp.br";
// NTP Packet Size
const uint8_t NTP_PACKET_SIZE = 48;
// NTP Packet
uint8_t ntp[NTP_PACKET_SIZE];
// UDP Port
const int UDP_LOCALPORT = 1337; //2390;
// Timer RTC
timeval RTC;
//Cria a estrutura que contem as informacoes da data.
struct tm data; 


// Functions
bool timeNTPUpdate();
uint32_t timeNTP();
void hourUpdate(uint32_t TimeStamp);
uint64_t time_get_time();
String uint64ToString(uint64_t input);

void setup()
{
    dataPipe = xQueueCreate(LEN_PIPE,sizeof(DataInfo)); // 60 slots is Pipe size, if esp32 is reset then decrease Pipe size
  
    Serial.begin(115200);

    pinMode(LED,OUTPUT);
    digitalWrite(LED,LOW);

    Serial.print(F("POWER METER POINT ["));
    Serial.print(POWERMETER);
    Serial.println(F("] - START"));

    init_WiFi();

    uint32_t lastTime = millis();
    uint32_t interval = 0.5*60*1000; // 30 segundos

    delay(2000);

    while (!timeNTPUpdate())    // Aguarda obter hora do servidor NTP
    {
      if ((millis() - lastTime) > interval) // reinicia se em 30 segundos não receber a hora do servidor
        ESP.restart();
      
      vTaskDelay(1000);
    }

    xTaskCreatePinnedToCore(ServerComunication, "comunic_task", 8192, NULL, 5, &serverComunicationHandle, 0); // menor indice, menor prioridade, maior indice, maior prioridade

    xTaskCreatePinnedToCore(SignalProcess, "SigProcess", 16384, NULL, 7, &signalHandle, 1); // Com prioridade 20 o WiFi não conecta
    
}



void loop()
{
  
   vTaskDelay(10000);

}


void ServerComunication( void *pvParameters )
{

  DataInfo dataBuffer;
  uint32_t lastTime = millis();
  uint32_t interval = 2*60*1000; //2 minutos
  uint8_t queueSize = 0;
  uint8_t reading = 0;
  String msgData = "";
  String msgLength = "";
  
  
  while(true) {
    
    if (reading < 10) // Dez leituras no máximo por mensagem para serem enviadas ao gateway
    {
      
      if (!xQueueReceive( dataPipe, &( dataBuffer ), (TickType_t) pdMS_TO_TICKS(1))) // Read pipe data measurement (wait 1ms if busy or empty) 
      {
        //Serial.println("E/B RECEIVE");  // Se estiver vazio ou oculpado e não foi possivel receber
      }
      else
      {
        for (int pos=0; (pos<dataBuffer._pos) and (pos<dataBuffer._size); pos++)
        {
          msgData +=  String((double)dataBuffer.rtc[pos] + (double)dataBuffer.ms[pos]/1000, 3) + ";" + 
                      String((float)dataBuffer.v[pos]/10, 1) + ";" +
                      String((float)dataBuffer.i[pos]/1000, 3) + ";" +
                      String((float)dataBuffer.p[pos]/10, 1) + ";" +
                      String((float)dataBuffer.q[pos]/10, 1) + ";" +
                      String((float)dataBuffer.s[pos]/10, 1) + "\n"; 
        }
        
        reading++;
        
        //Serial.println(msgData.length());
        //Serial.println(msgData);
        //msgData = "";
      }
    }


    if (reading >= 5) // Com cinco leituras inicia-se as tentativas de envio para o gateway
    {
      
      msgLength = String(POWERMETER) + String("&") + String(msgData.length());
      //msgLength = String(msgData.length());

      for (int i=0; i<(10-String(msgLength).length()); i++)
        msgLength += ' ';

      if (SendMsg(msgLength)) {
          //Serial.println("SEND MSG");
          if (SendMsg(msgData)) {
            //Serial.println("SEND MSG");
            msgData = "";
            reading = 0;
            lastTime = millis();
          }
          else Serial.println("ERRO SEND MSG DATA!");
        }
        else Serial.println("ERRO SEND MSG LENGTH!");

      queueSize = uxQueueMessagesWaiting(dataPipe);  // Retorna quantidade de itens ainda disponíveis no Pipe
      if (queueSize>=LEN_PIPE) // Descarta os dados armazenados na mensagem e reinicia as leituras 
      {
        msgData = "";
        reading = 0;
        Serial.println("PIPE FULL");    
      }
  
    }

    if ((millis() - lastTime) > interval) // reinicia se em 2 minutos não receber conseguir enviar os dados para gateway
      ESP.restart();


    vTaskDelay(20);
    
  }

}



/*******************************************************************************
  INTERRUPTIONS
*******************************************************************************/
void IRAM_ATTR onTimer()
{

  BaseType_t xHigherPriorityTaskWoken;

  xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR( signalHandle, &xHigherPriorityTaskWoken );
  portYIELD_FROM_ISR( );
  
}


/*******************************************************************************
  TASK FOR SIGNAL PROCESSING
*******************************************************************************/
void SignalProcess( void *pvParameters )
{

  // VARS - Signal Processing ------------------------------------------------

  // Sample Rate
  const uint16_t FREQ_SAMPLE= 4000;
  // Level DC - Filter High Pass
  const uint16_t DC_LEVEL = 2048;
  // Coefficientes Integer, Precision 2^15-1
  const int32_t PRECISION = 32767;

  // ADC Gain
  uint8_t adcGain = 10;

  // Sample Count
  uint16_t sampleCount = 0;
  // Zero-Crossing
  boolean zeroCross = false;
  // Zero-Crossing Count
  uint8_t zeroCrossCount  = 0;
  // Frequency
  uint32_t frequency = 0;
  // Sum Frequency
  uint32_t sumFrequency = 0;
  // Voltage Instantaneous
  int32_t voltInst = 0;
  // Current Instantaneous
  int32_t currInst = 0;
  // Power Instantaneous
  int32_t pwrInst = 0;
  // Sum Voltage Square
  uint32_t sumSquareVolt = 0;
  // Sum Current Square
  uint64_t sumSquareCurr = 0;
  // Sum Power Instantaneous
  int32_t sumPwr = 0;
  // Sum Voltage RMS
  uint32_t sumVoltRms = 0;
  // Sum Current RMS
  uint64_t sumCurrRms = 0;
  // Sum Power Real
  uint64_t sumPwrReal = 0;
  // Voltage RMS
  uint32_t voltRms = 0;
  // Current RMS
  uint32_t currRms = 0;
  // Power Real
  uint32_t pwrReal = 0;
  // Power Apparent
  uint32_t pwrApparent = 0;
  // Power Reative
  uint32_t pwrReative = 0;
  // Factor Power
  uint32_t factorPwr = 0;
  // Sum Energy Consumed per Cycle
  uint32_t sumEneCons = 0;
  // Energy Consumed per Second
  uint32_t eneConsSec = 0;
  // Energy Consumed Total
  //uint64_t eneConsTotal = 0;
  
  // Samples Raw Voltage
  int32_t samples_Volt[2] = {DC_LEVEL, DC_LEVEL};
  // Samples Raw Current
  int32_t samples_Curr1X[2] = {DC_LEVEL, DC_LEVEL};
  // Samples Raw Current x 5
  int32_t samples_Curr5X[2] = {DC_LEVEL, DC_LEVEL};
  // Samples Raw Current x 10
  int32_t samples_Curr10X[2] = {DC_LEVEL, DC_LEVEL};

  // Outputs Filter HighPass Voltage
  int32_t outFilterHP_Volt[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  // Outputs Filter HighPass Current
  int32_t outFilterHP_Curr1X[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  // Outputs Filter HighPass Current x 5
  int32_t outFilterHP_Curr5X[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  // Outputs Filter HighPass Current x 10
  int32_t outFilterHP_Curr10X[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  
  // Outputs Filter LowPass Voltage
  int32_t outFilterLP_Volt[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  // Outputs Filter LowPass Current
  int32_t outFilterLP_Curr1X[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  // Outputs Filter LowPass Current x 5
  int32_t outFilterLP_Curr5X[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  // Outputs Filter LowPass Current x 10
  int32_t outFilterLP_Curr10X[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
  
  // Outputs Filter LowPass Voltage Zero-Crossing
  int32_t outFilterZC_Volt[21] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

  // ADC - Configure ---------------------------------------------------------
  adc1_config_width(ADC_WIDTH_BIT_12);  // Resolution
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_0);  // Channel and attenuation (Voltage)
  adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_0);  // Channel and attenuation (Current1x)
  adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_0);  // Channel and attenuation (Current5x)
  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_0);  // Channel and attenuation (Current10x)
  analogSetClockDiv(1);
  //analogReadResolution(12);
  

  // TIMER0 - Configure for interruption -------------------------------------
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, (int)(1E6/FREQ_SAMPLE), true);
  timerAlarmEnable(timer);

    
  // Data Pipe/Buffers -------------------------------------------------------

  DataInfo dataBuffer1;
  DataInfo dataBuffer2;
  uint8_t _buffer = 1;

  //DataInfo dataBuffer;
  //bool _buffer = false;


  // RTC ---------------------------------------------------------------------
  uint64_t rtc_time = time_get_time();
  uint64_t last_rtc_time = rtc_time;

  while(true)
  {
    // ************************** Loop processamento dos sinais
    
    ulTaskNotifyTake( pdTRUE, portMAX_DELAY ); // aguarda notificação da interrupção Timer para execultar o loop de processamento

    // Increment Count Sample
    sampleCount++;

    // Samples ADC
    samplesADCraw(samples_Volt, samples_Curr1X, samples_Curr5X, samples_Curr10X);

    // Filter Remove Level DC (0.1Hz)
    filterRemoveDC(samples_Volt, outFilterHP_Volt, PRECISION);
    filterRemoveDC(samples_Curr1X, outFilterHP_Curr1X, PRECISION);
    filterRemoveDC(samples_Curr5X, outFilterHP_Curr5X, PRECISION);
    filterRemoveDC(samples_Curr10X, outFilterHP_Curr10X, PRECISION);

    // Filter Remove Noise (1600Hz)
    filterRemoveNoise(outFilterHP_Volt, outFilterLP_Volt, PRECISION);
    filterRemoveNoise(outFilterHP_Curr1X, outFilterLP_Curr1X, PRECISION);
    filterRemoveNoise(outFilterHP_Curr5X, outFilterLP_Curr5X, PRECISION);
    filterRemoveNoise(outFilterHP_Curr10X, outFilterLP_Curr10X, PRECISION);

    // Filter Remove Zero Cross (60Hz)
    filterZeroCross(outFilterLP_Volt, outFilterZC_Volt, PRECISION);

    // Return if Voltage Signal Zero Crossing / Cicle Full
    zeroCross = zeroCrossing(outFilterZC_Volt);

    // Return Voltge Inst. After Filtered
    voltInst = voltageInst(outFilterLP_Volt); 

    // Return Current Inst. After Filtered 
    currInst = currentInst(outFilterLP_Curr1X, outFilterLP_Curr5X, outFilterLP_Curr10X, adcGain);

    // Calc Power Inst. -> Pi = Vi * Ci
    pwrInst = voltInst * currInst;

    // Sum Square Voltage -> Vs = Vi^2 
    sumSquareVolt += voltInst * voltInst;

    // Sum Square Current -> Cs = Ci^2 
    sumSquareCurr += currInst * currInst;

    // Sum Power Inst.
    sumPwr += pwrInst;

    // Analyse ADC for each Sample and Configure for 1X, 5X or 10X for each Zero Crossing / Cicle Full
    adcGain = configADC(outFilterLP_Curr1X, outFilterLP_Curr5X, outFilterLP_Curr10X, zeroCross);

    // For each Zero Crossing / Cicle Full
    if (zeroCross)
    {
      // ************************************* Processa os dados a cada Cruzamento de Zero

      // Get time RTC
      rtc_time = time_get_time();

      // Calc Frequency for each Zero Crossing (Cicle Full)
      sumFrequency = calcFrequency(outFilterZC_Volt,sampleCount,FREQ_SAMPLE);
      // Round Frequency
      frequency = calcRound(sumFrequency,1);

      // Calc Voltage RMS for each Zero Crossing (Cicle Full)
      sumVoltRms = calcVoltRms(sumSquareVolt,sampleCount);
      // Round Voltage RMS
      voltRms = calcRound(sumVoltRms,1);

      // Calc Current RMS for each Zero Crossing (Cicle Full)
      sumCurrRms = calcCurrRms(sumSquareCurr,adcGain,sampleCount);
      // Round Current RMS
      currRms = calcRound(sumCurrRms,1);

      // Calc Power Real for each Zero Crossing (Cicle Full)
      sumPwrReal = calcPwrReal(sumPwr,adcGain,sampleCount);
      // Round Power Real
      pwrReal = calcRound(sumPwrReal,10000);

      // Calc Power Apparent for each Zero Crossing (Cicle Full)
      pwrApparent = calcPwrApparent(sumVoltRms,sumCurrRms);      
      // Round Power Apparent
      pwrApparent = calcRound(pwrApparent,10000);

     // Calc Power Reative for each Zero Crossing (Cicle Full)
      pwrReative = calcPwrReative(pwrReal,pwrApparent);
      // Round Power Reative
      pwrReative =  calcRound(pwrReative,1);

      // Calc Factor Power for each Zero Crossing (Cicle Full)
      factorPwr = calcFactorPwr(pwrReal,pwrApparent);
      // Round Factor Power
      factorPwr = calcRound(factorPwr,1);

      // Reset Number of Sample per Zero Corssing / Cicle Full
      sampleCount = 0; 

      // Reset sum / Cicle Full
      sumSquareVolt = 0;
      sumSquareCurr = 0;
      sumPwr = 0;
      sumFrequency = 0;
      sumVoltRms = 0;
      sumCurrRms = 0;
      sumPwrReal = 0;


      // ************************************* Preenche os dados processados nos buffers a cada Cruzamento de Zero
      
      if ((_buffer == 1) and (dataBuffer1._pos < dataBuffer1._size))
      {
        dataBuffer1.rtc[dataBuffer1._pos] = (uint32_t)(rtc_time / 1000);
        dataBuffer1.ms[dataBuffer1._pos] = rtc_time % 1000;
        dataBuffer1.v[dataBuffer1._pos] = voltRms;
        dataBuffer1.i[dataBuffer1._pos] = currRms;
        dataBuffer1.p[dataBuffer1._pos] = pwrReal;
        dataBuffer1.q[dataBuffer1._pos] = pwrReative;
        dataBuffer1.s[dataBuffer1._pos] = pwrApparent;
        dataBuffer1._pos++;
  
        if (dataBuffer1._pos >= dataBuffer1._size)
        {
          if (dataBuffer2._pos >= dataBuffer2._size) Serial.println("FULL B2");
          dataBuffer2._pos = 0; // Mesmo que buffer 2 estiver cheio e não foi enviado ao Pipe, mas buffer 1 já encheu, então será sobrescrito novos dados no buffer 2
          _buffer = 2;
        }
          
      }
      else if ((_buffer == 2) and (dataBuffer2._pos < dataBuffer2._size))
      {
        dataBuffer2.rtc[dataBuffer2._pos] = (uint32_t)(rtc_time / 1000);
        dataBuffer2.ms[dataBuffer2._pos] = rtc_time % 1000;
        dataBuffer2.v[dataBuffer2._pos] = voltRms;
        dataBuffer2.i[dataBuffer2._pos] = currRms;
        dataBuffer2.p[dataBuffer2._pos] = pwrReal;
        dataBuffer2.q[dataBuffer2._pos] = pwrReative;
        dataBuffer2.s[dataBuffer2._pos] = pwrApparent;
        dataBuffer2._pos++;
  
        if (dataBuffer2._pos >= dataBuffer2._size)
        {
          if (dataBuffer1._pos >= dataBuffer1._size) Serial.println("FULL B1");   
          dataBuffer1._pos = 0; // Mesmo que buffer 1 estiver cheio e não foi enviado ao Pipe, mas buffer 2 já encheu, então será sobrescrito novos dados no buffer 1
          _buffer = 1;
        }
        
      }

      // FIM ************************************* Processa os dados a cada Cruzamento de Zero
    }


    if ((_buffer == 2) and (dataBuffer1._pos >= dataBuffer1._size))  
    {  
      // se a fila estiver cheia ou oculpada, não aguarda e não preenche
      if (!xQueueSend( dataPipe, (void *) &dataBuffer1, (TickType_t) 0)) // Write pipe data measurement (not wait if busy or full) 
      {
        //Serial.println("F/B1 SEND"); // Erro ao escrever o buffer 1 no Pipe
      }
      else
      {
        dataBuffer1._pos = 0; // Se sucesso na escrita o buffer 1 no Pipe, então reinicia a posição de escrita no buffer 1
      }
    }
    else if ((_buffer == 1) and (dataBuffer2._pos >= dataBuffer2._size))  
    {  
      // se a fila estiver cheia ou oculpada, não aguarda e não preenche
      if (!xQueueSend( dataPipe, (void *) &dataBuffer2, (TickType_t) 0)) // Write pipe data measurement (not wait if busy or full) 
      {
        //Serial.println("F/B2 SEND"); // Erro ao escrever o buffer 2 no Pipe
      }
      else
      {
        dataBuffer2._pos = 0; // Se sucesso na escrita o buffer 2 no Pipe, então reinicia a posição de escrita no buffer 2
      }
    }


    // FIM ************************** Loop processamento dos sinais
  }
}


/* ===============   FUNCAO HOUR UPDATE   ============== */
void hourUpdate(uint32_t TimeStamp)
{

    RTC.tv_sec = TimeStamp;   //Atribui data atual em TimeStamp
    settimeofday(&RTC, NULL); //Configura o RTC para a data atribuida.
   
    Serial.println("RTC: Update (" + String(TimeStamp) + ")");
    
    return;

}

/*******************************************************************************
* NTP DATE/TIME
*******************************************************************************/
uint32_t timeNTP()
{

    Serial.print(F("NTP: [LOCA_LPORT]: "));
    Serial.print(UDP_LOCALPORT);
    Serial.print(F(" [NTP_SEREVR]: "));
    Serial.println(NTP_SERVER);
    
    // Return time_t from NTP Server
    if (!WifiIsConnected())
    {
        // No WiFi connection
        Serial.println(F("NTP: No WiFi Connection - ERRO"));
        return 0;
    }
    
    memset(ntp, 0, NTP_PACKET_SIZE);
    ntp[ 0] = 0b11100011; // LI, Version, Mode
    ntp[ 1] = 0;          // Stratum, or type of clock
    ntp[ 2] = 6;          // Polling Interval
    ntp[ 3] = 0xEC;       // Peer Clock Precision
    ntp[12] = 49;
    ntp[13] = 0x4E;
    ntp[14] = 49;
    ntp[15] = 52;
    // Get time from server
    WiFiUDP udp;
    udp.begin(UDP_LOCALPORT);
    udp.beginPacket(NTP_SERVER, 123);
    udp.write(ntp, NTP_PACKET_SIZE);
    udp.endPacket();
    vTaskDelay(1000); // wait 1 second
    unsigned long l;
    if (udp.parsePacket())
    {
        // Success
        udp.read(ntp, NTP_PACKET_SIZE);
        l = word(ntp[40], ntp[41]) << 16 | word(ntp[42], ntp[43]);
        l -= 2208988800UL;      // Calculate from 1900 to 1970
        Serial.println(F("NTP: Received Pack - Ok"));
    } 
    else
    {
        //Error
        Serial.println(F("NTP: No Receive Pack - ERRO"));
        l = 0;
    }
    return l;
}


/* ===============   FUNCAO CHAMA CONEXÃO E ATUALIZAÇÃO DA HORA POR NTP   ============== */
bool timeNTPUpdate()
{

    uint32_t timeStamp = timeNTP();
    if (timeStamp)
    {
        hourUpdate(timeStamp);
    }
    else
    {
        return false;
    }

    return true;
}



/* ===============   FUNCAO RETORNA HORA EM MILLIS  ============== */
uint64_t time_get_time() {
  //struct timeval tv;
  gettimeofday(&RTC, NULL);
  //return RTC.tv_sec; // s
  return (RTC.tv_sec * 1000LL + (RTC.tv_usec / 1000LL)); // ms
  //return (RTC.tv_sec * 1000000LL + RTC.tv_usec); // us
}


/* ===============   FUNCAO CONVERTE UINT64 EM STRING  ============== */
String uint64ToString(uint64_t input) {
  String result = "";
  uint8_t base = 10;

  do {
    char c = input % base;
    input /= base;

    if (c < 10)
      c +='0';
    else
      c += 'A' - 10;
    result = c + result;
  } while (input);
  return result;
}
