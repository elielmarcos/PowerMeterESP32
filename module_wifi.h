#include <WiFi.h>

#define DEBUG

#define LED 2

// WiFi Raspberry
/*
#define SSID "WiFi-Pi"
#define PASSWORD "raspberry"
#define PORT 12000
#define SERVER "192.168.50.1"
*/

// WiFi Hotspot Notebook
#ifdef WIFIHOTSPOT
  #define SSID "RedeEliel"
  #define PASSWORD "123456789"
  #define PORT 12000
  #define SERVER "192.168.137.1"
#endif

// WiFi Casa com servidor externo
#ifdef WIFICASA
  #define SSID "CASA 1074"
  #define PASSWORD "34383696"
  #define PORT 12000
  #define SERVER "elielmarcos.ddns.net"
#endif

// WiFi Apartamento 201 com servidor externo
#ifdef WIFIAPARTAMENTO
  #define SSID "APT 201"
  #define PASSWORD "999298448"
  #define PORT 12000
  #define SERVER "elielmarcos.ddns.net"
#endif

WiFiClient client;

void init_WiFi();
void WiFiInit();
boolean WiFiConnect();
boolean WifiIsConnected();
boolean reconnect();
boolean ClientConnected();
boolean SendMsg(String txMsg);
String RecvMsg();

TaskHandle_t connectionHandle = NULL;
void ConnectionHead(void *pvParameters);


void init_WiFi()
{
  WiFiInit();
  
  xTaskCreatePinnedToCore(ConnectionHead, "wifi_task", 4096, NULL, 6, &connectionHandle, 0); // menor indice, menor prioridade, maior indice, maior prioridade
  
}


void WiFiInit()
{

  //Desliga WiFi para receber as novas configurações
  WiFi.disconnect(true);  

  WiFi.mode(WIFI_STA);
  
  WiFi.setAutoConnect(false);
  WiFi.setAutoReconnect(false);
  WiFi.setSleep(false); 
  
  WiFi.setTxPower(WIFI_POWER_19_5dBm); // Sets WiFi RF power output to highest level, highest RF power usage 

  delay(5000);

  vTaskDelay(pdMS_TO_TICKS(500));

}


void ConnectionHead( void *pvParameters )
{
    for(;;)
    {
      if (ClientConnected())
        digitalWrite(LED, HIGH);
      else
        digitalWrite(LED, LOW);
        
      if (!WifiIsConnected())
        WiFiConnect();
  
      vTaskDelay(pdMS_TO_TICKS(500));
    }
}


boolean WiFiConnect()
{

  if ( !WifiIsConnected() )
  {
    if ( !reconnect() )
    {
      return false;
    }
  }

  #ifdef DEBUG
  Serial.print(F("WiFi: Connected with IP: "));
  Serial.println(WiFi.localIP());
  #endif
  return true;
  
}


boolean reconnect() 
{
  
  if ( !WifiIsConnected() ) 
  {
    #ifdef DEBUG
    Serial.println(F("WiFi: Reconnecting"));
    Serial.print(F("SSID: ")); Serial.println(SSID);
    Serial.print(F("PASS: ")); Serial.println(PASSWORD);
    #endif
    WiFi.disconnect(false);
    vTaskDelay(pdMS_TO_TICKS(500));
    #ifdef DEBUG
    Serial.println(F("WiFi: Connecting..."));
    #endif
    WiFi.mode(WIFI_STA);
	  WiFi.setTxPower(WIFI_POWER_19_5dBm); // Sets WiFi RF power output to highest level, highest RF power usage 
    vTaskDelay(pdMS_TO_TICKS(500));
    WiFi.begin(SSID,PASSWORD);
    vTaskDelay(pdMS_TO_TICKS(5000));

    if ( !WifiIsConnected() )
    {
      return false;
    }

    #ifdef DEBUG
    Serial.println("WiFi: Station Connected (" + String(SSID) + ")");
    #endif
  }
  
  return true;
  
}


boolean WifiIsConnected()
{
  if ( WiFi.status() != WL_CONNECTED ) 
  {
    return false;
  }
  
  return true;
}


boolean ClientConnected()
{

  static bool clt = 0;

  if (!client)
  {
    if (clt != client)
      goto DISCONNECT;
    
    if (WifiIsConnected() && client.connect(/*WiFi.gatewayIP()*/SERVER, PORT)){
      client.setTimeout(10);
      #ifdef DEBUG
      Serial.print("NODE CONNECTED: ");
      Serial.println(client.remoteIP());
      #endif
      clt = client;
    }
    else return false;
  }

  if (WifiIsConnected() && client.connected())
    return true;

  DISCONNECT:
  #ifdef DEBUG
  Serial.println("NODE DISCONNECTED");
  #endif
  client.stop();
  clt = client;
  return false;
  
}


boolean SendMsg(String txMsg)
{
  if (client.connected()) 
  {
    //Enviamos
    //client.print(txMsg);
    client.write(txMsg.c_str(), txMsg.length());
    client.flush();
    return true;
  }
  
  return false;
}


String RecvMsg()
{

  String rxMsg = "";
  
  if (client.connected() && client.available())
  {      
    //Se o cliente tem dados que deseja nos enviar
    while (client.available())
      rxMsg += String((char)client.read());
  }
  
  return String(rxMsg);
}
