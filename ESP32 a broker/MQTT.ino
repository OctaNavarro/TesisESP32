#include <WiFi.h>
#include <PubSubClient.h>

//-------------------VARIABLES GLOBALES--------------------------
int contconexion = 0;

const char *ssid = "WiFi-Arnet-bsmv-2.4G";
const char *password = "154012817";
char   SERVER[50]   = "3.83.156.245"; //"driver.cloudmqtt.com"
int    SERVERPORT   = 18813;
String USERNAME = "ESP32Proto";   
char   PASSWORD[50] = "12345678";     

unsigned long previousMillis = 0;

char PLACA[50];

char valueStr[15];
String strtemp = "";
char MEDICIONDB[50];

int rngNum;
String strMed = "";

//-------------------------------------------------------------------------
WiFiClient octaClient;
PubSubClient client(octaClient);

//------------------------RECONNECT-----------------------------
void reconnect() {
  uint8_t retries = 3;
  // Loop hasta que estamos conectados
  while (!client.connected()) {
    Serial.print("Intentando conexion MQTT...");
    // Crea un ID de cliente al azar
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    USERNAME.toCharArray(PLACA, 50);
    if (client.connect("", PLACA, PASSWORD)) {
      Serial.println("conectado");
     }
     else {
      Serial.print("fallo, rc=");
      Serial.print(client.state());
      Serial.println(" intenta nuevamente en 5 segundos");
      // espera 5 segundos antes de reintentar
      delay(5000);
    }
    retries--;
    if (retries == 0) {
      // esperar a que el WDT lo reinicie
      while (1);
    }
  }
}

//------------------------SETUP-----------------------------
void setup() {

  // Inicia Serial
  Serial.begin(9600);
  Serial.println("");

  // Conexión WIFI
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED and contconexion <50) { //Cuenta hasta 50 si no se puede conectar lo cancela
    ++contconexion;
    delay(500);
    Serial.print(".");
  }
  if (contconexion <50) {
      //para usar con ip fija
      IPAddress ip(192,168,1,156); 
      IPAddress gateway(192,168,1,1); 
      IPAddress subnet(255,255,255,0); 
      WiFi.config(ip, gateway, subnet); 
      
      Serial.println("");
      Serial.println("WiFi conectado");
      Serial.println(WiFi.localIP());
  }
  else { 
      Serial.println("");
      Serial.println("Error de conexion");
  }
  
  client.setServer(SERVER, SERVERPORT);
  
  String temperatura = "/" + USERNAME + "/" + "LectdB"; 
  temperatura.toCharArray(MEDICIONDB, 50);
    
}

//--------------------------LOOP--------------------------------
void loop() {

  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
    
  if (currentMillis - previousMillis >= 2000) { //envia la medición cada 2s
    previousMillis = currentMillis;
    
    int medicion = random(100);
    char valueStr [5];
    itoa(medicion, valueStr, 10);
    
    Serial.println("Enviando: [" +  String(MEDICIONDB) + "] " + medicion);
    client.publish(MEDICIONDB, valueStr); //MEDICIONDB es "/ESP32Proto/LectdB" y valueStr es la cadena de caracteres que mandamos a ese topic del broker
  }
    
  }
