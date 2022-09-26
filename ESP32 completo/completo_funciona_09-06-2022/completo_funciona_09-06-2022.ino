//-----Librerias para lectura de dB------
#include <driver/i2s.h>
#include "sos-iir-filter.h"

//-----Librerias para MQTT------
#include <WiFi.h>
#include <PubSubClient.h>

//-----Definiciones para lectura de dB-----

#define LEQ_PERIOD        0.5          // Tiempo de integración LAeq
#define WEIGHTING         A_weighting // Also avaliable: 'C_weighting' or 'None' (Z_weighting)
#define LEQ_UNITS         "LAeq"      // customize based on above weighting used
#define DB_UNITS          "dBA"       // customize based on above weighting used

#define MIC_EQUALIZER     INMP441    // See below for defined IIR filters or set to 'None' to disable
#define MIC_OFFSET_DB     2.2      // Default offset (sine-wave RMS vs. dBFS). Modify this value for linear calibration

#define MIC_SENSITIVITY   -26         // dBFS value expected at MIC_REF_DB (Sensitivity value from datasheet)
#define MIC_REF_DB        94.0        // Value at which point sensitivity is specified in datasheet (dB)
#define MIC_OVERLOAD_DB   116.0       // dB - Acoustic overload point
#define MIC_NOISE_DB      29          // dB - Noise floor
#define MIC_BITS          24          // valid number of bits in I2S data
#define MIC_CONVERT(s)    (s >> (SAMPLE_BITS - MIC_BITS))
#define MIC_TIMING_SHIFT  0           // Set to one to fix MSB timing for some microphones, i.e. SPH0645LM4H-x

double Leq_dB = 0;

constexpr double MIC_REF_AMPL = pow(10, double(MIC_SENSITIVITY)/20) * ((1<<(MIC_BITS-1))-1);

#define I2S_WS            18 
#define I2S_SCK           23 
#define I2S_SD            19 

#define I2S_PORT          I2S_NUM_0


//-----Variables globales para MQTT-----
int contconexion = 0;

const char *ssid = "WiFi-Arnet-bsmv-2.4G"; //Casa:"WiFi-Arnet-bsmv-2.4G";  Cel:"Octas phone"; Sosa:"NsAccessPoint-13722742" Facu: "Cachalote Escarlata 2.4GHz"
const char *password = "154012817"; //Casa y cel:"154012817"; Sosa: "HGC-121457"
char   SERVER[50]   = "3.83.156.245"; //"driver.cloudmqtt.com" "3.83.156.245"
int    SERVERPORT   = 18813;
String USERNAME = "ESP32Proto";   // Usuario de cloudmqtt
char   PASSWORD[50] = "12345678";  // Clave cloudmqtt   

unsigned long previousMillis = 0;

char PLACA[50];

char valueStr[15];
char MEDICIONDB[50];
String strMed = "";

//------Clientes WiFi y MQTT-----
WiFiClient octaClient;
PubSubClient client(octaClient);

//------FILTROS-------

SOS_IIR_Filter DC_BLOCKER = { 
  gain: 1.0,
  sos: {{-1.0, 0.0, +0.9992, 0}}
};

SOS_IIR_Filter INMP441 = {
  gain: 1.00197834654696, 
  sos: { 
    {-1.986920458344451, +0.986963226946616, +1.995178510504166, -0.995184322194091}
  }
};

SOS_IIR_Filter A_weighting = {
  gain: 0.169994948147430, 
  sos: { 
    {-2.00026996133106, +1.00027056142719, -1.060868438509278, -0.163987445885926},
    {+4.35912384203144, +3.09120265783884, +1.208419926363593, -0.273166998428332},
    {-0.70930303489759, -0.29071868393580, +1.982242159753048, -0.982298594928989}
  }
};

SOS_IIR_Filter C_weighting = {
  gain: -0.491647169337140,
  sos: { 
    {+1.4604385758204708, +0.5275070373815286, +1.9946144559930252, -0.9946217070140883},
    {+0.2376222404939509, +0.0140411206016894, -1.3396585608422749, -0.4421457807694559},
    {-2.0000000000000000, +1.0000000000000000, +0.3775800047420818, -0.0356365756680430}
  }
};


//-------Sampling----------

#define SAMPLE_RATE       48000 // Hz, fixed to design of IIR filters
#define SAMPLE_BITS       32    // bits
#define SAMPLE_T          int32_t 
#define SAMPLES_SHORT     (SAMPLE_RATE / 8) // ~125ms
#define SAMPLES_LEQ       (SAMPLE_RATE * LEQ_PERIOD)
#define DMA_BANK_SIZE     (SAMPLES_SHORT / 16)
#define DMA_BANKS         32

 
struct sum_queue_t {
  
  float sum_sqr_SPL;
  float sum_sqr_weighted;
  uint32_t proc_ticks;
};
QueueHandle_t samples_queue;


float samples[SAMPLES_SHORT] __attribute__((aligned(4)));


//-------I2S Microphone sampling setup---------- 
void mic_i2s_init() {

  const i2s_config_t i2s_config = {
    mode: i2s_mode_t(I2S_MODE_MASTER | I2S_MODE_RX),
    sample_rate: SAMPLE_RATE,
    bits_per_sample: i2s_bits_per_sample_t(SAMPLE_BITS),
    channel_format: I2S_CHANNEL_FMT_ONLY_LEFT,
    communication_format: i2s_comm_format_t(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_MSB),
    intr_alloc_flags: ESP_INTR_FLAG_LEVEL1,
    dma_buf_count: DMA_BANKS,
    dma_buf_len: DMA_BANK_SIZE,
    use_apll: true,
    tx_desc_auto_clear: false,
    fixed_mclk: 0
  };
  // I2S pin mapping
  const i2s_pin_config_t pin_config = {
    bck_io_num:   I2S_SCK,  
    ws_io_num:    I2S_WS,    
    data_out_num: -1, // not used
    data_in_num:  I2S_SD   
  };

  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pin_config);
}

#define I2S_TASK_PRI   4
#define I2S_TASK_STACK 2048

//-----Lectura I2S-----
void mic_i2s_reader_task(void* parameter) {
  mic_i2s_init();

  size_t bytes_read = 0;
  i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(int32_t), &bytes_read, portMAX_DELAY);

  while (true) {
    i2s_read(I2S_PORT, &samples, SAMPLES_SHORT * sizeof(SAMPLE_T), &bytes_read, portMAX_DELAY);

    TickType_t start_tick = xTaskGetTickCount();
    
    SAMPLE_T* int_samples = (SAMPLE_T*)&samples;
    for(int i=0; i<SAMPLES_SHORT; i++) samples[i] = MIC_CONVERT(int_samples[i]);

    sum_queue_t q;
    q.sum_sqr_SPL = MIC_EQUALIZER.filter(samples, samples, SAMPLES_SHORT);
    q.sum_sqr_weighted = WEIGHTING.filter(samples, samples, SAMPLES_SHORT);
    q.proc_ticks = xTaskGetTickCount() - start_tick;
   
    xQueueSend(samples_queue, &q, portMAX_DELAY);
  }
}

//-----Funcion envío MQTT-----
void envioMQTT (){
  if (!client.connected()) {
    reconnect();
  }
  client.loop();

  unsigned long currentMillis = millis();
    
  if (currentMillis - previousMillis >= 500) { //envia la medición cada 500ms
    previousMillis = currentMillis;
    
    char valueStr [5];
    dtostrf(Leq_dB,3,2,valueStr); //Convertir double a str, (double a convertir, cantidad de digitos de parte entera, cantidad de decimales, variable donde se guarda el str)
    
    Serial.println("Enviando: [" +  String(MEDICIONDB) + "] " + Leq_dB);
    client.publish(MEDICIONDB, valueStr); //MEDICIONDB es "/ESP32Proto/LectdB" y valueStr es la cadena de caracteres que mandamos a ese topic del broker
  }
  }

//-----Funcion reconnect MQTT-----
void reconnect() {
  uint8_t retries = 3;
  
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

void setup() {

  Serial.begin(9600);
  delay(1000); // Safety

    // Conexión WIFI
  WiFi.begin(ssid, password);
  Serial.println("Intentando conectarse a la red");
  while (WiFi.status() != WL_CONNECTED and contconexion <50) { //Cuenta hasta 50 si no se puede conectar lo cancela
    ++contconexion;
    delay(1000);
    Serial.print(".");
  }
  if (contconexion <50) {
      //para usar con ip fija
      IPAddress ip(192,168,1,156); // Casa: "192,168,1,156"; Cel: "192,168,43,184"; Sosa:"192,168,100,152";
      IPAddress gateway(192,168,1,1); //Casa: "192,168,1,1"; Cel: "192,168,82,103"; Sosa: "192,168,100,1";
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
  
  String mediciones = "/" + USERNAME + "/" + "LectdB"; 
  mediciones.toCharArray(MEDICIONDB, 50);
  
  samples_queue = xQueueCreate(8, sizeof(sum_queue_t));
  
  xTaskCreate(
  mic_i2s_reader_task,  // Función que queremos correr
  "Mic I2S Reader",    //  Descripción de la función
  I2S_TASK_STACK,      //  Tamaño del stack
  NULL,                //  Parámetros que se le pueden pasar a la función
  I2S_TASK_PRI,        //  Prioridad de la función
  NULL                 //  Task handdle  
  );

  sum_queue_t q;
  uint32_t Leq_samples = 0;
  double Leq_sum_sqr = 0;
  

  while (xQueueReceive(samples_queue, &q, portMAX_DELAY)) {

   
    double short_RMS = sqrt(double(q.sum_sqr_SPL) / SAMPLES_SHORT);
    double short_SPL_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(short_RMS / MIC_REF_AMPL);

    
    if (short_SPL_dB > MIC_OVERLOAD_DB) {
      Leq_sum_sqr = INFINITY;
    } else if (isnan(short_SPL_dB) || (short_SPL_dB < MIC_NOISE_DB)) {
      Leq_sum_sqr = -INFINITY;
    }

    // Accumulate Leq sum
    Leq_sum_sqr += q.sum_sqr_weighted;
    Leq_samples += SAMPLES_SHORT;

    // When we gather enough samples, calculate new Leq value
    if (Leq_samples >= SAMPLE_RATE * LEQ_PERIOD) {
      double Leq_RMS = sqrt(Leq_sum_sqr / Leq_samples);
      Leq_dB = MIC_OFFSET_DB + MIC_REF_DB + 20 * log10(Leq_RMS / MIC_REF_AMPL);
      Leq_sum_sqr = 0;
      Leq_samples = 0;
          
      Serial.printf("%.1f\n", Leq_dB);
      envioMQTT();
     }
    }
   }

void loop() {

}







    
