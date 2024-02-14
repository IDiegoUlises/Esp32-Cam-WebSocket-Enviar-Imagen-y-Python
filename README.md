# Esp32 Web Socket Como Servidor Envia una imagen a un Cliente Python

### Esp32 Cam 
```c++
#include <WiFi.h>
#include <WebSocketsServer.h>
#include "esp_camera.h"

//Credenciales WiFi
const char* ssid     = "Wifi Home 2.4G";
const char* password = "S4m4sw3n0s";

//Definir los pines del módulo ESP32 CAM
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

//Crea una instancia del servidor WebSocket en el puerto 80
//Esto significa que el ESP32 está listo para recibir conexiones WebSocket en ese puerto
WebSocketsServer webSocket = WebSocketsServer(80);

void ConectarWifi()
{
  //Iniciar la conexión Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Conectando ");
  Serial.println(ssid);

  //Esperar hasta que la conexión sea exitosa
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  //Mostrar la dirección IP asignada
  Serial.println("");
  Serial.println("WiFi conectado.");
  Serial.println("IP Direccion: ");
  Serial.println(WiFi.localIP());
}

//Configurar la cámara
void ConfigurarCamara()
{
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG; // Usar formato JPEG
  config.frame_size = FRAMESIZE_SVGA; // Usar resolución SVGA
  config.jpeg_quality = 10; // Calidad de la imagen (1-63)
  config.fb_count = 1;

  //Inicializar la cámara
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Inicializar la camara a fallado", err);
    return;
  }
}

//Una funcion que define que se ejecuta cuando sucede un evento
void WebSocketEvento(uint8_t num,
                     WStype_t type,
                     uint8_t * payload,
                     size_t length) {

  //Se ejecuta dependiento de los eventos del WebSocket
  switch (type)
  {
    //Cuando un cliente se desconecta
    case WStype_DISCONNECTED:
      Serial.println("Cliente desconectado");
      break;

    //Se ejecuta cuando un cliente se conecta exitosamente
    case WStype_CONNECTED:
      {
        camera_fb_t * fb = esp_camera_fb_get(); // Obtener la imagen (parte de la solución para obtener la imagen más reciente)
        esp_camera_fb_return(fb); //Liberar el búfer de fotogramas (parte de la solución para obtener la imagen más reciente)

        fb = esp_camera_fb_get(); //Obtener una imagen fresca
        size_t fbsize = fb->len;
        Serial.println(fbsize);
        Serial.println("Imagen capturada. Enviando datos del búfer de fotogramas.");
        webSocket.sendBIN(num, fb->buf, fbsize); //Envia los datos en binario
        esp_camera_fb_return(fb);
        Serial.println("Listo");
      }
      break;

    //En el caso de recibir un mensaje tipo texto del cliente
    case WStype_TEXT:
      Serial.printf("En el caso de recibir un mensaje de texto", num, payload);
      Serial.println((char*)payload);

      /* Descomentar estas lineas en caso que al recibir un mensaje especifico del cliente se necesite realizar una accion
        if (String((char*)payload) == "Texto")
        {
        //Accion a ejecutar
        }
      */
      break;

    //En el caso de recibir datos en binario del cliente
    case WStype_BIN:

    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}

void setup()
{
  //Inicia el puerto serial
  Serial.begin(115200);

  //Configura la camara
  ConfigurarCamara();

  //Funcion establecer conexion WiFi
  ConectarWifi();

  //Empieza el servidor WebSocket
  webSocket.begin();

  //Se esta definiendo una funcion callback que indica que accion se ejecutara cuando suceda un evento
  webSocket.onEvent(WebSocketEvento);
}

void loop()
{
  //Gestiona los eventos y procesa los datos en una conexión WebSocket
  webSocket.loop();
}
```

### Codigo en Python
```python
import websockets
import asyncio
import aiofiles

#Funcion principal que tiene el manejador handle y realiza la comunicacion
async def ws_client():
    print("WebSocket: Cliente conectado.")
    
    #Direccion IP del cliente y el puerto asignado
    url = "ws://192.168.1.3:80"

    #Es una funcion asincrona que establece una conexión a un servidor WebSocket en la URL proporcionada
    async with websockets.connect(url) as ws:

        #Escucha los mensajes entrantes
        while True:
            #Recibe los datos
            data = await ws.recv()
            #Guarda los datos de forma asincrona en binario en un archivo jpg
            async with aiofiles.open('imagen.jpg', 'wb') as out_file:
                await out_file.write(data)
            print("imagen guardada con exito.")
            break

#Empieza la conexion
asyncio.run(ws_client())
```
### Librerias Necesarias Para Python
* ``` pip install websockets ```
* ``` pip install aiofiles ```


