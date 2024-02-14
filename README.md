# Esp32 TCP

### Codigo Esp32 Cliente

```C++
// Incluir las librerías necesarias
#include "esp_camera.h"
#include <WiFi.h>

// Definir los pines del módulo ESP32 CAM
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

// Definir los parámetros de la red Wi-Fi
const char* ssid = "Wifi Home"; // Cambiar por el nombre de la red Wi-Fi
const char* password = "S4m4sw3n0s"; // Cambiar por la contraseña de la red Wi-Fi

// Definir los parámetros del servidor socket
const char* host = "192.168.100.16"; // Cambiar por la dirección IP del servidor socket
const int port = 8080; // Cambiar por el puerto del servidor socket

// Crear un objeto WiFiClient
WiFiClient client;

// Configurar la cámara
void setupCamera() {
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

  // Inicializar la cámara
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

// Conectar a la red Wi-Fi
void connectWiFi() {
  // Iniciar la conexión Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Esperar hasta que la conexión sea exitosa
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Mostrar la dirección IP asignada
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Conectar al servidor socket
void connectSocket() {
  // Iniciar la conexión al servidor socket
  Serial.print("Connecting to ");
  Serial.println(host);

  // Esperar hasta que la conexión sea exitosa
  while (!client.connect(host, port)) {
    delay(500);
    Serial.print(".");
  }

  // Mostrar que la conexión se ha establecido
  Serial.println("");
  Serial.println("Connected to the server.");
}

// Capturar y enviar una foto
void sendPhoto() {
  // Obtener el búfer de la imagen capturada
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Enviar el búfer al servidor socket
  Serial.println("Sending photo...");
  client.write(fb->buf, fb->len);
  Serial.println("Photo sent.");

  // Liberar el búfer
  esp_camera_fb_return(fb);
}

// Configurar el puerto serie y la cámara
void setup() {
  Serial.begin(115200);
  setupCamera();
}

// Conectar a la red Wi-Fi, al servidor socket y enviar una foto
void loop() {
  connectWiFi();
  connectSocket();
  sendPhoto();
  client.stop();
  WiFi.disconnect();
  delay(10000); // Esperar 10 segundos antes de repetir el proceso
}
```

### Codigo en Python Servidor
```python
import socket

# Crea un objeto socket
server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Asigna una dirección y un puerto al socket
server_address = ('192.168.1.16', 8080)
server_socket.bind(server_address)

# Escucha las conexiones entrantes
server_socket.listen()

# Acepta la conexión entrante
client_socket, client_address = server_socket.accept()

# Recibe los datos enviados por el cliente
data = b'' # Crea un buffer vacío para almacenar los datos
while True:
    chunk = client_socket.recv(1024) # Recibe un trozo de datos
    if not chunk: # Si no hay más datos, termina el bucle
        break
    data += chunk # Añade el trozo al buffer

# Guarda los datos recibidos en un archivo de imagen
with open('imagen.png', 'wb') as f:
    f.write(data)


# Cierra la conexión
client_socket.close()
server_socket.close()
```

# Codigo 2 en fases de pruebas
lo que hace se comporta como un servidor recibe la imagen luego recibe el mensaje EOT(IMPORTANTE SIN ESTO NO FUNCIONA) luego tiene que borrar el mensaje EOT que se recibe en caso de no borrarla al escribir la imagen al abrirla se mostrata como dañada
```python
import socket
import io

# Crea un socket para escuchar conexiones
s = socket.socket()
# Asigna el socket a una dirección IP y un puerto
s.bind(('192.168.1.16', 8080))
# Pon el socket en modo escucha
s.listen(1)

# Acepta una conexión de un cliente
conn, addr = s.accept()
print('Connected by', addr)

# Crea un buffer para almacenar los datos recibidos
buffer = b''

# Recibe los datos del cliente hasta que se reciba un EOT
while True:
    # Recibe un bloque de datos
    data = conn.recv(1024)
    # Si no hay datos, termina el bucle
    if not data:
        break
    # Añade los datos al buffer
    buffer += data
    # Si el último byte es un EOT, termina el bucle
    if data[-1] == 4:
        break

# Encuentra la posición del último byte del archivo
end = buffer.rfind(b'\xff\xd9')
# Extrae solo los datos de la imagen
image_data = buffer[:end+2]
# Guarda el buffer en un archivo de imagen
with io.open('imagen-recibida.png', 'wb') as f:
    f.write(image_data)

# Cierra el socket
conn.close()
s.close()
```

### Codigo de arduino
Este codigo captura una foto de la camara, desactiva la brown out para evitar problemas luego captura la foto la envia por socket y envia un mensaje EOT(IMPORTANTE) y cierra la conexion
```c++
// Incluir las librerías necesarias
#include "esp_camera.h"
#include <WiFi.h>

#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

// Definir los pines del módulo ESP32 CAM
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

// Definir los parámetros de la red Wi-Fi
const char* ssid = "Wifi Home 2.4G"; // Cambiar por el nombre de la red Wi-Fi
const char* password = "S4m4sw3n0s"; // Cambiar por la contraseña de la red Wi-Fi

// Definir los parámetros del servidor socket
const char* host = "192.168.1.16"; // Cambiar por la dirección IP del servidor socket
const int port = 8080; // Cambiar por el puerto del servidor socket

// Crear un objeto WiFiClient
WiFiClient client;

// Configurar la cámara
void setupCamera() {
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

  // Inicializar la cámara
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
}

// Conectar a la red Wi-Fi
void connectWiFi() {
  // Iniciar la conexión Wi-Fi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // Esperar hasta que la conexión sea exitosa
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // Mostrar la dirección IP asignada
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

// Conectar al servidor socket
void connectSocket() {
  // Iniciar la conexión al servidor socket
  Serial.print("Connecting to ");
  Serial.println(host);

  // Esperar hasta que la conexión sea exitosa
  while (!client.connect(host, port)) {
    delay(500);
    Serial.print(".");
  }

  // Mostrar que la conexión se ha establecido
  Serial.println("");
  Serial.println("Connected to the server.");
}

// Capturar y enviar una foto
void sendPhoto() {
  // Obtener el búfer de la imagen capturada
  camera_fb_t * fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  }

  // Enviar el búfer al servidor socket
  Serial.println("Sending photo...");
  client.write(fb->buf, fb->len);
  Serial.println("Photo sent.");

  client.print('\x04');
  Serial.println("Mensaje EOT enviado");

  // Liberar el búfer
  esp_camera_fb_return(fb);
}

// Configurar el puerto serie y la cámara
void setup() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable   detector
  Serial.begin(115200);
  setupCamera();
}

// Conectar a la red Wi-Fi, al servidor socket y enviar una foto
void loop() {
  connectWiFi();
  connectSocket();
  sendPhoto();
  client.stop();
  WiFi.disconnect();
  delay(10000); // Esperar 10 segundos antes de repetir el proceso
}
```

# NUEVO CODIGO PORQUE LO ANTERIOR FALLO TODO ESTE CODIGO UTILIZA WEBSERVER y funciona pero con algunos detalles la resolucion es muy baja y solo se puede ver la imagen es escala de grises

### Codigo arduino
```c++
#include "esp_camera.h"
#include <WiFi.h>
#include <WebSocketsServer.h>

// Select camera model
#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
#include "esp_camera.h"

const char* ssid     = "Wifi Home 2.4G";     // input your wifi name
const char* password = "S4m4sw3n0s";   // input your wifi passwords

// Definir los pines del módulo ESP32 CAM
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


// Globals
WebSocketsServer webSocket = WebSocketsServer(80);
 
// Called when receiving any WebSocket message
void onWebSocketEvent(uint8_t num,
                      WStype_t type,
                      uint8_t * payload,
                      size_t length) {

  // Figure out the type of WebSocket event
  switch(type) {

    // Client has disconnected
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;

    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connection from ", num);
        Serial.println(ip.toString());
      }
      break;

    // Echo text message back to client
    case WStype_TEXT:
      Serial.printf("[%u] Text: %s\n", num, payload);
      Serial.println((char*)payload);

      if (String((char*)payload) == "capture")
      {
        Serial.println("Capture Command Received - capturing frame");

        camera_fb_t * fb = NULL;
        fb = esp_camera_fb_get(); // get image... part of work-around to get latest image
        esp_camera_fb_return(fb); // return fb... part of work-around to get latest image
        
        fb = NULL;
        fb = esp_camera_fb_get(); // get fresh image
        size_t fbsize = fb->len;
        Serial.println(fbsize);
        Serial.println("Image captured. Returning frame buffer data.");
        webSocket.sendBIN(num, fb->buf, fbsize);
        esp_camera_fb_return(fb);
        Serial.println("Done");
      } else
      {
        webSocket.sendTXT(num, payload);
      }
      break;

    // For everything else: do nothing
    case WStype_BIN:
     // Serial.printf("[%u] get binary length: %u\n", num, length);
     // hexdump(payload, length);

      // send message to client
      // webSocket.sendBIN(num, payload, length);
     // break;
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("Connecting...");

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
  config.pixel_format = PIXFORMAT_GRAYSCALE;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  //if(psramFound()){
  //  config.frame_size = FRAMESIZE_UXGA;
  //  config.jpeg_quality = 10;
  //  config.fb_count = 2;
  //} else {
  config.frame_size = FRAMESIZE_QVGA; //FRAMESIZE_96X96; //FRAMESIZE_QQVGA; //FRAMESIZE_SVGA;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  //}

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Msg: Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_QVGA);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  Serial.print("Msg: Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

void loop() {
  // Look for and handle WebSocket data
  webSocket.loop();
}
```

### Codigo en python

```python
import websocket # pip install websocket-client
import numpy as np
import matplotlib.pyplot as plt
 
# Image dimensions - this needs to match our setting on the ESP32 CAM board
IMG_X = 320 # QVGA 320x240
IMG_Y = 240

ws = websocket.WebSocket()
ws.connect("ws://192.168.1.3") # Use the IP address from the ESP32 board - printed to the serial monitor
#192.168.1.3 IP DE ESP32

while(1):
    # Ask the user for some input and transmit it
    str = input("Say something: ")
    if (str == "exit"):
        break
    
    # sent the input string across the socket
    ws.send(str)

    # Wait for server to respond and print it
    if (str == "capture"):
        binResp = ws.recv_frame() # receiving binary image data (we assume grey scale) from camera
  
        # we are going to store the received binary image data in an array... 
        img_array = np.zeros((IMG_Y,IMG_X))
        x = 0
        y = 0
        count = 0

        # traverse the captured data one byte at a time and populate image array
        for byte in bytearray(binResp.data):
           #print (byte)
           y = count // IMG_X
           x = count % IMG_X
           img_array[y,x] = byte
           count = count + 1

        # display captured image
        plt.figure('Capture Image 1')
        plt.imshow(img_array)
        plt.set_cmap('gray')
        plt.show(block=False)

    else:
       result = ws.recv()
       print("Received: " + result)

# Gracefully close WebSocket connection
ws.close()
```
# Nuevo codigo mejorado pero no optimizado pero funciona esp32 envia la foto por web socket y el cliente python recibe la foto a color con una buena resolucion

### Esp32 Cam Codigo
```C++
#include "esp_camera.h"
#include <WiFi.h>
#include <WebSocketsServer.h>

// Select camera model
#define CAMERA_MODEL_WROVER_KIT // Has PSRAM
#include "esp_camera.h"

const char* ssid     = "Wifi Home 2.4G";     // input your wifi name
const char* password = "S4m4sw3n0s";   // input your wifi passwords

// Definir los pines del módulo ESP32 CAM
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


// Globals
WebSocketsServer webSocket = WebSocketsServer(80);
 
// Called when receiving any WebSocket message
void onWebSocketEvent(uint8_t num,
                      WStype_t type,
                      uint8_t * payload,
                      size_t length) {

  // Figure out the type of WebSocket event
  switch(type) {

    // Client has disconnected
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\n", num);
      break;

    // New client has connected
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connection from ", num);
        Serial.println(ip.toString());
      }
      break;

    // Echo text message back to client
    case WStype_TEXT:
      Serial.printf("[%u] Text: %s\n", num, payload);
      Serial.println((char*)payload);

      if (String((char*)payload) == "capture")
      {
        Serial.println("Capture Command Received - capturing frame");

        camera_fb_t * fb = NULL;
        fb = esp_camera_fb_get(); // get image... part of work-around to get latest image
        esp_camera_fb_return(fb); // return fb... part of work-around to get latest image
        
        fb = NULL;
        fb = esp_camera_fb_get(); // get fresh image
        size_t fbsize = fb->len;
        Serial.println(fbsize);
        Serial.println("Image captured. Returning frame buffer data.");
        webSocket.sendBIN(num, fb->buf, fbsize);
        esp_camera_fb_return(fb);
        Serial.println("Done");
      } else
      {
        webSocket.sendTXT(num, payload);
      }
      break;

    // For everything else: do nothing
    case WStype_BIN:
     // Serial.printf("[%u] get binary length: %u\n", num, length);
     // hexdump(payload, length);

      // send message to client
      // webSocket.sendBIN(num, payload, length);
     // break;
    case WStype_ERROR:
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
    default:
      break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println("Connecting...");

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
  config.pixel_format = PIXFORMAT_JPEG;
  
  // if PSRAM IC present, init with UXGA resolution and higher JPEG quality
  //                      for larger pre-allocated frame buffer.
  //if(psramFound()){
  //  config.frame_size = FRAMESIZE_UXGA;
  //  config.jpeg_quality = 10;
  //  config.fb_count = 2;
  //} else {
  config.frame_size = FRAMESIZE_SVGA; //FRAMESIZE_96X96; //FRAMESIZE_QQVGA; //FRAMESIZE_SVGA;
  config.jpeg_quality = 10;
  config.fb_count = 1;
  //}

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Msg: Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t * s = esp_camera_sensor_get();
  // drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_SVGA);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  Serial.print("Msg: Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  // Start WebSocket server and assign callback
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
}

void loop() {
  // Look for and handle WebSocket data
  webSocket.loop();
}
```

### Python cliente codigo
```python
import websockets
import asyncio
import aiofiles

# The main function that will handle connection and communication
# with the server
async def ws_client():
    print("WebSocket: Client Connected.")
    # Connect to the server
        name = input("capture or exit: ")
    url = "ws://192.168.1.3:80"
    async with websockets.connect(url) as ws:

        if name == 'exit':
            exit()

        #age = input("Your Age: ")
        # Send values to the server
        await ws.send(f"{name}")
        #await ws.send(f"{age}")

        # Stay alive forever, listen to incoming msgs
        while True:
            # Receive binary data
            data = await ws.recv()
            # Save the binary data to a file
            async with aiofiles.open('received_image.jpg', 'wb') as out_file:
                await out_file.write(data)
            print("Image received and saved.")
            break

asyncio.run(ws_client())
# Start the connection
```

# Codigo anterior de web socket server pero en desarollo porque se intenta optimizar EXPERIMENTAL

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

  // Esperar hasta que la conexión sea exitosa
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  // Mostrar la dirección IP asignada
  Serial.println("");
  Serial.println("WiFi conectado.");
  Serial.println("IP Direccion: ");
  Serial.println(WiFi.localIP());
}

// Configurar la cámara
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
      Serial.println("Cliente desconectado", num);
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
