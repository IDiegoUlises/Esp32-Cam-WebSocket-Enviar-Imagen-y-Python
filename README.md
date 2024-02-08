# Esp32-TCP

### Esp32 codigo

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

### Codigo en python servidor
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
