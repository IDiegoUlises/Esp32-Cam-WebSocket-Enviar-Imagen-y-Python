# Esp32-TCP


### Codigo en Python
```python
import socket
import threading
import time
import numpy as np
import cv2

begin_data = b'Frame Begin'
end_data = b'Frame Over'

#接收数据
# ESP32发送一张照片的流程
# 先发送Frame Begin 表示开始发送图片 然后将图片数据分包发送 每次发送1430 余数最后发送
# 完毕后发送结束标志 Frame Over 表示一张图片发送完毕
# 1430 来自ESP32cam发送的一个包大小为1430 接收到数据 data格式为b&#39;&#39;
def handle_sock(sock, addr):
    temp_data = b''#61; b&#39;&#39;
    t1 = int(round(time.time() * 1000))
    while True:
        data = sock.recv(1430)
        # 如果这一帧数据包的开头是 b&#39;Frame Begin&#39; 则是一张图片的开始
        if data[0:len(begin_data)] == begin_data:
            # 将这一帧数据包的开始标志信息&#xff08;b&#39;Frame Begin&#39;&#xff09;清除   因为他不属于图片数据
            data = data[len(begin_data):len(data)]
            # 判断这一帧数据流是不是最后一个帧 最后一针数据的结尾时b&#39;Frame Over&#39;
            while data[-len(end_data):] != end_data:
                temp_data = temp_data + data  # 不是结束的包 讲数据添加进temp_data
                data = sock.recv(1430)# 继续接受数据 直到接受的数据包包含b&#39;Frame Over&#39; 表示是这张图片的最后一针
            # 判断为最后一个包 将数据去除 结束标志信息 b&#39;Frame Over&#39;
            temp_data = temp_data + data[0:(len(data) - len(end_data))]  # 将多余的&#xff08;\r\nFrame Over&#xff09;去掉 其他放入temp_data
            # 显示图片
            receive_data = np.frombuffer(temp_data, dtype = 'uint8')  # 将获取到的字符流数据转换成1维数组
            r_img = cv2.imdecode(receive_data, cv2.IMREAD_COLOR)  # 将数组解码成图像
            r_img = r_img.reshape(480, 640, 3)
            t2 = int(round(time.time() * 1000))
            fps = 1000//(t2-t1) #duda si esta bien o se borra los comentarios
            cv2.putText(r_img, 'FPS' + str(fps), (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)
            cv2.imshow('server_frame', r_img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            t1 = t2
            print('接收到的数据包大小' + str(len(temp_data)))  # 显示该张照片数据大小
            temp_data = b'' #39;&#39;  # 清空数据 便于下一章照片使用

server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server.bind(('192.168.100.16', 8080)) #192.168.100.16 laptop -- #esp32 192.168.100.95 
server.listen(5)
CONNECTION_LIST = []

#主线程循环接收客户端连接
while True:
    sock, addr = server.accept()
    CONNECTION_LIST.append(sock)
    print('Connect--{}'.format(addr))
    #连接成功后开一个线程用于处理客户端
    client_thread = threading.Thread(target=handle_sock, args=(sock, addr))
    client_thread.start()
```

### CODIGO CREADO CON INTELIGENCIA ARTIFICIAL QUE FUNCIONA
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
server_address = ('192.168.100.16', 8080)
server_socket.bind(server_address)

# Escucha las conexiones entrantes
server_socket.listen()

# Acepta la conexión entrante
client_socket, client_address = server_socket.accept()

# Recibe los datos enviados por el cliente
#data = client_socket.recv(1024)

# Recibe los datos enviados por el cliente
data = b'' # Crea un buffer vacío para almacenar los datos
while True:
    chunk = client_socket.recv(1024) # Recibe un trozo de datos
    if not chunk: # Si no hay más datos, termina el bucle
        break
    data += chunk # Añade el trozo al buffer

# Guarda los datos recibidos en un archivo de imagen
with open('image.png', 'wb') as f:
    f.write(data)


# Cierra la conexión
client_socket.close()
server_socket.close()
```
