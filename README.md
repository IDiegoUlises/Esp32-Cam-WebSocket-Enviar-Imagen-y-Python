# Esp32-TCP

### Codigo  probado que funciona
```c++
#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include <vector>

const char *ssid = "Wifi Home";
const char *password = "S4m4sw3n0s"; //192.168.100.16 laptop
const IPAddress serverIP(192,168,100,16); //欲访问的地址
uint16_t serverPort = 8080;

#define maxcache 1430

WiFiClient client; //声明一个客户端对象&#xff0c;用于与服务器进行连接

//CAMERA_MODEL_AI_THINKER类型摄像头的引脚定义
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

static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sscb_sda = SIOD_GPIO_NUM,
    .pin_sscb_scl = SIOC_GPIO_NUM,
    
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,
    
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    
    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_VGA,
    .jpeg_quality = 12,
    .fb_count = 1,
};
void wifi_init()
{
    WiFi.mode(WIFI_STA);
    WiFi.setSleep(false); //关闭STA模式下wifi休眠&#xff0c;提高响应速度
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi Connected!");
    Serial.print("IP Address:");
    Serial.println(WiFi.localIP());
}
esp_err_t camera_init() {
    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        Serial.println("Camera Init Failed");
        return err;
    }
    sensor_t * s = esp_camera_sensor_get();
    //initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV2640_PID) {
    //        s-&gt;set_vflip(s, 1);//flip it back
    //        s-&gt;set_brightness(s, 1);//up the blightness just a bit
    //        s-&gt;set_contrast(s, 1);
    }
    Serial.println("Camera Init OK!");
    return ESP_OK;
}
 
void setup()
{
    Serial.begin(115200);
    wifi_init();
    camera_init();
}

void loop()
{
    Serial.println("Try To Connect TCP Server!");
    if (client.connect(serverIP, serverPort)) //尝试访问目标地址
    {
        Serial.println("Connect Tcp Server Success!");
        //client.println(&#34;Frame Begin&#34;);  //46 72 61 6D 65 20 42 65 67 69 6E // 0D 0A 代表换行  //向服务器发送数据
        while (1){       
          camera_fb_t * fb = esp_camera_fb_get();
          uint8_t * temp = fb->buf; //这个是为了保存一个地址&#xff0c;在摄像头数据发送完毕后需要返回&#xff0c;否则会出现板子发送一段时间后自动重启&#xff0c;不断重复
          if (!fb)
          {
              Serial.println("Camera Capture Failed");
          }
          else
          { 
            //先发送Frame Begin 表示开始发送图片 然后将图片数据分包发送 每次发送1430 余数最后发送 
            //完毕后发送结束标志 Frame Over 表示一张图片发送完毕 
            client.print("Frame Begin"); //一张图片的起始标志
            // 将图片数据分段发送
            int leng = fb->len;
            int timess = leng/maxcache;
            int extra = leng%maxcache;
            for(int j = 0; j<timess;j++)
            {
              client.write(fb->buf, maxcache); 
              for(int i =0;i< maxcache;i++)
              {
                fb->buf++;
              }
            }
            client.write(fb->buf, extra);
            client.print("Frame Over");      // 一张图片的结束标志
            Serial.print("This Frame Length:");
            Serial.print(fb->len);
            Serial.println("Succes To Send Image For TCP");
            //return the frame buffer back to the driver for reuse
            fb->buf = temp; //将当时保存的指针重新返还
            esp_camera_fb_return(fb);  //这一步在发送完毕后要执行&#xff0c;具体作用还未可知。        
          }
          delay(20);//短暂延时 增加数据传输可靠性
        }
        /*
        while (client.connected() || client.available()) //如果已连接或有收到的未读取的数据
        {
            if (client.available()) //如果有数据可读取
            {
                String line &#61; client.readStringUntil(&#39;\n&#39;); //读取数据到换行符
                Serial.print(&#34;ReceiveData&#xff1a;&#34;);
                Serial.println(line);
                client.print(&#34;--From ESP32--:Hello Server!&#34;);    
            }
        }
        Serial.println(&#34;close connect!&#34;);
        client.stop(); //关闭客户端
        */
    }
    else
    {
        Serial.println("Connect To Tcp Server Failed!After 10 Seconds Try Again!");
        client.stop(); //关闭客户端
    }
    delay(10000);
}
```

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
### ESP32 CODIGO

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
