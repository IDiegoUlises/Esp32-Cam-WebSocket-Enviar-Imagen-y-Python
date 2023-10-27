# Esp32-TCP

### Codigo no probado pero compila
```c++
#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include <vector>

const char *ssid = "Wifi Home";
const char *password = "contra";
const IPAddress serverIP(192,168,0,2); //欲访问的地址
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
