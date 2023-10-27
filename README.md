# Esp32-TCP

### Codigo no probado pero compila
```c++
#include "WiFi.h"

char *host = "192.168.100.95";
uint16_t port = 9000;

#include "esp_camera.h"
#include "FS.h"
#include "SD_MMC.h"

//Declaracion de pins para camara MODEL_AI_THINKER
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

void setup()
{
  //Inicia el puerto serial
  Serial.begin(115200);

  //Configuracion de la camara
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

  //Verifica si es compatible con PSRAM y elige la configuracion adecuada
  if (psramFound())
  {
    config.frame_size = FRAMESIZE_VGA; //Tamaño de la foto, se pueden elegir QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA, mientras mas pequeña mejor resolucion
    config.jpeg_quality = 10; //10-63 mientras menor el numero mejor calidad
    config.fb_count = 2;
  }

  else
  {
    config.frame_size = FRAMESIZE_VGA; //Tamaño de la foto, se pueden elegir QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA, mientras mas pequeña mejor resolucion
    config.jpeg_quality = 10; //10-63 mientras menor el numero mejor calidad
    config.fb_count = 2;
  }

  //Inicializar la camera con la configuracion
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK)
  {
    Serial.println("Camera no inicializada" + err);
    return;
  }

  //SD incializada correctamente
  if (!SD_MMC.begin())
  {
    Serial.println("SD Card Montada incorrectamente");
    return;
  }

  uint8_t cardType = SD_MMC.cardType();

  //En caso que no se detecte una tarjeta sd
  if (cardType == CARD_NONE)
  {
    Serial.println("No SD Card introducida");
    return;
  }

}
void loop()
{
  WiFiClient client;

  if (!client.connect(host, port)) {

    Serial.println("Connection to host failed");

    delay(1000);
    return;
  }

  Serial.println("Connected to server successful!");

  // capture camera frame
  camera_fb_t *fb = esp_camera_fb_get();
  if (!fb) {
    Serial.println("Camera capture failed");
    return;
  } else {
    Serial.println("Camera capture successful!");
  }
  const char *data = (const char *)fb->buf;
  // Image metadata.  Yes it should be cleaned up to use printf if the function is available
  Serial.print("Size of image:");
  Serial.println(fb->len);
  Serial.print("Shape->width:");
  Serial.print(fb->width);
  Serial.print("height:");
  Serial.println(fb->height);
  client.print("Shape->width:");
  client.print(fb->width);
  client.print("height:");
  client.println(fb->height);
  // Give the server a chance to receive the information before sending an acknowledgement.
  delay(1000);
  getResponse(client);
  Serial.print(data);
  client.write(data, fb->len);
  esp_camera_fb_return(fb);

  Serial.println("Disconnecting...");
  client.stop();

  delay(2000);
}

void getResponse(WiFiClient client) {
  byte buffer[8] = { NULL };
  while (client.available() > 0 || buffer[0] == NULL) {
    int len = client.available();
    Serial.println("Len" + len);
    if (len > 8) len = 8;
    client.read(buffer, len);
   /*
    if (printReceivedData) {
      Serial.write(buffer, len); // show in the serial monitor (slows some boards)
      Serial.println("");
    }
    */
  }
}

```
