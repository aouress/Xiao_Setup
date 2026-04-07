#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"
#include "pinout.h"
#include "apriltag.h"

WebServer server(80);

#define CAMERA_MODEL_XIAO_ESP32S3

// WiFi credentials
const char* ssid = "aidens-iphone";
const char* password = "aiden2005";

void startCameraServer();

// Stream handler
void streamHandler() {
  WiFiClient client = server.client();
  String response = "HTTP/1.1 200 OK\r\n";
  response += "Content-Type: multipart/x-mixed-replace; boundary=frame\r\n\r\n";
  server.sendContent(response);

  while (true) {
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      continue;
    }

    String head = "--frame\r\nContent-Type: image/jpeg\r\nContent-Length: " + String(fb->len) + "\r\n\r\n";
    client.print(head);
    client.write(fb->buf, fb->len);
    client.print("\r\n");
    esp_camera_fb_return(fb);

    // Exit loop if client disconnects
    if (!client.connected()) break;
    delay(30); // ~30 fps
  }
}

void startCameraServer() {
  server.on("/stream", HTTP_GET, streamHandler);
  server.begin();
  Serial.println("HTTP server started. Stream at /stream");
}

void setup() {
  Serial.begin(115200);
  while(!Serial); // wait for enumeration
  Serial.setDebugOutput(true);
  Serial.println();

  static camera_config_t config = {
    .pin_pwdn       = PWDN_GPIO_NUM,
    .pin_reset      = RESET_GPIO_NUM,
    .pin_xclk       = XCLK_GPIO_NUM,
    .pin_sccb_sda   = SIOD_GPIO_NUM,
    .pin_sccb_scl   = SIOC_GPIO_NUM,
    .pin_d7         = Y9_GPIO_NUM,
    .pin_d6         = Y8_GPIO_NUM,
    .pin_d5         = Y7_GPIO_NUM,
    .pin_d4         = Y6_GPIO_NUM,
    .pin_d3         = Y5_GPIO_NUM,
    .pin_d2         = Y4_GPIO_NUM,
    .pin_d1         = Y3_GPIO_NUM,
    .pin_d0         = Y2_GPIO_NUM,
    .pin_vsync      = VSYNC_GPIO_NUM,
    .pin_href       = HREF_GPIO_NUM,
    .pin_pclk       = PCLK_GPIO_NUM,

    .xclk_freq_hz   = 20000000, // The clock frequency of the image sensor
    
    .pixel_format   = PIXFORMAT_JPEG,  // The pixel format of the image: PIXFORMAT_ + YUV422|GRAYSCALE|RGB565|JPEG
    .frame_size     = FRAMESIZE_UXGA,  // The resolution size of the image: FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    .jpeg_quality   = 12, // The quality of the JPEG image, ranging from 0 to 63.
    .fb_count       = 2, // The number of frame buffers to use
    .fb_location = CAMERA_FB_IN_PSRAM, // Set the frame buffer storage location
    .grab_mode      = CAMERA_GRAB_WHEN_EMPTY //  The image capture mode.
  };

  // initialize the camera
  esp_err_t err = esp_camera_init(&config);
  if(err != ESP_OK)
  {
    Serial.printf("Camera init failed with error 0x%x\n", err);
    return;
  }

  // Camera reconfiguration 
  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if(config.pixel_format == PIXFORMAT_JPEG){
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

  // Connect to wifi
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while(WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }

  Serial.println();
  Serial.print("WiFi connected at IP address: ");
  Serial.println(WiFi.localIP());

  // Start streaming server
  startCameraServer();

}

void loop() {
  server.handleClient();
}