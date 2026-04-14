#include "esp_camera.h"
#include <Arduino.h>
#include "pinout.h"
#include "tagStandard41h12.h"
#include "tag36h11.h"
#include <apriltag.h>
#include <apriltag_pose.h>
#include "apriltag.h"
#include "tag16h5.h"
#include "tag25h9.h"
#include "tag36h11.h"
#include "common/image_u8.h"
#include "common/zarray.h"

void setup()
{
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
    
    .pixel_format   = PIXFORMAT_GRAYSCALE,  // The pixel format of the image: PIXFORMAT_ + YUV422|GRAYSCALE|RGB565|JPEG
    .frame_size     = FRAMESIZE_QVGA,  // The resolution size of the image: FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    .jpeg_quality   = 24, // The quality of the JPEG image, ranging from 0 to 63.
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
        Serial.begin(115200);
        while(!Serial); // wait for enumeration
        Serial.setDebugOutput(true);
        Serial.println();

        apriltag_detector_t *td = apriltag_detector_create();

        // Register all families — the output will tell you which one matched
        apriltag_family_t *tf1 = tagStandard41h12_create();
        //apriltag_family_t *tf2 = tag25h9_create();
        //apriltag_family_t *tf3 = tag36h11_create();
        apriltag_detector_add_family(td, tf1); // add_family uses hamming=2 internally
        //apriltag_detector_add_family(td, tf2);
        //apriltag_detector_add_family(td, tf3);

        // These values were found by grid-searching against real test images:
        //   quad_sigma=1.0  smooths noise without destroying thin borders
        //   min_white_black_diff=5  catches low-contrast / backlit tags
        //   quad_decimate=1.0  keeps full resolution (important for small tags)
        td->quad_decimate = 1.0f;
        td->quad_sigma = 0.0f;
        td->nthreads = 1;
        td->debug = 0;
        td->refine_edges = 1;
        td->qtp.min_white_black_diff = 5;

        Serial.println("done");
        Serial.print("Memory available in PSRAM: ");
        Serial.println(ESP.getFreePsram());
        Serial.println("Start detecting...");

  int total_detected;

  // Main detecting loop (we just ignore the loop() function)
  while (true) {
    // Get a frame from camera
    Serial.println("Before Camera get");
    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Failed to get frame! Retrying...");
      continue;
    }
    
    // Start marker (lets Python sync)
    Serial.println("FRAME_START");

    // Send raw grayscale image
    Serial.write(fb->buf, fb->len);

    // End marker
    Serial.println("FRAME_END");

    Serial.println("After camera get");

    // Convert our framebuffer to detector's input format
    image_u8_t im = {
      .width = fb->width,
      .height = fb->height,
      .stride = fb->width,
      .buf = fb->buf
    };


    zarray_t *detections = apriltag_detector_detect(td, &im);
    //image_u8_destroy(&im);
    int n = zarray_size(detections);
    Serial.print("Detected tag(s): ");
    Serial.println(n);
    total_detected += n;
    

    for (int j = 0; j < n; j++) {
      apriltag_detection_t *det;
      zarray_get(detections, j, &det);
      Serial.printf("  [%d] family=%-10s id=%-3d hamming=%d margin=%.2f\n",
      j, det->family->name, det->id, det->hamming, det->decision_margin);
      Serial.printf("centre: (%.1f, %.1f)\n", det->c[0], det->c[1]);
      Serial.printf("corners : (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f) (%.1f,%.1f)\n",
      det->p[0][0], det->p[0][1], det->p[1][0], det->p[1][1],
      det->p[2][0], det->p[2][1], det->p[3][0], det->p[3][1]);
    }

    Serial.println("Destroying detections");
    apriltag_detections_destroy(detections);
    

    // --- Cleanup -----------------------------------------------------------
    // apriltag_detector_destroy(td);
    // tag16h5_destroy(tf1);
    //tag25h9_destroy(tf2);
    //tag36h11_destroy(tf3);

    //Serial.println("");

    // Cleaning up
    Serial.print("Memory available in PSRAM: ");
    Serial.println(ESP.getFreePsram());
    //Serial.println("Cleaning up... ");

    // Free detection result object
    //apriltag_detections_destroy(detections);

    // Return camera framebuffer to the camera driver
    esp_camera_fb_return(fb);

    // Display time needed per frame
    // And frame count (if CAP_TO_SD defined)
    float t =  timeprofile_total_utime(td->tp) / 1.0E3;
    //Serial.Serial.println("t: %12.3f\n", t);
  }
}

void loop()
{

}