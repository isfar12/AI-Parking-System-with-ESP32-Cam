/* Edge Impulse Arduino examples
 * Copyright (c) 2022 EdgeImpulse Inc.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restr    // Handle web server requests
    server.handleClient();
    
    // Handle servo timer (check if servo needs to return to base position)
    servo_handle_timer();
    
    // Handle IR sensor for automatic gate control
    gate_control_system();

    // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...g without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

// MODIFIED: Changed from JPEG to RGB565 format for improved AI detection accuracy
// RGB565 provides better image quality without JPEG compression artifacts
// This improves car and coin detection performance for the trained Edge Impulse model

// These sketches are tested with 2.0.4 ESP32 Arduino Core
// https://github.com/espressif/arduino-esp32/releases/tag/2.0.4

/* Includes ---------------------------------------------------------------- */
#include <isfar12-project-1_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

#include "esp_camera.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>

// Select camera model - find more camera models in camera_pins.h file here
// https://github.com/espressif/arduino-esp32/blob/master/libraries/ESP32/examples/Camera/CameraWebServer/camera_pins.h

// #define CAMERA_MODEL_ESP_EYE // Has PSRAM
#define CAMERA_MODEL_AI_THINKER // Has PSRAM

#if defined(CAMERA_MODEL_ESP_EYE)
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    4
#define SIOD_GPIO_NUM    18
#define SIOC_GPIO_NUM    23

#define Y9_GPIO_NUM      36
#define Y8_GPIO_NUM      37
#define Y7_GPIO_NUM      38
#define Y6_GPIO_NUM      39
#define Y5_GPIO_NUM      35
#define Y4_GPIO_NUM      14
#define Y3_GPIO_NUM      13
#define Y2_GPIO_NUM      34
#define VSYNC_GPIO_NUM   5
#define HREF_GPIO_NUM    27
#define PCLK_GPIO_NUM    25

#elif defined(CAMERA_MODEL_AI_THINKER)
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

#else
#error "Camera model not selected"
#endif
    
/* WiFi Configuration ------------------------------------------------------ */
const char* ssid = "ESPCAM";       // Replace with your WiFi network name
const char* password = "12345678"; // Replace with your WiFi password

/* Web Server Configuration ------------------------------------------------ */
WebServer server(80);

// Function declarations for web server
void handleStream();
void handleRoot();
void handleCapture();

/* OLED Display Configuration ---------------------------------------------- */
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET -1        // Reset pin (not used with I2C)
#define SCREEN_ADDRESS 0x3C  // I2C address for 0.96" OLED

// I2C pins for ESP32-CAM (avoid camera pins)
#define I2C_SDA 14  // GPIO 14 for SDA (Camera Y4 - but we can share carefully)
#define I2C_SCL 15  // GPIO 15 for SCL (Camera Y3 - but we can share carefully)

// Alternative safer pins (if above don't work):
// #define I2C_SDA 2   // GPIO 2 (safer option)
// #define I2C_SCL 4   // GPIO 4 (safer option, but disables flash LED)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* Servo Motor Configuration ----------------------------------------------- */
// Entry Gate Servo (AI-controlled for car + coin detection)
#define SERVO_ENTRY_PIN 12         // GPIO 12 for entry servo control
#define SERVO_ENTRY_PWM_CHANNEL 1  // LEDC channel for entry servo
#define SERVO_ENTRY_PWM_FREQ 50    // 50Hz PWM frequency for entry servo
#define SERVO_ENTRY_PWM_RESOLUTION 16  // 16-bit resolution for precise control

// Exit Gate Servo (IR-controlled for car exit detection)  
#define SERVO_EXIT_PIN 2           // GPIO 2 for exit servo control
#define SERVO_EXIT_PWM_CHANNEL 2   // LEDC channel for exit servo
#define SERVO_EXIT_PWM_FREQ 50     // 50Hz PWM frequency for exit servo
#define SERVO_EXIT_PWM_RESOLUTION 16   // 16-bit resolution for precise control

// Servo angle to PWM duty cycle mapping (for SG90 servos)
#define SERVO_MIN_PULSE 1000  // 1ms pulse = 0 degrees
#define SERVO_MAX_PULSE 2000  // 2ms pulse = 180 degrees
#define SERVO_BASE_ANGLE 0    // Base position (0 degrees) - Gates closed
#define SERVO_ACTIVE_ANGLE 90 // Active position (90 degrees) - Gates open

// Entry servo control variables
unsigned long entry_servo_start_time = 0;
bool entry_servo_is_active = false;

// Exit servo control variables
unsigned long exit_servo_start_time = 0;
bool exit_servo_is_active = false;

/* Parking Slot Management ------------------------------------------------- */
#define MAX_PARKING_SLOTS 4   // Maximum number of parking slots
int available_slots = 4;      // Current available parking slots (starts at 4)

/* IR Sensor Configuration ------------------------------------------------- */
#define IR_SENSOR_PIN 13     // GPIO 13 for IR sensor input (safe pin for ESP32-CAM)
#define IR_DETECTION_DELAY 1000  // Debounce delay in milliseconds (1 second)

// IR sensor variables
unsigned long ir_last_trigger = 0;
bool ir_sensor_triggered = false;
bool gate_auto_mode = true;    // Enable automatic gate control via IR sensor

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS           320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS           240
#define EI_CAMERA_FRAME_BYTE_SIZE                 3

/* Private variables ------------------------------------------------------- */
static bool debug_nn = false; // Set this to true to see e.g. features generated from the raw signal
static bool is_initialised = false;
static bool streaming_mode = false; // Track if we're in streaming mode
uint8_t *snapshot_buf; //points to the output of the capture

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

    //XCLK 20MHz or 10MHz for OV2640 double FPS (Experimental)
    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_RGB565, // Back to RGB565 for sensor compatibility
    .frame_size = FRAMESIZE_QVGA,    //QQVGA-UXUGA Do not use sizes above QVGA when not JPEG

    .jpeg_quality = 12, //0-63 lower number means higher quality (not used with RGB565)
    .fb_count = 1,       //Using only 1 frame buffer to save memory
    .fb_location = CAMERA_FB_IN_PSRAM,
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

/* Function definitions ------------------------------------------------------- */
bool ei_camera_init(void);
void ei_camera_deinit(void);
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) ;
void oled_init(void);
void oled_display_message(String title, String message, float confidence = -1);
void oled_clear(void);
void servo_entry_init(void);
void servo_exit_init(void);
void servo_move_to_angle(int pin, int angle);
void servo_entry_handle_timer(void);
void servo_exit_handle_timer(void);
void servo_entry_activate_sequence(void);
void servo_exit_activate_sequence(void);
void ir_sensor_init(void);
bool ir_sensor_read(void);
void gate_control_system(void);
void update_parking_slots(int change);
void display_parking_status(void);
void wifi_init(void);
void webserver_init(void);

/**
* @brief      Arduino setup function
*/
void setup()
{
    // put your setup code here, to run once:
    Serial.begin(115200);
    //comment out the below line to start inference immediately after upload
    // while (!Serial); // REMOVED: This blocks ESP32-CAM when no serial monitor is connected
    delay(2000); // Give serial time to initialize
    Serial.println("Edge Impulse Inferencing Demo");
    Serial.println("ESP32-CAM Boot Diagnostic:");
    Serial.println("Free heap: " + String(ESP.getFreeHeap()));
    Serial.println("Chip revision: " + String(ESP.getChipRevision()));
    Serial.println("CPU frequency: " + String(ESP.getCpuFreqMHz()) + " MHz");
    
    // Initialize OLED display
    oled_init();
    oled_display_message("AI PARKING", "Initializing...", -1);
    
    // Initialize servo motors
    servo_entry_init();
    servo_exit_init();
    
    // Initialize IR sensor for automatic gate control
    ir_sensor_init();
    
    // Initialize WiFi and Web Server
    wifi_init();
    webserver_init();
    
    // Display initial parking status
    Serial.println("AI Parking System Initialized:");
    Serial.println("Available slots: " + String(available_slots) + "/" + String(MAX_PARKING_SLOTS));
    display_parking_status();
    
    if (ei_camera_init() == false) {
        ei_printf("Failed to initialize Camera!\r\n");
        oled_display_message("ERROR", "Camera Init Failed", -1);
        delay(2000);
    }
    else {
        ei_printf("Camera initialized\r\n");
        oled_display_message("SUCCESS", "Camera Ready", -1);
        delay(1000);
    }

    ei_printf("\nStarting continious inference in 2 seconds...\n");
    oled_display_message("AI PARKING", "Starting AI...", -1);
    ei_sleep(2000);
}

/**
* @brief      Get data and run inferencing
*
* @param[in]  debug  Get debug info if true
*/
void loop()
{
    // Handle web server requests
    server.handleClient();
    
    // Handle servo timers (check if servos need to return to base position)
    servo_entry_handle_timer();
    servo_exit_handle_timer();
    
    // Handle IR sensor for automatic gate control
    gate_control_system();

    // Monitor memory usage periodically (reduced frequency)
    static unsigned long last_memory_check = 0;
    if (millis() - last_memory_check > 30000) { // Every 30 seconds instead of 10
        last_memory_check = millis();
    }

    // instead of wait_ms, we'll wait on the signal, this allows threads to cancel us...
    if (ei_sleep(5) != EI_IMPULSE_OK) {
        return;
    }

    snapshot_buf = (uint8_t*)malloc(EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE);

    // check if allocation was successful
    if(snapshot_buf == nullptr) {
        ei_printf("ERR: Failed to allocate snapshot buffer!\n");
        return;
    }

    ei::signal_t signal;
    signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
    signal.get_data = &ei_camera_get_data;

    if (ei_camera_capture((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshot_buf) == false) {
        ei_printf("Failed to capture image\r\n");
        free(snapshot_buf);
        return;
    }

    // Run the classifier
    ei_impulse_result_t result = { 0 };

    EI_IMPULSE_ERROR err = run_classifier(&signal, &result, debug_nn);
    if (err != EI_IMPULSE_OK) {
        ei_printf("ERR: Failed to run classifier (%d)\n", err);
        return;
    }

    // print the predictions
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
                result.timing.dsp, result.timing.classification, result.timing.anomaly);

#if EI_CLASSIFIER_OBJECT_DETECTION == 1
    ei_printf("Object detection bounding boxes:\r\n");
    bool car_detected = false;
    bool coin_detected = false;
    float max_car_confidence = 0.0;
    float max_coin_confidence = 0.0;
    
    for (uint32_t i = 0; i < result.bounding_boxes_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.bounding_boxes[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
                
        // Check for car or coin detection with confidence threshold
        if (bb.value > 0.6) { // 60% confidence threshold
            if (strcmp(bb.label, "car") == 0) {
                car_detected = true;
                if (bb.value > max_car_confidence) max_car_confidence = bb.value;
            } else if (strcmp(bb.label, "coin") == 0) {
                coin_detected = true;
                if (bb.value > max_coin_confidence) max_coin_confidence = bb.value;
            }
        }
    }
    
    // Display results on OLED and handle entry gate logic
    if (car_detected && coin_detected && available_slots > 0) {
        // Both car and coin detected with available slots - ENTRY ALLOWED
        String slots_msg = "Entry: " + String(available_slots-1) + "/" + String(MAX_PARKING_SLOTS);
        oled_display_message("CAR ENTRY", slots_msg, max_car_confidence > max_coin_confidence ? max_car_confidence : max_coin_confidence);
        
        // Only activate entry gate if not already active and exit gate is not active
        if (!entry_servo_is_active && !exit_servo_is_active) {
            servo_entry_activate_sequence();
            update_parking_slots(-1); // Decrease available slots
        }
        
    } else if (car_detected && coin_detected && available_slots == 0) {
        // Car and coin detected but parking full - NO ENTRY
        oled_display_message("PARKING", "FULL - NO ENTRY", -1);
        
    } else if (car_detected) {
        // Only car detected - NO ENTRY (need coin for payment)
        String slots_msg = "Need Payment: " + String(available_slots) + "/" + String(MAX_PARKING_SLOTS);
        oled_display_message("CAR FOUND", slots_msg, max_car_confidence);
        
    } else if (coin_detected) {
        // Only coin detected - NO ENTRY (need car)
        String slots_msg = "Need Car: " + String(available_slots) + "/" + String(MAX_PARKING_SLOTS);
        oled_display_message("COIN FOUND", slots_msg, max_coin_confidence);
        
    } else {
        // No objects detected - IDLE STATE
        String slots_msg = "Available: " + String(available_slots) + "/" + String(MAX_PARKING_SLOTS);
        oled_display_message("AI PARKING", slots_msg, -1);
    }

    // Print the prediction results (classification)
#else
    ei_printf("Predictions:\r\n");
    float max_confidence = 0.0;
    String detected_class = "";
    
    for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
        ei_printf("  %s: ", ei_classifier_inferencing_categories[i]);
        ei_printf("%.5f\r\n", result.classification[i].value);
        
        // Find the class with highest confidence
        if (result.classification[i].value > max_confidence) {
            max_confidence = result.classification[i].value;
            detected_class = String(ei_classifier_inferencing_categories[i]);
        }
    }
    
    // Display classification results on OLED
    if (max_confidence > 0.6) {
        String display_text = detected_class;
        display_text += " Found!";
        oled_display_message("CLASSIFIED", display_text, max_confidence);
        ei_printf("High confidence classification: %s\r\n", detected_class.c_str());
    } else {
        oled_display_message("SCANNING", "Low confidence", max_confidence);
        ei_printf("Low confidence detection\r\n");
    }
#endif

    // Print anomaly result (if it exists)
#if EI_CLASSIFIER_HAS_ANOMALY
    ei_printf("Anomaly prediction: %.3f\r\n", result.anomaly);
#endif

#if EI_CLASSIFIER_HAS_VISUAL_ANOMALY
    ei_printf("Visual anomalies:\r\n");
    for (uint32_t i = 0; i < result.visual_ad_count; i++) {
        ei_impulse_result_bounding_box_t bb = result.visual_ad_grid_cells[i];
        if (bb.value == 0) {
            continue;
        }
        ei_printf("  %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\r\n",
                bb.label,
                bb.value,
                bb.x,
                bb.y,
                bb.width,
                bb.height);
    }
#endif


    free(snapshot_buf);

}

/**
 * @brief   Setup image sensor & start streaming
 *
 * @retval  false if initialisation failed
 */
bool ei_camera_init(void) {

    if (is_initialised) return true;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

    //initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x\n", err);
      return false;
    }

    sensor_t * s = esp_camera_sensor_get();
    // initial sensors are flipped vertically and colors are a bit saturated
    if (s->id.PID == OV3660_PID) {
      s->set_vflip(s, 1); // flip it back
      s->set_brightness(s, 1); // up the brightness just a bit
      s->set_saturation(s, 0); // lower the saturation
    }

    // Optimize camera settings for AI detection with RGB format
    s->set_framesize(s, FRAMESIZE_QVGA); // Ensure QVGA (320x240) for consistent processing
    s->set_quality(s, 10); // Good quality for better feature detection
    s->set_colorbar(s, 0); // Disable color bar
    s->set_whitebal(s, 1); // Enable white balance
    s->set_gain_ctrl(s, 1); // Enable gain control
    s->set_exposure_ctrl(s, 1); // Enable exposure control
    s->set_hmirror(s, 0); // Disable horizontal mirror
    s->set_vflip(s, 0); // Disable vertical flip (adjust as needed)

#if defined(CAMERA_MODEL_M5STACK_WIDE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
#elif defined(CAMERA_MODEL_ESP_EYE)
    s->set_vflip(s, 1);
    s->set_hmirror(s, 1);
    s->set_awb_gain(s, 1);
#endif

    is_initialised = true;
    return true;
}

/**
 * @brief      Stop streaming of sensor data
 */
void ei_camera_deinit(void) {

    //deinitialize the camera
    esp_err_t err = esp_camera_deinit();

    if (err != ESP_OK)
    {
        ei_printf("Camera deinit failed\n");
        return;
    }

    is_initialised = false;
    return;
}


/**
 * @brief      Capture, rescale and crop image
 *
 * @param[in]  img_width     width of output image
 * @param[in]  img_height    height of output image
 * @param[in]  out_buf       pointer to store output image, NULL may be used
 *                           if ei_camera_frame_buffer is to be used for capture and resize/cropping.
 *
 * @retval     false if not initialised, image captured, rescaled or cropped failed
 *
 */
bool ei_camera_capture(uint32_t img_width, uint32_t img_height, uint8_t *out_buf) {
    bool do_resize = false;

    if (!is_initialised) {
        ei_printf("ERR: Camera is not initialized\r\n");
        return false;
    }

    camera_fb_t *fb = esp_camera_fb_get();

    if (!fb) {
        ei_printf("Camera capture failed\n");
        return false;
    }

    // Convert RGB565 to RGB888 for Edge Impulse processing
    bool converted = fmt2rgb888(fb->buf, fb->len, PIXFORMAT_RGB565, snapshot_buf);

    esp_camera_fb_return(fb);

    if(!converted){
        ei_printf("Conversion failed\n");
        return false;
    }

    if ((img_width != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
        || (img_height != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
        do_resize = true;
    }

    if (do_resize) {
        ei::image::processing::crop_and_interpolate_rgb888(
        out_buf,
        EI_CAMERA_RAW_FRAME_BUFFER_COLS,
        EI_CAMERA_RAW_FRAME_BUFFER_ROWS,
        out_buf,
        img_width,
        img_height);
    }

    return true;
}

static int ei_camera_get_data(size_t offset, size_t length, float *out_ptr)
{
    // we already have a RGB888 buffer, so recalculate offset into pixel index
    size_t pixel_ix = offset * 3;
    size_t pixels_left = length;
    size_t out_ptr_ix = 0;

    while (pixels_left != 0) {
        // Swap BGR to RGB here
        // due to https://github.com/espressif/esp32-camera/issues/379
        out_ptr[out_ptr_ix] = (snapshot_buf[pixel_ix + 2] << 16) + (snapshot_buf[pixel_ix + 1] << 8) + snapshot_buf[pixel_ix];

        // go to the next pixel
        out_ptr_ix++;
        pixel_ix+=3;
        pixels_left--;
    }
    // and done!
    return 0;
}

/**
 * @brief      Initialize OLED display
 */
void oled_init(void) {
    // Initialize I2C with custom SDA/SCL pins
    Wire.begin(I2C_SDA, I2C_SCL);
    
    // Initialize OLED display
    if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
        Serial.println(F("SSD1306 allocation failed"));
        return;
    }
    
    // Clear the buffer and show initial message
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0,0);
    display.println(F("ESP32-CAM AI"));
    display.println(F("OLED Ready!"));
    display.display();
    delay(1000);
    
    Serial.println("OLED Display initialized");
}

/**
 * @brief      Display message on OLED screen
 *
 * @param[in]  title       Title text (line 1)
 * @param[in]  message     Message text (line 2-3)
 * @param[in]  confidence  Confidence value (-1 to skip)
 */
void oled_display_message(String title, String message, float confidence) {
    display.clearDisplay();
    
    // Title (large text)
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(title);
    
    // Message (normal text)
    display.setTextSize(1);
    display.setCursor(0, 20);
    display.println(message);
    
    // Confidence (if provided)
    if (confidence >= 0) {
        display.setCursor(0, 35);
        display.print("Confidence: ");
        display.print(confidence * 100, 1);
        display.println("%");
    }
    
    // Processing time info
    display.setCursor(0, 50);
    display.print("Time: ");
    display.print(millis() / 1000);
    display.println("s");
    
    display.display();
}

/**
 * @brief      Clear OLED display
 */
void oled_clear(void) {
    display.clearDisplay();
    display.display();
}

/**
 * @brief      Initialize entry servo motor using simple GPIO control
 */
void servo_entry_init(void) {
    // Configure entry servo pin as output
    pinMode(SERVO_ENTRY_PIN, OUTPUT);
    digitalWrite(SERVO_ENTRY_PIN, LOW);  // Ensure pin starts LOW
    
    // Set entry servo to base position (0 degrees)
    servo_move_to_angle(SERVO_ENTRY_PIN, SERVO_BASE_ANGLE);
}

/**
 * @brief      Initialize exit servo motor using simple GPIO control
 */
void servo_exit_init(void) {
    // Configure exit servo pin as output
    pinMode(SERVO_EXIT_PIN, OUTPUT);
    digitalWrite(SERVO_EXIT_PIN, LOW);  // Ensure pin starts LOW
    
    // Set exit servo to base position (0 degrees)
    servo_move_to_angle(SERVO_EXIT_PIN, SERVO_BASE_ANGLE);
}

/**
 * @brief      Move servo to specific angle (0-180 degrees)
 *
 * @param[in]  pin    GPIO pin for the servo
 * @param[in]  angle  Target angle in degrees (0-180)
 */
void servo_move_to_angle(int pin, int angle) {
    // Constrain angle between 0-180
    angle = constrain(angle, 0, 180);
    
    // Generate PWM signal using delayMicroseconds for servo control
    // Increased pulses for reliable movement (50 pulses = 1 second)
    for (int i = 0; i < 50; i++) {  // 50 pulses for better movement
        digitalWrite(pin, HIGH);
        
        // Calculate pulse width: 1000-2000 microseconds (1-2ms)
        int pulse_width = map(angle, 0, 180, 1000, 2000);
        delayMicroseconds(pulse_width);
        
        digitalWrite(pin, LOW);
        // Complete 20ms cycle (50Hz)
        delayMicroseconds(20000 - pulse_width);
    }
    
    // Small delay to ensure servo reaches position
    delay(100);
}

/**
 * @brief      Handle entry servo timer - return to base position after 5 seconds
 */
void servo_entry_handle_timer(void) {
    if (entry_servo_is_active && (millis() - entry_servo_start_time >= 5000)) {
        // 5 seconds have passed, return entry servo to base position
        servo_move_to_angle(SERVO_ENTRY_PIN, SERVO_BASE_ANGLE);
        entry_servo_is_active = false;
        Serial.println("ENTRY GATE: Closed (5 seconds elapsed)");
    }
}

/**
 * @brief      Handle exit servo timer - return to base position after 5 seconds
 */
void servo_exit_handle_timer(void) {
    if (exit_servo_is_active && (millis() - exit_servo_start_time >= 5000)) {
        // 5 seconds have passed, return exit servo to base position
        servo_move_to_angle(SERVO_EXIT_PIN, SERVO_BASE_ANGLE);
        exit_servo_is_active = false;
        Serial.println("EXIT GATE: Closed (5 seconds elapsed)");
    }
}

/**
 * @brief      Activate entry servo sequence - move to 90 degrees and start timer
 */
void servo_entry_activate_sequence(void) {
    if (!entry_servo_is_active) {
        // Move entry servo to active position (90 degrees)
        servo_move_to_angle(SERVO_ENTRY_PIN, SERVO_ACTIVE_ANGLE);
        
        // Start timer
        entry_servo_start_time = millis();
        entry_servo_is_active = true;
        Serial.println("ENTRY GATE: Opened (AI detected car + coin)");
    }
}

/**
 * @brief      Activate exit servo sequence - move to 90 degrees and start timer
 */
void servo_exit_activate_sequence(void) {
    if (!exit_servo_is_active) {
        // Move exit servo to active position (90 degrees)
        servo_move_to_angle(SERVO_EXIT_PIN, SERVO_ACTIVE_ANGLE);
        
        // Start timer
        exit_servo_start_time = millis();
        exit_servo_is_active = true;
        Serial.println("EXIT GATE: Opened (IR sensor detected object)");
    }
}

/**
 * @brief      Initialize IR sensor
 */
void ir_sensor_init(void) {
    // Configure IR sensor pin as input with internal pull-up
    pinMode(IR_SENSOR_PIN, INPUT_PULLUP);
}

/**
 * @brief      Read IR sensor with debouncing
 * 
 * @return     true if car detected, false otherwise
 */
bool ir_sensor_read(void) {
    // Read IR sensor (LOW = object detected for most IR sensors)
    bool current_state = !digitalRead(IR_SENSOR_PIN); // Invert because we use pull-up
    
    // Debouncing - only trigger if enough time has passed
    if (current_state && (millis() - ir_last_trigger > IR_DETECTION_DELAY)) {
        ir_last_trigger = millis();
        return true;
    }
    
    return false;
}

/**
 * @brief      Handle automatic exit gate control based on IR sensor
 */
void gate_control_system(void) {
    if (!gate_auto_mode) return; // Skip if auto mode disabled
    
    // Check IR sensor for object detection (car exiting)
    if (ir_sensor_read()) {
        // Only allow exit if parking is not empty (available_slots < MAX_PARKING_SLOTS)
        if (available_slots < MAX_PARKING_SLOTS) {
            Serial.println("IR SENSOR: Object detected - Car exiting");
            
            // Only activate exit gate if not already active
            if (!exit_servo_is_active) {
                servo_exit_activate_sequence();
                update_parking_slots(+1); // Increase available slots (car leaving)
                
                // Update OLED to show exit status
                String exit_msg = "Exit: " + String(available_slots) + "/" + String(MAX_PARKING_SLOTS);
                oled_display_message("CAR EXIT", exit_msg, -1);
            }
            
        } else {
            // Parking is already empty - ignore IR sensor
            Serial.println("IR SENSOR: Ignored - Parking already empty");
        }
    }
}

/**
 * @brief      Update parking slot count and ensure it stays within bounds
 * 
 * @param[in]  change  +1 for car exiting, -1 for car entering
 */
void update_parking_slots(int change) {
    available_slots += change;
    
    // Ensure slots stay within bounds (0 to MAX_PARKING_SLOTS)
    if (available_slots < 0) {
        available_slots = 0;
    } else if (available_slots > MAX_PARKING_SLOTS) {
        available_slots = MAX_PARKING_SLOTS;
    }
    
    Serial.println("PARKING STATUS: " + String(available_slots) + "/" + String(MAX_PARKING_SLOTS) + " slots available");
}

/**
 * @brief      Display current parking status on OLED and serial
 */
void display_parking_status(void) {
    String status_msg;
    
    if (available_slots == 0) {
        status_msg = "FULL 0/" + String(MAX_PARKING_SLOTS);
    } else if (available_slots == MAX_PARKING_SLOTS) {
        status_msg = "Empty " + String(available_slots) + "/" + String(MAX_PARKING_SLOTS);
    } else {
        status_msg = "Available " + String(available_slots) + "/" + String(MAX_PARKING_SLOTS);
    }
    
    // Show parking status after a brief delay
    delay(1500);
    oled_display_message("AI PARKING", status_msg, -1);
    Serial.println("Current parking status: " + status_msg);
}

/**
 * @brief      Initialize WiFi connection
 */
void wifi_init(void) {
    Serial.println("Connecting to WiFi...");
    oled_display_message("WiFi", "Connecting...", -1);
    
    WiFi.begin(ssid, password);
    WiFi.setSleep(false);
    
    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 20) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    
    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("");
        Serial.println("WiFi connected!");
        Serial.print("Camera Stream Ready! Go to: http://");
        Serial.println(WiFi.localIP());
        
        oled_display_message("WiFi OK", WiFi.localIP().toString(), -1);
        delay(2000);
    } else {
        Serial.println("WiFi connection failed!");
        oled_display_message("WiFi", "Failed!", -1);
        delay(2000);
    }
}

/**
 * @brief      Initialize web server with streaming only
 */
void webserver_init(void) {
    server.on("/", handleRoot);  // Root shows the streaming page
    server.on("/stream", handleStream);  // Stream endpoint
    server.on("/capture", handleCapture);  // Single capture
    
    server.begin();
    Serial.println("Web server started - streaming ready");
}

/**
 * @brief      Handle root page - Simple streaming page
 */
void handleRoot() {
    String html = "<!DOCTYPE html><html><head>";
    html += "<meta http-equiv='refresh' content='1'>";
    html += "<style>body{font-family:Arial;text-align:center;margin:20px;}";
    html += "img{max-width:100%;height:auto;border:1px solid #ccc;}";
    html += "</style></head><body>";
    html += "<h2>ESP32-CAM Live Stream</h2>";
    html += "<img src='/capture' alt='Camera Stream'>";
    html += "<p>Auto-refreshing every second</p>";
    html += "<p><a href='/capture'>Single Capture</a></p>";
    html += "</body></html>";
    
    server.send(200, "text/html", html);
}

/**
 * @brief      Handle single image capture - RGB565 to JPEG for web display
 */
void handleCapture() {
    // Capture RGB565 frame (sensor's native format)
    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        server.send(500, "text/plain", "Camera failed");
        return;
    }
    
    // Convert RGB565 to JPEG for web display
    uint8_t* jpeg_buf = NULL;
    size_t jpeg_len = 0;
    bool success = fmt2jpg(fb->buf, fb->len, fb->width, fb->height, PIXFORMAT_RGB565, 80, &jpeg_buf, &jpeg_len);
    
    // Return frame buffer immediately after conversion
    esp_camera_fb_return(fb);
    
    if (success && jpeg_buf && jpeg_len > 0) {
        // Send JPEG to browser
        server.send_P(200, "image/jpeg", (const char*)jpeg_buf, jpeg_len);
        free(jpeg_buf);
    } else {
        server.send(500, "text/plain", "JPEG conversion failed");
    }
}

/**
 * @brief      Handle video stream request - Simple RGB565 stream
 */
void handleStream() {
    // Redirect to capture for now
    server.sendHeader("Location", "/capture");
    server.send(302, "text/plain", "Redirecting to capture");
}

#if !defined(EI_CLASSIFIER_SENSOR) || EI_CLASSIFIER_SENSOR != EI_CLASSIFIER_SENSOR_CAMERA
#error "Invalid model for current sensor"
#endif
