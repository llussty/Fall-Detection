#include "esp_camera.h"
#include <WiFi.h>
#include <Arduino.h>

#include <UrlEncode.h>
#include <HTTPClient.h>

//Fall detection model
#include <Fall_Detection_v3_inferencing.h>
#include "edge-impulse-sdk/dsp/image/image.hpp"

// Pin Definitions for Camera Module
#define CAMERA_MODEL_ESP32S3_EYE
#define PWDN_GPIO_NUM -1
#define RESET_GPIO_NUM -1
#define XCLK_GPIO_NUM 15
#define SIOD_GPIO_NUM 4
#define SIOC_GPIO_NUM 5
#define Y2_GPIO_NUM 11
#define Y3_GPIO_NUM 9
#define Y4_GPIO_NUM 8
#define Y5_GPIO_NUM 10
#define Y6_GPIO_NUM 12
#define Y7_GPIO_NUM 18
#define Y8_GPIO_NUM 17
#define Y9_GPIO_NUM 16
#define VSYNC_GPIO_NUM 6
#define HREF_GPIO_NUM 7
#define PCLK_GPIO_NUM 13

/* Constant defines -------------------------------------------------------- */
#define EI_CAMERA_RAW_FRAME_BUFFER_COLS 320
#define EI_CAMERA_RAW_FRAME_BUFFER_ROWS 240
#define EI_CAMERA_FRAME_BYTE_SIZE 3
#define MAX_SSID_LENGTH 20
#define MAX_PASSWORD_LENGTH 20

// Credentials for API
String phoneNumber; // Replace with your mobile number
String APIKey;            // Replace with your API key

// WiFi Credentials
char ssid[MAX_SSID_LENGTH];                // WiFi SSID
char password[MAX_PASSWORD_LENGTH];        // WiFi Password

// Fall Detection
float fallThreshold = 0.7;  // Minimum value needed for a fall prediction to count as fall
int fallIteration = 5;      // Range of average used for fall detection
float totalFallValue = 0;   // 


/* Private variables ------------------------------------------------------- */
static bool DEBUGNN = false;   // Debug flag for Neural Network
static bool isInitialised = false;
uint8_t *snapshotBuffer;      // Buffer to hold the captured image


// Camera Configuration
static camera_config_t cameraConfig = {
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
  .xclk_freq_hz = 20000000,      // XCLK Frequency
  .ledc_timer = LEDC_TIMER_0,
  .ledc_channel = LEDC_CHANNEL_0,
  .pixel_format = PIXFORMAT_JPEG, // Pixel Format
  .frame_size = FRAMESIZE_QVGA,   // Frame Size
  .jpeg_quality = 12,             // JPEG Quality
  .fb_count = 2,                  // Frame Buffer Count
  .fb_location = CAMERA_FB_IN_PSRAM,
  .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

// Function Declarations
void captureEICamera(uint32_t imageWidth, uint32_t imageHeight, uint8_t *bufferOut);
static int getCameraData(size_t offset, size_t length, float *out_ptr);
void detectFall();
void checkForFall();
void sendWhatsAppMessage(String message);
void startupConfiguration();
void startCameraServer();


void setup() {
  Serial.begin(115200);
  while(!Serial);
  Serial.setDebugOutput(true);

  startupConfiguration();

  Serial.println("Connecting to Wi-Fi Access Point");
  WiFi.begin(ssid, password);
  Serial.println();

  // Wait for WiFi connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  // Initialize Camera
  Serial.println("Starting Camera");
  esp_err_t cameraStart = esp_camera_init(&cameraConfig);

  // Start Camera Server
  startCameraServer(); 

  // Print IP address to connect
  Serial.print("Camera Ready! Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect");

  // Delay before starting inference
  ei_printf("\nStarting continuous inference in 2 seconds...\n");
  ei_sleep(2000);
}

void loop() {
  
  ei_sleep(2000); // Sleep (ms)
  detectFall(); // Perform fall detection
};

// Send WhatsApp message
void sendWhatsAppMessage(String message) {
  // Construct API URL with mobile number, encoded message, and API key
  String APIURL = "https://api.callmebot.com/whatsapp.php?phone=" 
  + phoneNumber + "&text=" + urlEncode(message) + "&apikey=" + APIKey;
  
  // Initialize HTTP client
  HTTPClient http;
  http.begin(APIURL);  // Specify the URL for the HTTP request

  // Print the URL to which the request is being sent
  Serial.println("Sending request to: " + APIURL);

  // Add header specifying the content type
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  
  // Send HTTP GET request and get the response code
  int httpResponseCode = http.GET();  // Change to POST method if necessary

  // Print the HTTP response code
  Serial.print("HTTP response code: ");
  Serial.println(httpResponseCode);

  // Check if the message was sent successfully
  if (httpResponseCode == 200) {
    Serial.println("WhatsApp message sent successfully");
  } else {
    // Print error message and server response if there was an error
    Serial.println("Error sending the message");
    String response = http.getString();
    Serial.println("Server response: " + response);
  }

  // End HTTP connection
  http.end();
}

// Callback function to get camera data
static int getCameraData(size_t offset, size_t length, float *out_ptr) {
  size_t pixel_ix = offset * 3;     // Calculate starting index of the pixel data
  size_t pixels_left = length;      // Number of pixels left to process
  size_t out_ptr_ix = 0;            // Index for the output pointer

  // Loop through the pixels and convert them to float values
  while (pixels_left != 0) {
    // Convert pixel data (RGB) to a single float value and store it in the output buffer
    out_ptr[out_ptr_ix] = (snapshotBuffer[pixel_ix] << 16) + (snapshotBuffer[pixel_ix + 1] << 8) + snapshotBuffer[pixel_ix + 2];

    // Move to the next pixel
    out_ptr_ix++;
    pixel_ix += 3;
    pixels_left--;
  }

  return 0;  // Return 0 to indicate success
}

// Capture camera image
void captureEICamera(uint32_t imageWidth, uint32_t imageHeight, uint8_t *bufferOut) {
  bool do_resize = false; // Flag to indicate whether resizing is needed
  
  // Get camera frame buffer
  camera_fb_t *frameBuffer = esp_camera_fb_get();
  
  // Convert camera frame buffer to RGB888 format
  bool converted = fmt2rgb888(frameBuffer->buf, frameBuffer->len, PIXFORMAT_JPEG, snapshotBuffer);
  
  // Return camera frame buffer to the pool
  esp_camera_fb_return(frameBuffer);
  
  // Check if resizing is needed based on image width and height
  if ((imageWidth != EI_CAMERA_RAW_FRAME_BUFFER_COLS)
      || (imageHeight != EI_CAMERA_RAW_FRAME_BUFFER_ROWS)) {
    do_resize = true; // Set flag if resizing is needed
  }
  
  // Perform resizing if needed
  if (do_resize) {
    // Resize the captured image to match the specified width and height
    ei::image::processing::crop_and_interpolate_rgb888(
      bufferOut,                                        // Output buffer
      EI_CAMERA_RAW_FRAME_BUFFER_COLS,                  // Original image width
      EI_CAMERA_RAW_FRAME_BUFFER_ROWS,                  // Original image height
      bufferOut,                                        // Resized image buffer
      imageWidth,                                      // Target width
      imageHeight                                      // Target height
    );
  }
}

// Perform fall detection
void checkForFall() {
  snapshotBuffer = (uint8_t *)malloc(// Allocates memory for snapshotBuffer from heap
    EI_CAMERA_RAW_FRAME_BUFFER_COLS * EI_CAMERA_RAW_FRAME_BUFFER_ROWS * EI_CAMERA_FRAME_BYTE_SIZE); 

  // Define signal for capturing camera data
  ei::signal_t signal;
  signal.total_length = EI_CLASSIFIER_INPUT_WIDTH * EI_CLASSIFIER_INPUT_HEIGHT;
  signal.get_data = &getCameraData;

  // Capture image from the camera
  captureEICamera((size_t)EI_CLASSIFIER_INPUT_WIDTH, (size_t)EI_CLASSIFIER_INPUT_HEIGHT, snapshotBuffer);

  // Run the fall detection classifier
  ei_impulse_result_t result = { 0 };
  EI_IMPULSE_ERROR startClassifier = run_classifier(&signal, &result);

  // Print timing information alongside prediction details
  ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
            result.timing.dsp, result.timing.classification, result.timing.anomaly);

  // Check if fall is detected
  bool fallFound = result.bounding_boxes[0].value > 0.8;
  for (size_t ix = 0; ix < result.bounding_boxes_count; ix++) {
    auto fall = result.bounding_boxes[ix];
    if (fall.value == 0) {
      continue;
    }
    // Print information about detected fall
    ei_printf("    %s (%f) [ x: %u, y: %u, width: %u, height: %u ]\n", 
    fall.label, fall.value, fall.x, fall.y, fall.width, fall.height);
    
    // Update totalFallValue with the fall value
    totalFallValue = fall.value;
  }
  // Print message if no fall is detected
  if (!fallFound) {
    ei_printf("    No Fall found\n");
  }
  free(snapshotBuffer); // Free snapshotBuffer from heap
}

// Perform fall detection and send message if fall detected
void detectFall() {
  // Check for fall detection
  checkForFall();
  
  // Print the total fall value
  Serial.println(totalFallValue);
  
  // Check if total fall value is greater than fall threshold
  if (totalFallValue > fallThreshold) {
    float averageFall = 0.0;
    
    // Perform fall detection multiple times and calculate average fall value of fallIteration
    for (int i = 0; i < fallIteration; i++) {
      Serial.println("Loop running");
      checkForFall();

      //Adds and equates the averagefall with totalFallValue
      averageFall += totalFallValue;
    }
    averageFall /= fallIteration; // Calculate average fall value by dividing against fallIteration then equating
    
    // If average fall value is greater than defined Fallthreshold, send a message
    if (averageFall > fallThreshold) {
      Serial.println("Sending Message");
      // Prepare message with URL
      String message = "A fall has been detected. Please check the following URL for details: http://"; 

      // Concatenates string URL with IP camera server
      String url = WiFi.localIP().toString();  
      message += url;
      
      sendWhatsAppMessage(message);
    }
    
    ei_printf("Average fall probability: %f\n", averageFall);
  }
  
  // Reset total fall value for next iteration of loop
  totalFallValue = 0;
}

// Configures Variables
void startupConfiguration() {
  Serial.println("Configuration: ");
  Serial.println();

  Serial.println("Enter Phone number you wish to use:");
  while (!Serial.available()); // Wait for user input
  phoneNumber = Serial.readString(); // Read user input

  Serial.println("Enter API Key:");
  while (!Serial.available()); // Wait for user input
  APIKey = Serial.readString(); // Read user input

  Serial.println("Enter WiFi SSID:");
  while (!Serial.available());
  Serial.readBytesUntil('\n', ssid, MAX_SSID_LENGTH); // Read user input and store in ssid array

  Serial.println("Enter WiFi Password:");
  while (!Serial.available());
  Serial.readBytesUntil('\n', password, MAX_PASSWORD_LENGTH); // Read user input and store in password array

  Serial.println("Do you wish to change the Fall detection configuration? Y/N");
  while (!Serial.available());
  char userResponse = Serial.read(); 

  if (userResponse == 'Y' || userResponse == 'y') {

    Serial.println("Enter Fall Detection Threshold (default = 0.70): ");
    while (!Serial.available());
    fallThreshold = Serial.parseFloat(); // Read user input as float


    Serial.println("Enter Fall Iteration Threshold (default = 5): ");
    while (!Serial.available());
    fallIteration = Serial.parseInt(); // Read user input as int

  }
  Serial.println("Phone Number: ");
  Serial.println(phoneNumber);

  Serial.println("API Key: ");
  Serial.println(APIKey);

  Serial.println("WiFi SSID: ");
  Serial.println(ssid);

  Serial.println("WiFi Password: ");
  Serial.println(password);

  Serial.println("Fall Threshold: ");
  Serial.println(fallThreshold);

  Serial.println("Fall Iteration: ");
  Serial.println(fallIteration);

}