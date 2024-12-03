#include "Arduino_LED_Matrix.h" // Library for onboard LEDs UNO R4 Wifi
#include <FastLED.h>  // Include FastLED before Artnet or it breaks
#include <ArtnetWiFi.h>  // Include Artnet for WiFi platforms

ArduinoLEDMatrix matrix;
ArtnetWiFiReceiver artnet;
uint8_t universe = 0;  // DMX Universe (0 - 15)

#define DATA_PIN 6        // Data Line Pin
#define NUM_LEDS 100  // LEDs in Strand
CRGB leds[NUM_LEDS];

//Setup Wifi
#include "Wifi_Secrets.h"
#include <WiFiS3.h>

char ssid[] = WIFI_SSID;        // Network SSID
char pass[] = WIFI_PASS;        // Network password
 
bool artnet_first_connect = false;
int rgb_channels[] = {1, 2, 3, 8, 9, 10};  // DMX channels for RGB Color Inputs -- fix this later to match real DMX and not the offset by 1
int num_channels = sizeof(rgb_channels) / sizeof(rgb_channels[0]);
int control_channel = 0;  // Control channel for bar mode -- same as above, fix to match correct and not off by one

//Lighting Parameters -- make this a include file later.
#define STATIC_FADE_LENGTH 20 //Total length of LEDs in the Fade to/from Color 1 to 2
int color_wheel = 0; //This is used to iterate where in the color rotation we are
#define COLOR_WHEEL_MAX 256
#define COLOR_WHEEL_INCREMENT 8

//Draw a tree on the matrix until ArtNet is received.
byte matrix_frame[8][12] = {
  { 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0 },
  { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0 },
  { 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0 },
  { 0, 0, 0, 1, 1, 1, 1, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0 }
};

void setup() {
  // put your setup code here, to run once:
    Serial.begin(115200);
    matrix.begin();
    matrix.renderBitmap(matrix_frame, 8, 12);
    FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
    leds[0].setRGB(0,0,255);
    FastLED.show();
    
    // Connect to WiFi
    WiFi.begin(ssid, pass);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.print("WiFi connected, IP = ");
    Serial.println(WiFi.localIP());
    leds[0].setRGB(0,255,255);
    FastLED.show();

    // Initialize Artnet
    artnet.begin();
    artnet.subscribeArtDmxUniverse(universe, onDmxData);
    old_school_lights();
    FastLED.show();
}


// Callback function for DMX data reception
void onDmxData(const uint8_t* dmx_data, uint16_t size, const ArtDmxMetadata &metadata, const ArtNetRemoteInfo &remote) {
    light_scene_render(dmx_data);
    if (!artnet_first_connect) {
        artnet_first_connect = true;
        memset(matrix_frame, 0, sizeof(matrix_frame));
        matrix.renderBitmap(matrix_frame, 8, 12);
    }

    bool matrix_changed = false;

    // Update matrix based on DMX data
    for (int i = 0; i < num_channels; i++) {
        int dmx_value = dmx_data[rgb_channels[i]];
        matrix_changed |= matrix_dmx_light(i, dmx_value);
    }

    // Update light scene
    matrix_changed |= matrix_light_scene(11, dmx_data[control_channel]);

    // Render matrix only if changed
    if (matrix_changed) {
        matrix.renderBitmap(matrix_frame, 8, 12);
    }

}

// Update the matrix based on DMX light values
bool matrix_dmx_light(int column, int dmx_value) {
    bool changed = false;
    for (int i = 0; i < 8; i++) {
        if (matrix_frame[i][column] != 0) {
            matrix_frame[i][column] = 0;
            changed = true;
        }
    }
    for (int i = 0; i < 8; i++) {
        if (dmx_value == 0) {
            break;
        }
        if ((i * 256 / 8) < (dmx_value + 1)) {
            if (matrix_frame[i][column] != 1) {
                matrix_frame[i][column] = 1;
                changed = true;
            }
        }
    }
    return changed;
}

// Update the matrix bar mode based on DMX values
bool matrix_light_scene(int column, int dmx_value) {
    bool changed = false;
    for (int i = 0; i < 8; i++) {
        if (matrix_frame[i][column] != 0) {
            matrix_frame[i][column] = 0;
            changed = true;
        }
    }
    for (int i = 0; i < 8; i++) {
        if (i < dmx_value) {
            if (matrix_frame[i][column] != 1) {
                matrix_frame[i][column] = 1;
                changed = true;
            }
        }
    }
    return changed;
}

void light_scene_render(const uint8_t* dmx_data){

  switch (dmx_data[control_channel]){
    case 0: {
      old_school_lights();
    }
    break;
    case 1: {
      two_colors_static(dmx_data);
    }
    break;
    case 2: {
      smooth_fade_static(dmx_data);
    }
    break;
    case 3: {
      smooth_fade_sliding(dmx_data);
    }
    break;
    case 4: {
      rgb_color_wheel_sliding();
    }
    break;
  }
  FastLED.show();
  color_wheel_advance();
}

void color_wheel_advance(){
  color_wheel += COLOR_WHEEL_INCREMENT;
  if (color_wheel >= COLOR_WHEEL_MAX){
    color_wheel = 0;
  }
}

void old_school_lights(){
  int color = 0;
  for(int i = 0; i < NUM_LEDS; i++) {
    switch(color){
      case 0: { // Set it Red
        leds[i].setRGB(255,0,0);
        
      }
      break;
      case 1: { // Set it Green
        leds[i].setRGB(0,255,0);
        
      }
      break;
      case 2: { // Set it Pink
        leds[i].setRGB(255,0,180);
        
      }
      break;
      case 3: { // Set it Blue
        leds[i].setRGB(0,0,255);
        
      }
      break;
      case 4: { //Set it yellow
        leds[i].setRGB(255,255,0);
      }
    }
    color++;
    if(color >= 5){
      color = 0;
    }
  }
}

void two_colors_static(const uint8_t* dmx_data){
  int color = 0;
  for(int i = 0; i < NUM_LEDS; i++) {
    switch(color){
      case 0: { // Set it Red
        leds[i].setRGB(dmx_data[rgb_channels[0]],dmx_data[rgb_channels[1]],dmx_data[rgb_channels[2]]);
      }
      break;
      case 1: { // Set it Green
        leds[i].setRGB(dmx_data[rgb_channels[3]],dmx_data[rgb_channels[4]],dmx_data[rgb_channels[5]]);
      }
      break;
    }
    color++;
    if(color>=2){
      color = 0;
    }
  }
}

void smooth_fade_static(const uint8_t* dmx_data){
  // Smooth the % of each color according to the  length of the sequence, ascend first, then descend
  int bulb_seq = 0;
  bool ascending = true;
  for(int i = 0; i < NUM_LEDS; i++) {
    float fade_factor = float(bulb_seq) / float(STATIC_FADE_LENGTH);
    leds[i].setRGB(
      (dmx_data[rgb_channels[0]] * fade_factor) + (dmx_data[rgb_channels[3]] * (1 - fade_factor)) / 2, //Red
      (dmx_data[rgb_channels[1]] * fade_factor) + (dmx_data[rgb_channels[4]] * (1 - fade_factor)) / 2, //Green
      (dmx_data[rgb_channels[2]] * fade_factor) + (dmx_data[rgb_channels[5]] * (1 - fade_factor)) / 2 //Blue
    );
    if(ascending == true){
      bulb_seq++;
      if(bulb_seq >= STATIC_FADE_LENGTH){
        ascending = false;
      }
    }
    else{
      bulb_seq--;
      if(bulb_seq <= 0){
        ascending = true;
      }
    }
  }
}

void smooth_fade_sliding(const uint8_t* dmx_data) {
  // Calculate phase offset based on the color wheel position
  float phase_offset = float(color_wheel) / float(COLOR_WHEEL_MAX);

  for (int i = 0; i < NUM_LEDS; i++) {
    // Calculate the position in the wave, wrapping every STATIC_FADE_LENGTH
    float position = (phase_offset + float(i) / STATIC_FADE_LENGTH);

    // Use a sine wave to calculate the fade factor (normalized to [0, 1])
    float fade_factor = 0.5 * (1.0 + sin(position * 2.0 * PI)); // Maps sine wave to [0, 1]

    // Interpolate between the two colors
    leds[i].setRGB(
      (uint8_t)((dmx_data[rgb_channels[0]] * fade_factor) + (dmx_data[rgb_channels[3]] * (1.0 - fade_factor))), // Red
      (uint8_t)((dmx_data[rgb_channels[1]] * fade_factor) + (dmx_data[rgb_channels[4]] * (1.0 - fade_factor))), // Green
      (uint8_t)((dmx_data[rgb_channels[2]] * fade_factor) + (dmx_data[rgb_channels[5]] * (1.0 - fade_factor)))  // Blue
    );
  }
}

void rgb_color_wheel_sliding() {
  // Calculate the hue offset for sliding the gradient
  float hue_offset = float(color_wheel) / float(COLOR_WHEEL_MAX) * 255.0;

  for (int i = 0; i < NUM_LEDS; i++) {
    // Calculate the hue for the current LED, repeating every STATIC_FADE_LENGTH
    float position = float(i % STATIC_FADE_LENGTH) / float(STATIC_FADE_LENGTH);
    uint8_t hue = uint8_t(hue_offset + position * 255.0);

    // Set the LED color using the HSV model
    leds[i] = CHSV(hue, 255, 255); // Fully saturated and bright
  }
}

void loop() {
  art_net::OpCode opCode = artnet.parse();
}
