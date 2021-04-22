#include <Notecard.h>
#include <OneWire.h>
#ifdef ARDUINO_ARCH_ESP32
  #include <WiFi.h>
#endif

#define productUID "com.blues.zfields:showcase"
#define serialDebug Serial

static const uint8_t DO_PIN = A0;
static const uint8_t DO_PIN_EN = 13;
static const uint8_t ORP_PIN = A1;
static const uint8_t ORP_PIN_EN = 12;
static const uint8_t PH_PIN = A2;
static const uint8_t PH_PIN_EN = 27;
static const uint8_t TBT_PIN = A3;
static const uint8_t TBT_PIN_EN = 33;
static const uint8_t TDS_PIN = A4;
static const uint8_t TDS_PIN_EN = 15;
static const uint8_t TEMP_PIN = 4;
static const uint8_t TEMP_PIN_EN = 32;

static const size_t DO_DELAY_MS = 1;
static const size_t ORP_DELAY_MS = 2;
static const size_t PH_DELAY_MS = 80;
static const size_t TBT_DELAY_MS = 2;
static const size_t TDS_DELAY_MS = 200;

// This period controls the waking frequency of your host MCU, and will have a
// direct impact on battery conservation. It should be used to strike a balance
// between battery performance and data collection requirements.
static size_t PERIOD_S = 600;

Notecard notecard;
OneWire ds18b20(TEMP_PIN);

int sampleAverageAnalogSignal (uint8_t analog_pin_, uint8_t enable_pin_, size_t sensor_delay_ms_) {
  static const size_t SAMPLE_COUNT = 25;
  static int sample[SAMPLE_COUNT];
  size_t max_sample = std::numeric_limits<size_t>::min(), min_sample = std::numeric_limits<size_t>::max();
  size_t max_sample_index = 0, min_sample_index = 0;
  size_t sample_aggregate = 0;
  
  // Enable the sensor
  ::pinMode(enable_pin_,OUTPUT);
  ::digitalWrite(enable_pin_,HIGH);

  // Await the sensor
  :: delay(sensor_delay_ms_);
  
  // Sample the sensor multiple times over one second
  for (size_t i = 0 ; i < SAMPLE_COUNT ; ++i) {
    static const size_t SAMPLE_DELAY_MS = (1000/SAMPLE_COUNT);
    sample[i] = ::analogRead(analog_pin_);
    ::delay(SAMPLE_DELAY_MS);
  }
  
  // Disable the sensor
  ::digitalWrite(enable_pin_,LOW);
  //TODO: Determine if the pin be set to INPUT to conserve battery

  // Identify the highest and lowest samples
  for (size_t i = 0 ; i < SAMPLE_COUNT ; ++i) {
    max_sample_index = (sample[i] > max_sample) * i + (sample[i] <= max_sample) * max_sample_index;
    max_sample = (sample[i] > max_sample) * sample[i] + (sample[i] <= max_sample) * max_sample;
    
    min_sample_index = (sample[i] < min_sample) * i + (sample[i] >= min_sample) * min_sample_index;
    min_sample = (sample[i] < min_sample) * sample[i] + (sample[i] >= min_sample) * min_sample;
  }
  
  // Average the remaining readings
  for (size_t i = 0 ; i < SAMPLE_COUNT ; ++i) {
    // Remove the highest and lowest samples
    if ((i == max_sample_index) || (i == min_sample_index)) { continue; }
    sample_aggregate += sample[i];
  }
  
  return (sample_aggregate / (SAMPLE_COUNT - 2));
}

float sampleDs18b20 (uint8_t one_wire_pin_, uint8_t enable_pin_) { 
  static const size_t MAX_ATTEMPTS = 3;
  uint16_t encoded_temperature = 15984;
  
  // Enable the sensor
  ::pinMode(enable_pin_,OUTPUT);
  ::digitalWrite(enable_pin_,HIGH);

  // Sample the sensor (with retries)
  for (size_t i = 0 ; i < MAX_ATTEMPTS ; ++i) {
    const static size_t SAMPLING_DELAY_MS = 702;
    const static size_t SCRATCHPAD_SIZE = 9;
    byte scratchpad[SCRATCHPAD_SIZE];
  
    // Begin a temperature conversion (Convert T [44h])
    ds18b20.reset();
    ds18b20.skip();
    ds18b20.write(0x44);
    ::delay(SAMPLING_DELAY_MS);
  
    // Read the contents of the scratchpad (Read Scratchpad [BEh])
    ds18b20.reset();
    ds18b20.skip();
    ds18b20.write(0xBE); // Read Scratchpad
    for (int i = 0; i < SCRATCHPAD_SIZE; ++i) {
      scratchpad[i] = ds18b20.read();
    }
  
    // Validate checksum
    if ( OneWire::crc8(scratchpad, (SCRATCHPAD_SIZE-1)) != scratchpad[(SCRATCHPAD_SIZE-1)]) { continue; }
  
    // Reassemble temperature value
    encoded_temperature = ((static_cast<uint16_t>(scratchpad[1]) << 8) | scratchpad[0]);
    break;
  }
  
  // Disable the sensor
  ::digitalWrite(enable_pin_,LOW);
  //TODO: Determine if the pin be set to INPUT to conserve battery

  // The encoded temperature carries fractional information and is shifted 4 bits to the left.
  // Divide by 16 into float value to preserve fractional information.
  return (static_cast<float>(encoded_temperature)/16.0f);
}

void setup() {
#ifdef ARDUINO_ARCH_ESP32
  // Disable radios to improve power profile
  WiFi.mode(WIFI_OFF);
  ::btStop();
#endif

  // Provide visual signal when the Host MCU is powered
  ::pinMode(LED_BUILTIN, OUTPUT);
  ::digitalWrite(LED_BUILTIN, HIGH);

  // Initialize Debug Output
  serialDebug.begin(115200);
  static const size_t MAX_SERIAL_WAIT_MS = 5000;
  size_t begin_serial_wait_ms = ::millis();
  while (!serialDebug && (MAX_SERIAL_WAIT_MS > (::millis() - begin_serial_wait_ms))) {
    ; // wait for serial port to connect. Needed for native USB
  }
  notecard.setDebugOutputStream(serialDebug);

  // Initialize Notecard
  notecard.begin();

  // Configure Notecard to synchronize with Notehub periodically, as well as
  // adjust the frequency based on the battery level
  {
    J * req = notecard.newRequest("hub.set");
    JAddStringToObject(req, "product", productUID);
    JAddStringToObject(req, "mode", "periodic");
    JAddStringToObject(req, "vinbound", "usb:60;high:120;normal:240;low:480;dead:0");
    JAddStringToObject(req, "voutbound", "usb:30;high:60;normal:90;low:120;dead:0");
    notecard.sendRequest(req);
  }

  // Optimize voltage variable behaviors for LiPo battery
  {
    J * req = notecard.newRequest("card.voltage");
    JAddStringToObject(req, "mode", "lipo");
    notecard.sendRequest(req);
  }

  // Disable IMU module to improve battery performance
  // NOTE: Also disables Notecard temperature sensor
  //
  // Field observation: Testing confirmed the unit was able to operate on battery
  //                    power without needing to disable the IMU, which provides
  //                    additional context to device environment and behavior.
  //{
  //  J * req = notecard.newRequest("card.motion.mode");
  //  JAddBoolToObject(req, "stop", true);
  //  notecard.sendRequest(req);
  //}

  // Establish a template to optimize queue size and data usage
  {
    J * req = notecard.newRequest("note.template");
    JAddStringToObject(req, "file", "sensor.qo");
    J * body = JAddObjectToObject(req, "body");
    JAddNumberToObject(body, "do", 12);
    JAddNumberToObject(body, "orp", 12);
    JAddNumberToObject(body, "ph", 12);
    JAddNumberToObject(body, "tbt", 12);
    JAddNumberToObject(body, "tds", 12);
    JAddNumberToObject(body, "temp", 12.1);
    notecard.sendRequest(req);
  }

  // Sample Pseudo Sensor and queue results
  {
    J * req = notecard.newRequest("note.add");
    JAddStringToObject(req, "file", "sensor.qo");
    J * body = JAddObjectToObject(req, "body");
    JAddNumberToObject(body, "do", sampleAverageAnalogSignal(DO_PIN,DO_PIN_EN,DO_DELAY_MS));
    JAddNumberToObject(body, "orp", sampleAverageAnalogSignal(ORP_PIN,ORP_PIN_EN,ORP_DELAY_MS));
    JAddNumberToObject(body, "ph", sampleAverageAnalogSignal(PH_PIN,PH_PIN_EN,PH_DELAY_MS));
    JAddNumberToObject(body, "tbt", sampleAverageAnalogSignal(TBT_PIN,TBT_PIN_EN,TBT_DELAY_MS));
    JAddNumberToObject(body, "tds", sampleAverageAnalogSignal(TDS_PIN,TDS_PIN_EN,TDS_DELAY_MS));
    JAddNumberToObject(body, "temp", sampleDs18b20(TEMP_PIN,TEMP_PIN_EN));
    notecard.sendRequest(req);
  }

  // Check environment variable for sleep period
  {
    J * req = notecard.newRequest("env.get");
    JAddStringToObject(req, "name", "period_s");
    J * rsp = notecard.requestAndResponse(req);

    // Process Note contents
    if (JHasObjectItem(rsp,"text")) {
      J * text_obj = JGetObjectItem(rsp, "text");
      if (JIsString(text_obj)) {
        PERIOD_S = static_cast<size_t>(::atoi(JStringValue(text_obj)));
      }
    }

    // Delete response
    notecard.deleteResponse(rsp);
  }
}

void loop() {
  // Request sleep from loop to safeguard against tranmission failure, and
  // ensure sleep request is honored so power usage is minimized.
  {
    // Create a "command" instead of a "request", because the host
    // MCU is going to power down and cannot receive a response.
    J * req = NoteNewCommand("card.attn");
    JAddStringToObject(req, "mode", "sleep");
    JAddNumberToObject(req, "seconds", PERIOD_S);
    notecard.sendRequest(req);
  }

  // Wait 1s before retrying
  ::delay(1000);
}
