// Walter W Glockner

#include <Kawasaki_FFF.h>
#include <SPI.h>
#include <Ethernet.h>

struct P1AMConfig {
  byte mac[6] = { 0x60, 0x52, 0xD0, 0x07, 0x7F, 0xCB };
  IPAddress ip = IPAddress (192, 168, 1, 2);
  EthernetServer server = EthernetServer(10000);
  EthernetClient clients[8]; // Support up to 8 clients
} config;

struct DeviceState {
    float extrudeFlowrate = 0.0, extruderTemp = 0.0;
    bool isPrinting = false, isPaused = false, startFlag;
    bool automated = false;
} extruderState;

Extruder Extruder;
SL4848CR Heater1;
ModbusClientKAW ModbusClientKAW;
unsigned long lastIOSendTime = 0;
const unsigned long ioUpdateInterval = 50; // Milliseconds


signed int extruderData[25];
signed int heater1Data[25];
int extruderMotorEnable = 0;



int heater1Address = 1;
float eMotorRPM = 0.0;
float layerHeight = 0.0;
float firstLayerHeight = 0.0;
float defaultLineWidth = 0.0;
float firstLayerLineWidth = 0.0;

bool isClientConnected = false;
bool wasClientConnected = false;
bool isGuiConnected = false;

uint32_t outputStates[9];  // Array to store the state of output pins
channelLabel kawGoesMOOvePin = { 3, 1 };
channelLabel extrusionMotorAnalogPin = { 7, 1 };
channelLabel clearPin = { 3, 9 };
channelLabel airPin = { 4, 2 };
channelLabel airAdjustPin = { 7, 8 };
channelLabel extrusionPin = { 2, 10 };
channelLabel retractionPin = { 2, 11 };
const int rumblerPin = 2;// no channel because GPIO module
int channel, pin, state;

char incomingByte;
bool stopFlag = 0;

void setup() {
  Serial.begin(9600);
  delay(100);
  while (!P1.init());
  Ethernet.begin(config.mac, config.ip);
  config.server.begin();
  initializeHardware();
}

void initializeHardware() {
  // Stop extrusion motor
  Extruder.stopEMotor(kawGoesMOOvePin, extrusionMotorAnalogPin);
  // Initialize all P1AM outputs to LOW
  P1.writeDiscrete(LOW,airPin);
  // Set heater temps to 0

  //Initialize Modbus
  while (!ModbusClientKAW.init());
  // Wait until initializations have completed before 
  // turning on ClearCore
    Heater1.setSetPointValue(heater1Address, 0);
  delay(3000);
  P1.writeDiscrete(HIGH, clearPin);
}

// Handle incoming new Clients
void acceptNewClients() {
  EthernetClient newClient = config.server.available();
  if (newClient) {
    bool existing = false;
    for (int i = 0; i < 8; i++) {
      if (config.clients[i] == newClient) {
        existing = true;
        break;
      }
    }
    if (!existing) {
      for (int i = 0; i < 8; i++) {
        if (!config.clients[i]) {
          config.clients[i] = newClient;
          Serial.println("New client connected: " + String(i));
          return;
        }
      }
      Serial.println("New client attempted to connect, but max client limit reached.");
      newClient.stop();
    }
  }
}

// Read from new clients
void handleClients() {
  for (int i = 0; i < 8; i++) {
    if (config.clients[i]) {
      if (config.clients[i].available()) {
        String currentLine = config.clients[i].readStringUntil('\n');
        handleCommand(currentLine, config.clients[i]);
      }
      if (!config.clients[i].connected()) {
        Serial.println("Client " + String(i) + " disconnected.");
        config.clients[i].stop();
      }
    }
  }
}


void handleOutputCommand(int channel, int pin, bool state) {
  P1.writeDiscrete(state, channel, pin);  // Assuming P1 is a method or object dealing with I/O
  Serial.print("Output set on channel ");
  Serial.print(channel);
  Serial.print(", pin ");
  Serial.print(pin);
  Serial.println(state ? " HIGH" : " LOW");
}

void handleCommand(String command, EthernetClient &client) {
  Serial.println(command);

  //int channel, pin, state;

  if (command.startsWith("SET_OUTPUT:")) {
    if (sscanf(command.c_str(), "SET_OUTPUT:%d:%d:%d", &channel, &pin, &state) == 3) {
      handleOutputCommand(channel, pin, state);
      client.println("Output command processed.");
    } else {
      client.println("Invalid output command.");
    }
    return;
  }

 if (command == "command:start_print") {
    if (!extruderState.isPrinting) {
      extruderState.isPrinting = true;
      extruderState.startFlag = true;
      //Extruder.setEnableMotor(Extruder.getExtrudeFlowrate(), extrusionMotorAnalogPin, kawGoesMOOvePin);
      P1.writeAnalog((2048/800)*Extruder.getExtrudeFlowrate()+2048,extrusionMotorAnalogPin);

      Serial.println((2048/800)*Extruder.getExtrudeFlowrate()+2048);
      
      //P1.writeAnalog(Extruder., extrusionMotorAnalogPin);
      P1.writeDiscrete(HIGH, kawGoesMOOvePin);
      //Extruder.setEnableMotor(Extruder.getExtrudeFlowrate(), extrusionMotorAnalogPin, kawGoesMOOvePin);
      P1.writeAnalog((2048/800)*Extruder.getExtrudeFlowrate()+2048,extrusionMotorAnalogPin);

      Serial.println((2048/800)*Extruder.getExtrudeFlowrate()+2048);
      client.println("Print started.");
      Serial.println("Print started.");
    } else {
      client.println("Print already in progress.");
      Serial.println("Print already in progress.");
      //Extruder.setEnableMotor(Extruder.getExtrudeFlowrate(), extrusionMotorAnalogPin, kawGoesMOOvePin);
      P1.writeAnalog((2048/800)*Extruder.getExtrudeFlowrate()+2048,extrusionMotorAnalogPin);

      Serial.println((2048/800)*Extruder.getExtrudeFlowrate()+2048);
      
    }
  } else if (command == "command:stop_print") {
    if (extruderState.isPrinting) {
      extruderState.isPrinting = false;
      extruderState.startFlag = false;
      P1.writeAnalog(2048, extrusionMotorAnalogPin);

      P1.writeDiscrete(LOW, kawGoesMOOvePin);
      client.println("Print stopped.");
      while(!Serial.println("Print stopped."));
    } else {
      client.println("No print to stop.");
      Serial.println("No print to stop.");
    }
  } else if (command == "command:pause_print") {
    if (extruderState.isPrinting && !extruderState.isPaused) {
      extruderState.isPaused = true;
      P1.writeAnalog(2048, extrusionMotorAnalogPin);

      P1.writeDiscrete(LOW, kawGoesMOOvePin);
      client.println("Print paused.");
      Serial.println("Print paused.");
    } else {
      client.println("Print already paused or not started.");
      Serial.println("Print already paused or not started.");
    }
  } else if (command == "command:resume_print") {
    if (!extruderState.isPrinting && extruderState.isPaused) {
      extruderState.isPrinting = true;
      extruderState.isPaused = false;
      //Extruder.setEnableMotor(Extruder.getExtrudeFlowrate(), extrusionMotorAnalogPin, kawGoesMOOvePin);
      P1.writeAnalog((2048/800)*Extruder.getExtrudeFlowrate()+2048,extrusionMotorAnalogPin);

      Serial.println((2048/800)*Extruder.getExtrudeFlowrate());
      P1.writeDiscrete(HIGH, kawGoesMOOvePin);
      client.println("Print resumed.");
      Serial.println("Print resumed.");
    } else {
      client.println("Print not paused or already running.");
      Serial.println("Print not paused or already running.");
    }
  }
}


  void loop() {
    if(extruderState.isPrinting){
      //while(!Extruder.setEnableMotor(Extruder.getExtrudeFlowrate(), extrusionMotorAnalogPin, kawGoesMOOvePin));
      P1.writeAnalog((2048.0/800.0)*Extruder.getExtrudeFlowrate()+2048.0,extrusionMotorAnalogPin);

      Serial.println((2048.0/800.0)*Extruder.getExtrudeFlowrate()+2048.0);

      //Serial.println((2048.0/800.0)*Extruder.getExtrudeFlowrate()+2048.0);
      //Serial.println(Extruder.getExtrudeFlowrate());
      P1.writeDiscrete(HIGH, kawGoesMOOvePin);
    }

  acceptNewClients(); // Accept a new client to connect to the sever
  handleClients(); // handle the inputs from the clients
  // Loop through all connected clients 
  // to receive their commands and send 
  // commands to them
  for (int i = 0; i < 8; i++) {
    if (config.clients[i].connected()) {
      static String currentLine[8];  // Declare an array of String for each client to persist across loop calls
      while (config.clients[i].available()) {
        char c = config.clients[i].read();
        Serial.print(c);  // Output every character received for debugging

        if (c == '\n') {
          Serial.println("Received newline character.");
          if (currentLine[i] == "REQUEST_IO_UPDATE") {
            sendIODigitalSignals(config.clients[i]);  // Send the IO state update immediately upon request
          }
          if (currentLine[i].startsWith("SET_OUTPUT:")) {
            int channel, pin, state;
            if (sscanf(currentLine[i].c_str(), "SET_OUTPUT:%d:%d:%d", &channel, &pin, &state) == 3) {
              handleOutputCommand(channel, pin, state);
              config.clients[i].println("Output command processed.");
            }
          }

          int separatorPos = currentLine[i].indexOf(':');
          if (separatorPos != -1) {
            String variableName = currentLine[i].substring(0, separatorPos);
            String valueStr = currentLine[i].substring(separatorPos + 1);
            parseVariable(variableName, valueStr, config.clients[i]);
          }
          currentLine[i] = "";  // Reset currentLine after processing
        } else if (c != '\r') {
          currentLine[i] += c;  // Accumulate the line until newline is found
        }
      }

      if (currentLine[i] == "GUI_CONNECTED") {
        config.clients[i].println("ACK_CONNECTED");  // Acknowledge connection
        Serial.println("GUI successfully connected.");
      }
    }
  }
}
// Sends the digitial IO signals from 
// channel 1 and channel 2 
void sendIODigitalSignals(EthernetClient &client) {
  String channel_1_states = "channel_1_states:";
  String channel_2_states = "channel_2_states:";
  for (int channel = 1; channel <= 2; channel++) {
    for (int pin = 1; pin <= 16; pin++) {
      bool state = P1.readDiscrete({ channel, pin });
      if (channel == 1) {
        channel_1_states += state ? "1" : "0";
        if (pin != 16) {
          channel_1_states += ",";
        }
      }
      if (channel == 2) {
        channel_2_states += state ? "1" : "0";
        if (pin != 16) {
          channel_2_states += ",";
        }
      }
    }
  }
  Serial.println(channel_1_states);  // Debugging statement
  Serial.println(channel_2_states);  // Debugging statement
  if (client.connected()) {
    client.println(channel_1_states + "\n" + channel_2_states + "\n");
  }
}

// Write the pin coming from the PC
void setOutputState(int channel, int pin, bool state) {
  P1.writeDiscrete(state, channel, pin);
  Serial.print("Set output for Channel ");
  Serial.print(channel);
  Serial.print(", Pin ");
  Serial.print(pin);
  Serial.println(state ? " HIGH" : " LOW");
}

void parseVariable(String variableName, String valueStr, EthernetClient &client) {
  
  // Set Extrude Flowrate
  if (variableName == "extrudeFlowrate") {
    extruderState.extrudeFlowrate = valueStr.toFloat();
    Extruder.setExtrudeFlowrate(extruderState.extrudeFlowrate);
    Serial.print("Extrude Flowrate set:");
    Serial.println(extruderState.extrudeFlowrate);
  } 
  // Set extruder temperature
  else if (variableName == "temperature") {
    extruderState.extruderTemp = valueStr.toFloat();
    Heater1.setSetPointValue(heater1Address, extruderState.extruderTemp);
    Serial.print("Extruder state extruder temp = ");
    Serial.println(extruderState.extruderTemp);
  } 
  // Turn air on
  else if (variableName == "air") {
    P1.writeDiscrete(valueStr == "on" ? HIGH : LOW, airPin);
  } 
  // Allow control of extrusion to be externally controlled
  else if (variableName == "automated") {
    extruderState.automated = valueStr == "on";
  } 
  // Set extrusion motor RPM
  else if (variableName == "eMotorRPM") {
    eMotorRPM = valueStr.toFloat();
    Extruder.setExtrudeFlowrate(eMotorRPM);
    Serial.println("Set extrude flowrate to: ");
    Serial.println(eMotorRPM);

  } 
  // Set layer Height
  else if (variableName == "layer_height") {
    layerHeight = valueStr.toFloat();
  } 
  // Set first layer height
  else if (variableName == "first_layer_height") {
    firstLayerHeight = valueStr.toFloat();
  } 
  // Set Default Line Width
  else if (variableName == "default_line_width") {
    defaultLineWidth = valueStr.toFloat();
  } 
  // Set First Layer Line width
  else if (variableName == "first_layer_line_width") {
    firstLayerLineWidth = valueStr.toFloat();
  } 
  // Turn on the rumbler
  else if (variableName == "rumbler") {
    if(valueStr == "on\n"){
      
    }
    analogWrite(rumblerPin, valueStr == "off" ? 0:1);
    Serial.println("Toggled Rumbler");
  } 
  // Disconnect from the server
  else if (variableName == "stop_client") {
    client.stop();
    isClientConnected = false;
  }
}
