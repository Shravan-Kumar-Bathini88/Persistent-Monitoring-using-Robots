#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

// Replace with your network credentials
const char* ssid = "Stereo_Camera_Left98B7401C0610";
const char* password = "";

// Define the LED pin for DAC output (Use GPIO 25 or 26)
const int RED = 32; // GPIO 25 is DAC1
const int YELLOW = 25;
const int GREEN = 26;

// Define PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 8; // 8-bit resolution means duty cycle can be from 0-255


// Define the static IP, gateway, and subnet mask
IPAddress local_IP(192, 168, 4, 12);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);
// Define the server port
const int serverPort = 12345;

// Create a WiFiServer object
WiFiServer server(serverPort);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  //pinMode(RED, OUTPUT);
  // Configure the LED PWM functionalitites
  ledcSetup(ledChannel, freq, resolution);

  // Attach the LED PWM channel to the GPIO pin
  ledcAttachPin(RED, ledChannel);


  // Attempt to configure the static IP
  if (!WiFi.config(local_IP, gateway, subnet)) {
    Serial.println("Failed to configure static IP");
  } else {
    Serial.println("Static IP configured successfully");
  }

  // Start connecting to Wi-Fi
  Serial.println("Connecting to Wi-Fi...");
  WiFi.begin(ssid, password);

  // Wait for the Wi-Fi connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print(".");
  }

  // When connected, print the IP address
  Serial.println("");
  Serial.println("Wi-Fi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  // Start the server
  server.begin();

    int value = 255;
    //dacWrite(RED, value);
    //digitalWrite(RED, HIGH);

          for (int i =0;i<=255;i++){
            ledcWrite(ledChannel, i);
            delay(100);
          }
    //ledcWrite(ledChannel, 255);
    ledcWrite(ledChannel, 0);

    dacWrite(YELLOW, value);
    delay(10000);
    dacWrite(GREEN, value);
    dacWrite(YELLOW, 0); // Write analog value to DAC pin
    delay(1000);
    //dacWrite(RED, 0);
    //digitalWrite(RED, LOW);
    //ledcWrite(ledChannel, 0);
    //dacWrite(YELLOW, 0);
       for (int i =255;i>=0;i--){
            dacWrite(GREEN, i);
            delay(100);


      }

}

void loop() {
  // Check for incoming client connections
  WiFiClient client = server.available();
  
  if (client) {
    Serial.println("Client connected");

    // Read the incoming command from the client
    String command = client.readStringUntil('\n');
    command.trim(); // Remove any trailing newline or spaces

    if (command == "RESET") {
      // Reset brightness
      Serial.println("Resetting brightness");
      resetBrightness();
    } 
    else if (command == "RED"){
      //Serial.println()
      //dacWrite(RED, 255);
      //digitalWrite(RED, HIGH);
      dacWrite(YELLOW, 0);
      dacWrite(GREEN, 0);
        // Gradually increase the brightness
      for (int dutyCycle = 0; dutyCycle <= 255; dutyCycle++) {
         // Change the brightness
        ledcWrite(ledChannel, dutyCycle);
       // Wait for a bit to see the change
         delay(10);
         }

      delay(1000);

    }
      else if (command == "YELLOW"){
      //Serial.println()
      dacWrite(YELLOW, 255);
      //digitalWrite(RED, LOW);
      ledcWrite(ledChannel, 0);
      dacWrite(GREEN, 0);
      delay(1000);

    }
      else if (command == "GREEN"){
      //Serial.println()
     
      //digitalWrite(RED, LOW);
      ledcWrite(ledChannel, 0);
      dacWrite(YELLOW, 0);
      for (int i =255;i>=0;i--){
            dacWrite(GREEN, i);
            delay(10);


      }
      delay(1000);

    }
    else {
      Serial.println("Unknown command");
    }

    // Close the client connection
    client.stop();
    Serial.println("Client disconnected");
  }

  // Gradually increase the brightness using DAC
  //for (int value = 0; value <= 255; value++) {
 // Adjust delay for desired fade speed
  //}

  // Reset brightness
  //dacWrite(RED, 0);
  //
//  delay(1000); // Wait before starting again
}

void resetBrightness() {
  // Reset brightness to zero
  //dacWrite(RED, 0);
  delay(1000); // Wait before starting again
}
