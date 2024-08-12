#include <WiFi.h>
#include <WiFiClient.h>
#include <WiFiServer.h>

// Replace with your network credentials
const char* ssid = "Stereo_Camera_Left98B7401C0610";
const char* password = "";

// Define the LED pin for DAC output (Use GPIO 25 or 26)
const int dacPin = 25; // GPIO 25 is DAC1

// Define the static IP, gateway, and subnet mask
IPAddress local_IP(192, 168, 4, 10);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

// Define the server port
const int serverPort = 12345;

// Create a WiFiServer object
WiFiServer server(serverPort);

void setup() {
  // Initialize serial communication
  Serial.begin(115200);

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
    dacWrite(dacPin, value); // Write analog value to DAC pin
    delay(1000);
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
    } else {
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
  //dacWrite(dacPin, 0);
  //
//  delay(1000); // Wait before starting again
}

void resetBrightness() {
  // Reset brightness to zero
  dacWrite(dacPin, 0);
  delay(1000); // Wait before starting again
}
