/*********
  Rui Santos
  Complete project details at http://randomnerdtutorials.com  
*********/

// Load Wi-Fi library
//#include <ESP8266WiFi.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "CameraWebServer_AP.h"
#include <ArduinoJson.h>

CameraWebServer_AP CameraWebServerAP;

// Replace with your network credentials
//const char* ssid     = "Verizon_NV9QLQ";
//const char* password = "foal9-tip-sow";
  const char *ssid = "Stereo_Camera_Left98B7401C0610";
  const char *password = "";

IPAddress local_IP(192, 168, 4, 2);
IPAddress gateway(192, 168, 4, 1);
IPAddress subnet(255, 255, 255, 0);

#define RXD2 33
#define TXD2 4

// Set web server port number to 80
WiFiServer server(80);
WiFiClient client = server.available();


// Variable to store the HTTP request
//String header;

// Auxiliar variables to store the current output state
//String output5State = "off";
//String output4State = "off";

// Assign output variables to GPIO pins
//const int output5 = 5;
//const int output4 = 4;

// Current time
unsigned long currentTime = millis();
// Previous time
unsigned long previousTime = 0; 
// Define timeout time in milliseconds (example: 2000ms = 2s)
const long timeoutTime = 2000;

void setup() {
  Serial.begin(9600);
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  CameraWebServerAP.CameraWebServer_AP_Init();
  // Initialize the output variables as outputs
 // pinMode(output5, OUTPUT);
 // pinMode(output4, OUTPUT);
  // Set outputs to LOW
 // digitalWrite(output5, LOW);
 // digitalWrite(output4, LOW);

  // Connect to Wi-Fi network with SSID and password
  Serial.print(" Connecting to ");
  Serial.println(ssid);
  client.print(" Connecting to ");
  client.println(ssid);

  //WiFi.mode()
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
    client.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  client.println("");
  client.println("WiFi connected.");
  client.println("IP address: ");
  client.println(WiFi.localIP());
  server.begin();
  for(int i=1; i<5;i++){
  testClientInput("{\"N\":102,\"D1\":2, \"D2\":255}");
  }

  //testClientInput("{\"N\":102,\"D1\":2, \"D2\":255}");

}

bool WA_en = false;
// Function to simulate client input for testing
void testClientInput(const String& simulatedInput) {
    WiFiClient client;
    String readBuff;
    bool data_begin = true;

     for (char c : simulatedInput) {
        Serial.print(c); // Print to Serial Monitor for debugging
        // Simulate client.read() by processing each character
        if (true == data_begin && c == '{') { // Received start character
            data_begin = false;
        }
        if (false == data_begin && c != ' ') { // Remove spaces
            readBuff += c;
        }
        if (false == data_begin && c == '}') { // Received end character
            data_begin = true;
            Serial2.print(readBuff); // Send to Arduino via Serial2
            readBuff = "";
        }
    }
}

void SocketServer_Test(void)
{
  static bool ED_client = true;
  WiFiClient client = server.available(); //尝试建立客户对象
  
  if (client)                             //如果当前客户可用
  {
    WA_en = true;
    ED_client = true;
    Serial.println("[Client connected]");
    client.println("[Client connected]");

    String readBuff;
    String sendBuff;
    uint8_t Heartbeat_count = 0;
    bool Heartbeat_status = false;
    bool data_begin = true;
    while (client.connected()) //如果客户端处于连接状态
   while (true) //如果客户端处于连接状态
    {
      if (client.available()) //如果有可读数据
      if (true) //如果有可读数据
      {
        char c = client.read();             //读取一个字节
        //char c = "{
       //   "N": 106,
        //  "D1": 3
        //}";
       // const char* c = "{\"N\": 106, \"D1\": 3}";


        Serial.print(c);                    //从串口打印
        if (true == data_begin && c == '{') //接收到开始字符
        {
          data_begin = false;
        }
        if (false == data_begin && c != ' ') //去掉空格
        {
          readBuff += c;
        }
        if (false == data_begin && c == '}') //接收到结束字符
        {
          data_begin = true;
          if (true == readBuff.equals("{Heartbeat}"))
          {
            Heartbeat_status = true;
          }
          else
          {
            Serial2.print(readBuff);
          }
          Serial2.print(readBuff);
          readBuff = "";
        }
      }
      if (Serial2.available())
      {
        char c = Serial2.read();
        sendBuff += c;
        //Serial.print(c);
        if (c == '}') //接收到结束字符
        {
          client.print(sendBuff);
          Serial.print(sendBuff); //从串口打印
          sendBuff = "";
        }
      }

      static unsigned long Heartbeat_time = 0;
      if (millis() - Heartbeat_time > 1000) //心跳频率
      {
       // client.print("{Heartbeat}");
        if (true == Heartbeat_status)
        {
          Heartbeat_status = false;
          Heartbeat_count = 0;
        }
        else if (false == Heartbeat_status)
        {
          Heartbeat_count += 1;
        }
        if (Heartbeat_count > 3)
        {
          Heartbeat_count = 0;
          Heartbeat_status = false;
          break;
        }
        Heartbeat_time = millis();
      }
      static unsigned long Test_time = 0;
      if (millis() - Test_time > 1000) //定时检测连接设备
      {
        Test_time = millis();
        //Serial2.println(WiFi.softAPgetStationNum());
        if (0 == (WiFi.softAPgetStationNum())) //如果连接的设备个数为“0” 则向车模发送停止命令
        {
          Serial2.print("{\"N\":100}");
          //break;
        }
      }
    }
    Serial2.print("{\"N\":100}");
    client.stop(); //结束当前连接:
    Serial.println("[Client disconnected]");
    client.println("[Client disconnected]");


  }
  else
  {
    if (ED_client == true)
    {
      ED_client = false;
      Serial2.print("{\"N\":100}");
    }
  }
}



/*作用于测试架*/
void FactoryTest(void)
{
  static String readBuff;
  String sendBuff;
  if (Serial2.available())
  {
    char c = Serial2.read();
    readBuff += c;
    if (c == '}') //接收到结束字符
    {
      if (true == readBuff.equals("{BT_detection}"))
      {
        Serial2.print("{BT_OK}");
        Serial.println("Factory...");
      }
      else if (true == readBuff.equals("{WA_detection}"))
      {
        Serial2.print("{");
        //Serial2.print(CameraWebServerAP.wifi_name);
        Serial2.print("}");
        Serial.println("Factory...");
      }
      readBuff = "";
    }
  }
  {
    if ((WiFi.softAPgetStationNum())) //连接的设备个数不为“0” led指示灯长亮
    {
      if (true == WA_en)
      {
        digitalWrite(13, LOW);
        Serial2.print("{WA_OK}");
        WA_en = false;
      }
    }
    else
    {
      //获取时间戳 timestamp
      static unsigned long Test_time;
      static bool en = true;
      if (millis() - Test_time > 100)
      {
        if (false == WA_en)
        {
          Serial2.print("{WA_NO}");
          WA_en = true;
        }
        if (en == true)
        {
          en = false;
          digitalWrite(13, HIGH);
        }
        else
        {
          en = true;
          digitalWrite(13, LOW);
        }
        Test_time = millis();
      }
    }
  }
}

void loop(){
  //WiFiClient client = server.available();   // Listen for incoming clients

  /*if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    currentTime = millis();
    previousTime = currentTime;
    while (client.connected() && currentTime - previousTime <= timeoutTime) { // loop while the client's connected
      currentTime = millis();         
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
            
            // turns the GPIOs on and off
            if (header.indexOf("GET /5/on") >= 0) {
              Serial.println("GPIO 5 on");
              output5State = "on";
              digitalWrite(output5, HIGH);
            } else if (header.indexOf("GET /5/off") >= 0) {
              Serial.println("GPIO 5 off");
              output5State = "off";
              digitalWrite(output5, LOW);
            } else if (header.indexOf("GET /4/on") >= 0) {
              Serial.println("GPIO 4 on");
              output4State = "on";
              digitalWrite(output4, HIGH);
            } else if (header.indexOf("GET /4/off") >= 0) {
              Serial.println("GPIO 4 off");
              output4State = "off";
              digitalWrite(output4, LOW);
            }
            
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #195B6A; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #77878A;}</style></head>");
            
            // Web Page Heading
            client.println("<body><h1>ESP8266 Web Server</h1>");
            
            // Display current state, and ON/OFF buttons for GPIO 5  
            client.println("<p>GPIO 5 - State " + output5State + "</p>");
            // If the output5State is off, it displays the ON button       
            if (output5State=="off") {
              client.println("<p><a href=\"/5/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/5/off\"><button class=\"button button2\">OFF</button></a></p>");
            } 
               
            // Display current state, and ON/OFF buttons for GPIO 4  
            client.println("<p>GPIO 4 - State " + output4State + "</p>");
            // If the output4State is off, it displays the ON button       
            if (output4State=="off") {
              client.println("<p><a href=\"/4/on\"><button class=\"button\">ON</button></a></p>");
            } else {
              client.println("<p><a href=\"/4/off\"><button class=\"button button2\">OFF</button></a></p>");
            }
            client.println("</body></html>");
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");*/
       SocketServer_Test();
       FactoryTest();
    String incomingMessage_from_arduino;
    while (Serial2.available()) {
      // Read the incoming message
      //char c = Serial2.read();
      //String 
      incomingMessage_from_arduino = Serial2.readStringUntil('\n');
      Serial.println(incomingMessage_from_arduino);
      client.println(incomingMessage_from_arduino);

     // client.println(incomingMessage_from_arduino);
      //String incomingMessage = Serial2.readStringUntil('\n');
      // Print the received message to the Serial Monitor
      //Serial.print("ESP32 received: ");
      //Serial.println(incomingMessage);
      // Parse the incoming message
    //parseMessage(incomingMessage);
  }
   //WiFiClient client = server.available();
  //while (client.connected()){
  //  client.println(incomingMessage_from_arduino);
  //}
  //delay(1000);  // Add a delay to avoid flooding
}


void parseMessage(const String& message) {
  if (message.startsWith("{") && message.endsWith("}")) {
    // Remove the curly braces
    String content = message.substring(1, message.length() - 1);

    // Split the message into parts
    int separatorIndex = content.indexOf('_');
    if (separatorIndex != -1) {
      String commandNumber = content.substring(0, separatorIndex);
      String dataPart = content.substring(separatorIndex + 1);

      Serial.print("Command Number: ");
      Serial.println(commandNumber);

      if (dataPart == "true" || dataPart == "false") {
        // Handle boolean data
        bool hasObstacle = (dataPart == "true");
        Serial.print("Ultrasonic detection status: ");
        Serial.println(hasObstacle ? "Obstacle detected" : "No obstacle");
      } else {
        // Handle numeric data
        int distance = dataPart.toInt();
        Serial.print("Ultrasonic distance: ");
        Serial.print(distance);
        Serial.println(" cm");
      }
    } else {
      Serial.println("Invalid message format.");
    }
  } else {
    Serial.println("Invalid message received.");
  }
}