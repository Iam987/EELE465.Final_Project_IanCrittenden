#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>

HardwareSerial MySerial(1); //UART 1
const int MySerialRX_PIN = 16;
const int MySerialTX_PIN = 17;
char Rx_Buf[32];
int Rx_index = 0;
char Tx_Buf[32];
int Tx_index = 0;

int GarageState = 0;
float INTEMP = 4;
float OUTTEMP = 5;
float INTEMP_F = 4;
float OUTTEMP_F = 5;
char State = 'L';
String Pass = "1010A";

/* Put your SSID & Password */
const char* ssid = "ShedWhyFi";
const char* password = "Alphabet$oup";

WebServer server(555);

void setup() {
  Serial.begin(115200);

  MySerial.begin(9600, SERIAL_8N1, MySerialRX_PIN, MySerialTX_PIN);

  WiFi.begin(ssid, password); 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();
  
  server.on("/",handle_OnConnect);
  server.on("/Refresh",handle_Refresh);
  server.on("/GarageOpen",handle_GarageOpen);
  server.on("/GarageClosed",handle_GarageClosed);
  server.on("/Lock",handle_Lock);
  server.on("/1010A", handle_Unlock);
  server.onNotFound(handle_NotFound);
  
  server.begin();
  Serial.println("HTTP server started");
}
void loop() {
  server.handleClient();
}

void request_data(){
  MySerial.write('R');
  delay(500);
  while(MySerial.available()>0){
    Rx_Buf[Rx_index] = MySerial.read();
    Rx_index += 1;
  }
  Rx_index = 0;
  OUTTEMP = Rx_Buf[0];
  OUTTEMP += 0.5 * Rx_Buf[1];
  INTEMP = Rx_Buf[2];
  INTEMP += 0.5 * Rx_Buf[3] + 1;
  OUTTEMP_F = OUTTEMP * 1.8 + 32;
  INTEMP_F = INTEMP * 1.8 + 32;
  GarageState = Rx_Buf[4];
}

void handle_OnConnect() {
  Serial.println("Client Connected");
  request_data();
  server.send(200, "text/html", SendHTML()); 
}

void handle_Lock() {
  State = 'L';
  server.send(200, "text/html",SendHTML());
}

void handle_Unlock(){
  if(Pass == "1010A"){
    State = 'U';
  }
  server.send(200, "text/html",SendHTML());

}

void handle_GarageOpen() {
  if(State == 'U'){
  Serial.println("Garage Opening");
  GarageState = 1;
  Rx_Buf[2] = 1;
  MySerial.write('O');
  delay(100);
  }
  request_data();
  server.send(200, "text/html", SendHTML()); 
}

void handle_GarageClosed() {
  if(State == 'U'){
  Serial.println("Garage Closing");
  GarageState = 0;
  Rx_Buf[2] = 0;
  MySerial.write('C');
  delay(100);
  }
  request_data();
  server.send(200, "text/html", SendHTML()); 
}

void handle_Refresh(){
  Serial.println("Refresh");
  request_data();
  server.send(200, "text/html", SendHTML());
}

void handle_NotFound(){
  server.send(404, "text/plain", "Not found");
}

String SendHTML(){
  String ptr = "<!DOCTYPE html> <html>\n";
  ptr +="<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1.0, user-scalable=no\">\n";
  ptr +="<title>Home Status</title>\n";
  ptr +="<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}\n";
  ptr +="body{margin-top: 50px;} h1 {color: #444444;margin: 50px auto 30px;} h3 {color: #444444;margin-bottom: 50px;}\n";
  ptr +=".button {display: block;width: 80px;background-color: #29c900;border: none;color: white;padding: 13px 30px;text-decoration: none;font-size: 25px;margin: 0px auto 35px;cursor: pointer;border-radius: 4px;}\n";
  ptr +=".button-open {background-color: #29c900;}\n";
  ptr +=".button-open:active {background-color: #2980b9;}\n";
  ptr +=".button-close {background-color: #c90000;}\n";
  ptr +=".button-close:active {background-color: #2c3e50;}\n";
  ptr +="p {font-size: 25px;color: #888;margin-bottom: 10px;}\n";
  ptr +="h4 {font-size: 25px;color: #a81d09;margin-bottom: 10px;}\n";
  ptr +="</style>\n";
  ptr +="</head>\n";
  ptr +="<body>\n";
  ptr +="<h1>EELE465_Final_Project Web Server - Ian Crittenden</h1>\n";
  
  if(State == 'L'){
    ptr +="<h4>Locked!</h4>\n";
  }
  else{
    ptr +="<a class=\"button button-close\" href=\"/Lock\">Lock</a>\n";
    if(GarageState)
    {ptr +="<p>Garage Status: Open</p><a class=\"button button-close\" href=\"/GarageClosed\">Close</a>\n";}
    else
    {ptr +="<p>Garage Status: Closed</p><a class=\"button button-open\" href=\"/GarageOpen\">Open</a>\n";}
  }

  ptr +="<a class=\"button button-open\" href=\"/Refresh\">Refresh</a>\n";
  ptr +="<p>Outdoor Temp: " + String(OUTTEMP,1) + "&deg;C, "+ String(OUTTEMP_F,1) + "&deg;F</p>\n";
  ptr +="<p>Indoor Temp: " + String(INTEMP,1) + "&deg;C, "+ String(INTEMP_F,1) + "&deg;F</p>\n";

  ptr +="</body>\n";
  ptr +="</html>\n";
  return ptr;
}