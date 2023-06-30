#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>  //https://github.com/bbx10/WebServer_tng


WebServer server ( 80 );

const char* ssid     = "same";
const char* password = "12345678";

void setup() 
{ 
  Serial.begin(115200);
  connectToWifi();
  beginServer();
}

void loop() {
 server.handleClient();
 delay(1000);
}

void connectToWifi()
{
  WiFi.enableSTA(true); 
  delay(2000);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void beginServer()
{
  server.on ("/", handleRoot);
  server.begin();
  Serial.println ( "HTTP server started" );
}

void handleRoot(){ 
    server.send ( 200, "text/html", getPage() );
}

String getPage(){
  String page = "<html lang=en-EN><head><meta http-equiv='refresh' content='60'/>";
  page += "<title>ESP32 WebServer - educ8s.tv</title>";
  page += "<style> body { background-color: #fffff; font-family: Arial, Helvetica, Sans-Serif; Color: #000000; }</style>";
  page += "</head><body><h1>ESP32 WebServer</h1>";
  page += "<h3>WASTE CLASSIFIER MR2P1</h3>";
  page += "<ul><li>waste type: ";
  page += temperature;
  page += "</body></html>";
  return page;
}