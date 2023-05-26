//"""""""""""""""""""""""""""

#include <DFRobot_SIM808.h>
#include <SoftwareSerial.h>

#define PIN_TX 11
#define PIN_RX 10
SoftwareSerial mySerial(PIN_TX, PIN_RX);
DFRobot_SIM808 sim808(&mySerial); // Connect RX,TX,PWR,

// DFRobot_SIM808 sim808(&Serial);

// char http_cmd[] = "GET /media/uploads/mbed_official/hello.txt HTTP/1.0\r\n\r\n";
char http_cmd[] = "GET /237fe907-1177-4599-b2f0-4c2d2006d9d9 HTTP/1.1\r\nHost: webhook.site\r\n\r\n";
// char http_cmd[] = "GET /ama/api/add_data?dev_id=myid&lat=70.6093748&lng=56.5514707&tem=25&wt=442&hum=42&pid=1 HTTP/1.0\r\n\r\n";
char buffer[1024];

void setup()
{
    pinMode(8, OUTPUT);
    digitalWrite(8, HIGH);

    mySerial.begin(9600);
    Serial.begin(9600);

    //******** Initialize sim808 module *************
    while (!sim808.init())
    {
        delay(1000);
        Serial.print("Sim808 init error\r\n");
    }
    delay(3000);

    //*********** Attempt DHCP *******************
    while (!sim808.join(F("internet")))
    { // Make sure it is the name of your APN
        Serial.println("Sim808 join network error");
        delay(2000);
    }

    //************ Successful DHCP ****************
    Serial.print("IP Address is ");
    Serial.println(sim808.getIPAddress());

    //*********** Establish a TCP connection ************
    // if(!sim808.connect(TCP,"mbed.org", 80)) {
    if (!sim808.connect(TCP, "webhook.site", 80))
    {
        Serial.println("Connect error");
    }
    else
    {
        Serial.println("Connect webhook.site successfull");
    }

    //*********** Send a GET request *****************
    Serial.println("waiting to fetch...");
    sim808.send(http_cmd, sizeof(http_cmd) - 1);
    while (true)
    {
        int ret = sim808.recv(buffer, sizeof(buffer) - 1);
        if (ret <= 0)
        {
            Serial.println("fetch over...");
            break;
        }
        buffer[ret] = '\0';
        Serial.print("Recv: ");
        Serial.print(ret);
        Serial.print(" bytes: ");
        Serial.println(buffer);
        break;
    }

    //************* Close TCP or UDP connections **********
    sim808.close();

    //*** Disconnect wireless connection, Close Moving Scene *******
    sim808.disconnect();
}

void loop()
{
}

//"""""""""""""""""""""""""""