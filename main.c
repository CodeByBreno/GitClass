#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPSPlus.h>
#include <LoRa.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define LED_PIN 2

// BMP380
Adafruit_BMP3XX bmp;
float lastAltitude = 0.0;

// GPS
TinyGPSPlus gps;
HardwareSerial SerialGPS(2); // RX=16, TX=17

// LoRa (pinos podem variar conforme seu módulo)
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 26

void setup()
{

    Serial.begin(9600);
    SerialGPS.begin(9600, SERIAL_8N1, 16, 17); // GPS: RX=16, TX=17

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Inicia BMP380
    if (!bmp.begin_I2C())
    {
        Serial.println("Erro ao inicializar o BMP380!");
        while (1)
            ;
    }

    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    if (bmp.performReading())
    {
        lastAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    }

    // Inicia LoRa
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(915E6)) // ajuste para 433E6 ou 868E6 conforme módulo
    {
        Serial.println("Erro ao iniciar LoRa!");
        while (1)
            ;
    }

    Serial.println("LoRa iniciado com sucesso!");
    Serial.println("BMP380 e GPS iniciados com sucesso!");
}

void loop()
{
    // =================== BMP380 ===================
    float currentAltitude = lastAltitude;
    if (bmp.performReading())
    {
        currentAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
        if (currentAltitude < lastAltitude - 0.1)
            digitalWrite(LED_PIN, HIGH);
        else
            digitalWrite(LED_PIN, LOW);
        lastAltitude = currentAltitude;
    }

    // =================== GPS ===================
    while (SerialGPS.available() > 0)
    {
        gps.encode(SerialGPS.read());
    }

    double latitude = gps.location.isValid() ? gps.location.lat() : 0.0;
    double longitude = gps.location.isValid() ? gps.location.lng() : 0.0;
    int hour = gps.time.isValid() ? gps.time.hour() : -1;
    int minute = gps.time.isValid() ? gps.time.minute() : -1;
    int second = gps.time.isValid() ? gps.time.second() : -1;

    // =================== Serial Debug ===================
    Serial.print("Altitude: ");
    Serial.print(currentAltitude);
    Serial.println(" m");
    Serial.print("Lat: ");
    Serial.println(latitude, 6);
    Serial.print("Lng: ");
    Serial.println(longitude, 6);
    Serial.print("Hora: ");
    Serial.print(hour);
    Serial.print(":");
    Serial.print(minute);
    Serial.print(":");
    Serial.println(second);

    // =================== Envio via LoRa ===================
    String data = String(currentAltitude, 2) + "," +
                  String(latitude, 6) + "," +
                  String(longitude, 6) + "," +
                  String(hour) + "," +
                  String(minute) + "," +
                  String(second);

    LoRa.beginPacket();
    LoRa.print(data);
    LoRa.endPacket();

    Serial.print("Enviado via LoRa: ");
    Serial.println(data);

    delay(2000); // Aguarda 2s
}
