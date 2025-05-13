#include <Wire.h>
#include <Adafruit_BMP3XX.h>
#include <TinyGPSPlus.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define LED_PIN 2

// Objetos do sensor e GPS
Adafruit_BMP3XX bmp;
TinyGPSPlus gps;
HardwareSerial SerialGPS(2); // RX=16, TX=17

float lastAltitude = 0.0;

void setup()
{
    Serial.begin(115200);
    SerialGPS.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17

    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Inicializa o sensor BMP380
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
}

void loop()
{
    // ----------------- Leitura do BMP380 -----------------
    if (bmp.performReading())
    {
        float currentAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

        Serial.print("Altitude (BMP380): ");
        Serial.print(currentAltitude);
        Serial.println(" m");

        if (currentAltitude < lastAltitude - 0.1)
        {
            digitalWrite(LED_PIN, HIGH);
        }
        else
        {
            digitalWrite(LED_PIN, LOW);
        }

        lastAltitude = currentAltitude;
    }
    else
    {
        Serial.println("Erro na leitura do BMP380!");
    }

    // ----------------- Leitura do GPS -----------------
    while (SerialGPS.available() > 0)
    {
        gps.encode(SerialGPS.read());
    }

    if (gps.location.isUpdated())
    {
        Serial.print("Latitude: ");
        Serial.println(gps.location.lat(), 6);

        Serial.print("Longitude: ");
        Serial.println(gps.location.lng(), 6);

        if (gps.date.isValid() && gps.time.isValid())
        {
            Serial.print("Data: ");
            Serial.print(gps.date.day());
            Serial.print("/");
            Serial.print(gps.date.month());
            Serial.print("/");
            Serial.println(gps.date.year());

            Serial.print("Hora: ");
            Serial.print(gps.time.hour());
            Serial.print(":");
            Serial.print(gps.time.minute());
            Serial.print(":");
            Serial.println(gps.time.second());
        }
        else
        {
            Serial.println("Hora/Data inválidas");
        }
    }
    else
    {
        Serial.println("Aguardando sinal de GPS...");
    }

    delay(1000);
}
