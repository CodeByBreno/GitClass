#include <Wire.h>
#include <Adafruit_BMP3XX.h>

#define SEALEVELPRESSURE_HPA (1013.25)
#define LED_PIN 2

Adafruit_BMP3XX bmp;
float lastAltitude = 0.0;

void setup()
{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    // Inicializa o sensor BMP380
    if (!bmp.begin_I2C())
    {
        Serial.println("Erro ao inicializar o BMP380!");
        while (1)
            ;
    }

    // Configurações padrão do BMP380
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);

    // Primeira leitura da altitude
    if (bmp.performReading())
    {
        lastAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);
    }
}

void loop()
{
    if (bmp.performReading())
    {
        float currentAltitude = bmp.readAltitude(SEALEVELPRESSURE_HPA);

        Serial.print("Altitude: ");
        Serial.print(currentAltitude);
        Serial.println(" m");

        // Verifica se a altitude está diminuindo
        if (currentAltitude < lastAltitude - 0.1)
        { // usa uma tolerância de 0.1m
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
        Serial.println("Erro na leitura do sensor!");
    }

    delay(500); // Espera 500 ms entre as leituras
}
