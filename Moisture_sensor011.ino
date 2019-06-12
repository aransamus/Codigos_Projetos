//Programa: Monitoracao de planta usando Arduino - Planta Robô
//Inclusão dos leds RGB 
//Inclusão do Sensor de temperatura e umidade
//Inclusão do Buzzer - 20-10-2018 Versão 09.01
//Inclusão do Coller/Temperatura  - 20-10-2018 Versão 10.01
//Inclusão da Bomba de Agua  - 20-10-2018 Versão 11.01
//Autor: Jonas Esteves 

//------Led Endereçavel
#include <Adafruit_NeoPixel.h> //Inclusão da biblioteca Led
#ifdef __AVR__
  #include <avr/power.h>
#endif

#include "DHT.h" //Inclusão do DHT

#define PIN 13 //pino do led
#define NUMPIXELS 10 //Quantidade de leds 
// Parameter 1 = number of pixels in strip
// Parameter 2 = pin number (most are valid)
// Parameter 3 = pixel type flags, add together as needed:
//   NEO_KHZ800  800 KHz bitstream (most NeoPixel products w/WS2812 LEDs)
//   NEO_KHZ400  400 KHz (classic 'v1' (not v2) FLORA pixels, WS2811 drivers)
//   NEO_GRB     Pixels are wired for GRB bitstream (most NeoPixel products)
//   NEO_RGB     Pixels are wired for RGB bitstream (v1 FLORA pixels, not v2)
Adafruit_NeoPixel pixels = Adafruit_NeoPixel(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

//Sensor de Temperatura 
#define DHTPIN A1 // pino que estamos conectado
#define DHTTYPE DHT11 // DHT 11
// Conecte pino 1 do sensor (esquerda) ao +5V
// Conecte pino 2 do sensor ao pino de dados definido em seu Arduino
// Conecte pino 4 do sensor ao GND
// Conecte o resistor de 10K entre pin 2 (dados) 
// e ao pino 1 (VCC) do sensor
DHT dht(DHTPIN, DHTTYPE);

//------Definição dos pinos
#define pino_sinal_analogico A0 //Sensor de solo
int valor_analogico; //Valor do Sensor de solo
#define vibra 3 //Motor de Vibração
#define buzzer 9 //Buzzer
float seno;
int frequencia;

//Motores - Definicoes pinos Arduino ligados a entrada da Ponte H
int IN1 = 4; //Motor Cooler +
int IN2 = 5; //Motor Cooler - 
int IN3 = 6; //Motor Agua +
int IN4 = 7; //Motor Agua -

void acendeSensor_solo();//Funçao que controla o acendimento do Sensor_solo
void acendeVibra();//Funçao que controla o acendimento do Sensor_solo
void acendeDHT();//Funçao que controla o acendimento do Sensor_solo

//int vibra = 3; //Motor de vibração

unsigned long tempoSensor_solo=0; //Variavel para armazenar o tempo do Sensor_solo
unsigned long tempoVibra=0; //Variavel para armazenar o tempo do Sensor_solo
unsigned long tempoDHT=0; //Variavel para armazenar o tempo do Sensor_solo

bool comutarSensor_solo = false;
bool comutarVibra = false;
bool comutarDHT = false;
 
void setup()
{
  Serial.begin(9600);
//  strip.begin();//Leds endereçaveis
//  strip.show(); // Initialize all pixels to 'off'

//Led Enderecavel
#if defined (__AVR_ATtiny85__)
  if (F_CPU == 16000000) clock_prescale_set(clock_div_1);
#endif
  // End of trinket special code
  pixels.begin(); // This initializes the NeoPixel library.


  //Sensor de temperatura e umidade
  Serial.println("DHTxx test!");
  // Sensores
  dht.begin(); //Sensor de temperatura e umidade
  
  pinMode(pino_sinal_analogico, INPUT);
  pinMode(vibra, OUTPUT);
  pinMode(buzzer,OUTPUT);

  //Motores - Define os pinos como saida
  pinMode(IN1, OUTPUT); //Motor Cooler +
  pinMode(IN2, OUTPUT); //Motor Cooler - 
  pinMode(IN3, OUTPUT); //Motor Agua +
  pinMode(IN4, OUTPUT); //Motor Agua -
}
 
void loop()
{
  Sensor_solo(); // Sensor de solo
  Vibra();      // Motor de vibração
  espera();     // Loop de espera - controle de tempo
  //RGBLoop();    //Leds endereçaveis
  Sensor_DHT();
  
}

void Sensor_solo(){
    if(comutarSensor_solo){
    for(int i=0;i<NUMPIXELS;i++){ 
    //----Comutar
            //Le o valor do pino A0 do sensor
            valor_analogico = analogRead(pino_sinal_analogico);
           
            //Mostra o valor da porta analogica no serial monitor
            Serial.print("Porta analogica: ");
            Serial.print(valor_analogico);
                   
            //Solo umido, acende o led verde
            if (valor_analogico > 0 && valor_analogico < 300)
            {
              Serial.println(" Status: Solo umido");
              pixels.setPixelColor(i, pixels.Color(0,150,0)); // Moderately bright green color.
//              digitalWrite(vibra, LOW);
//              Serial.println("não vibra");
              noTone(buzzer);// Sem som
              //Bomba de Agua
              digitalWrite(IN3, HIGH); //Motor Agua +
              digitalWrite(IN4, HIGH); //Motor Agua -
              pixels.show(); // This sends the updated pixel color to the hardware.
            }
           
            //Solo com umidade moderada, acende led amarelo
            if (valor_analogico > 300 && valor_analogico < 620)
            {
              Serial.println(" Status: Umidade moderada");
              pixels.setPixelColor(i, pixels.Color(250,250,0)); // Moderately bright green color.
//              digitalWrite(vibra, LOW);
//              Serial.println("não vibra");
              noTone(buzzer);// Sem som
              //Bomba de Agua
              digitalWrite(IN3, HIGH); //Motor Agua +
              digitalWrite(IN4, HIGH); //Motor Agua -
              pixels.show(); // This sends the updated pixel color to the hardware.
            }
           
            //Solo seco, acende led vermelho
            if (valor_analogico > 620 && valor_analogico < 1024)
            {
              Serial.println(" Status: Solo seco");
              pixels.setPixelColor(i, pixels.Color(250,0,0)); // Moderately bright green color.
              //digitalWrite(vibra, HIGH);
              //Serial.println("vibra");
              tone(buzzer, 100);// Envia um sinal sonoro 1KHz 
              //Bomba de Agua
              digitalWrite(IN3, HIGH); //Motor Agua +
              digitalWrite(IN4, LOW); //Motor Agua -
              pixels.show(); // This sends the updated pixel color to the hardware.
            }
            
            }
    //----Comutar
    tempoSensor_solo=millis();
    comutarSensor_solo=false;
    }
}

void Vibra(){
        if(comutarVibra){
        digitalWrite(vibra, !digitalRead(vibra)); //Comando que comuta o estado do pino, se era alto vira baixo e vice-versa
        Serial.println("vibra");
        tempoVibra=millis();
        comutarVibra=false;
        }
}

void Sensor_DHT(){
          if(comutarDHT){
          digitalWrite(DHTPIN, !digitalRead(DHTPIN)); //Comando que comuta o estado do pino, se era alto vira baixo e vice-versa

          // A leitura da temperatura e umidade pode levar 250ms!
            // O atraso do sensor pode chegar a 2 segundos.
            float h = dht.readHumidity();
            float t = dht.readTemperature();
            // testa se retorno é valido, caso contrário algo está errado.
            if (isnan(t) || isnan(h)) 
            {
              Serial.println("Failed to read from DHT");
            } 
            else 
            {
              Serial.print("Umidade: ");
              Serial.print(h);
              Serial.print(" %t");
              Serial.print("Temperatura: ");
              Serial.print(t);
              Serial.println(" *C");
                     
                      // Condição de Temperatura.
                      if (dht.readTemperature() >= 28) {
                      digitalWrite(IN1, HIGH);//Cooler Temperatura +
                      digitalWrite(IN2, LOW); //Cooler Temperatura -
                      }
                      if (dht.readTemperature() <= 27) {
                      digitalWrite(IN1, HIGH);//Cooler Temperatura +
                      digitalWrite(IN2, HIGH); //Cooler Temperatura -
                      }
                      // Fim Condição de Temperatura.
            }

        tempoDHT=millis();
        comutarDHT=false;
        }
}



//-----------Controle de Tempo
void espera(){
    //-------Sensor de Solo
    if(!comutarSensor_solo && millis()-tempoSensor_solo>=3000){
    comutarSensor_solo = true;
    }
    //-------Vibra
    if(!comutarVibra && millis()-tempoVibra>=3000){
    comutarVibra = true;
    }
    //-------Sensor de Umidade e temperatura
    if(!comutarDHT && millis()-tempoDHT>=5000){
    comutarDHT = true;
    }
}
 
