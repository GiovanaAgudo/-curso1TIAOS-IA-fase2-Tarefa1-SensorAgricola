// ======================= FarmTech – Irrigação Inteligente (Laranja) =======================
// Objetivo: simular um sistema de irrigação usando ESP32 + DHT22 + LDR(pH) + 3 botões (NPK) + relé

#include <Arduino.h>
#include <DHTesp.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

// ----------------------------- Mapeamento de Pinos (ESP32) ------------------------------
// Botões dos nutrientes (usei pull-up interno, então apertado = LOW)
const int PIN_BTN_N = 26;   // Nitrogênio
const int PIN_BTN_P = 25;   // Fósforo
const int PIN_BTN_K = 33;   // Potássio

// Sensores e atuador
const int PIN_LDR   = 34;   // LDR no pino ADC (AO do módulo)
const int PIN_DHT   = 15;   // DHT22 (pino DATA)
const int PIN_RELAY = 27;   // Relé (pino IN)

// Muitos módulos de relé são "ativo em nível LOW"
const int RELAY_ACTIVE_LEVEL = LOW;
const int RELAY_IDLE_LEVEL   = HIGH;

// --------------------- Anti-ruído de botões e anti-oscilação da bomba -------------------
// Debounce dos botões: 16 amostras de 5 ms = ~80 ms
const uint8_t  BTN_SAMPLES     = 16;
const uint16_t BTN_INTERVAL_MS = 5;

// Anti-chatter da bomba: tempo mínimo em cada estado (evita liga/desliga frenético)
const unsigned long PUMP_MIN_ON_MS  = 2500;  // mínimo ligada: 2.5 s
const unsigned long PUMP_MIN_OFF_MS = 1500;  // mínimo desligada: 1.5 s
static unsigned long lastPumpChange  = 0;    // salvo quando a bomba troca de estado

// ------------------------------ Regras para Laranja ---------------------------
// Para ligar, considero pH numa faixa um pouco mais "apertada".
// Para manter ligada, deixo a faixa mais "larga" (histerese de pH).
const float PH_ON_MIN    = 5.9;
const float PH_ON_MAX    = 6.9;
const float PH_KEEP_MIN  = 5.6;
const float PH_KEEP_MAX  = 7.2;

// Faixas de umidade (histerese): liga <45%, desliga >=55%
const float HUMI_ON      = 45.0;
const float HUMI_OFF     = 55.0;

// Emergências
const float HUMI_EMERGENCY     = 35.0;  // seca extrema: liga independente de pH/NPK
const float TEMP_EMERGENCY_ON  = 40.0;  // calor extremo: liga independente de pH/NPK
const float TEMP_EMERGENCY_OFF = 35.0;  // só "desarma" o calor quando esfriar até aqui

// No simulador, a curva do LDR ficou invertida pra mim (mais luz → pH menor), então uso true
bool INVERT_LDR = true;

// ------------------------------ Estado do sistema / sensores ----------------------------
// Bomba atual
bool pumpOn = false;

// Flag para saber se a bomba foi LIGADA por calor. Só nesse caso o "esfriou" desliga.
static bool heatEmergency = false;

// Telemetria no Serial a cada 1 s (só pra não floodar)
unsigned long lastPrint = 0;

// DHT com "cache": a lib fica instável se chamar MUITO rápido; então leio a cada 1,5 s
DHTesp dht;
static TempAndHumidity thCache;
static unsigned long lastDhtRead = 0;
const  unsigned long DHT_INTERVAL_MS = 1500;

// ------------------------------------ Utilitários ---------------------------------------
// Leio o botão várias vezes (com intervalo) e só considero "apertado" se deu LOW na maioria.
bool readButtonStable(int pin, uint16_t samples = 5, uint16_t intervalMs = 2) {
  uint16_t countLow = 0;
  for (uint16_t i = 0; i < samples; i++) {
    if (digitalRead(pin) == LOW) countLow++;    // com INPUT_PULLUP: apertado = LOW
    delay(intervalMs);
  }
  return (countLow > samples / 2);
}

// Média simples do ADC para o LDR, porque analógico costuma oscilar um pouco.
int analogReadAvg(int pin, int samples = 10) {
  long acc = 0;
  for (int i = 0; i < samples; i++) {
    acc += analogRead(pin);
    delayMicroseconds(300);  
  }
  return (int)(acc / samples);
}

// Converte a leitura do LDR (0..4095) num "pH" 0..14, só para fins didáticos desta simulação.
float mapLdrToPh(int adc) {
  float r = adc / 4095.0f;                      // normalizo para 0..1
  if (INVERT_LDR) r = 1.0f - r;                 // inverto se precisar
  return r * 14.0f;                             // escalo para 0..14
}

// Liga/desliga a bomba no pino do relé e salva quando trocou (pro anti-chatter).
void setPump(bool on) {
  pumpOn = on;
  digitalWrite(PIN_RELAY, on ? RELAY_ACTIVE_LEVEL : RELAY_IDLE_LEVEL);
  lastPumpChange = millis();
}

// -------------------------------------- Setup -------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(200);

  // Botões com pull-up interno: solto=HIGH, apertado=LOW
  pinMode(PIN_BTN_N, INPUT_PULLUP);
  pinMode(PIN_BTN_P, INPUT_PULLUP);
  pinMode(PIN_BTN_K, INPUT_PULLUP);

  // LDR (ADC) e relé
  pinMode(PIN_LDR, INPUT);
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, RELAY_IDLE_LEVEL);

  // DHT22
  dht.setup(PIN_DHT, DHTesp::DHT22);

  // I2C explícito no ESP32 (SDA=21, SCL=22)
  Wire.begin(21, 22);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("FarmTech");
  lcd.setCursor(0, 1); lcd.print("Iniciando...");
  delay(1000);
  lcd.clear();

  Serial.println(F("FarmTech – Irrigacao Inteligente (Laranja)"));
}

// --------------------------------------- Loop -------------------------------------------
void loop() {
  // 1) Leitura "resistente" dos botões (debounce por amostragem)
  bool N_ok = readButtonStable(PIN_BTN_N, BTN_SAMPLES, BTN_INTERVAL_MS);
  bool P_ok = readButtonStable(PIN_BTN_P, BTN_SAMPLES, BTN_INTERVAL_MS);
  bool K_ok = readButtonStable(PIN_BTN_K, BTN_SAMPLES, BTN_INTERVAL_MS);

  // 2) LDR → pH (com média do ADC)
  int   ldrAdc = analogReadAvg(PIN_LDR, 10);
  float ph     = mapLdrToPh(ldrAdc);

  // 3) DHT22 com cache
  unsigned long now = millis();
  if (now - lastDhtRead >= DHT_INTERVAL_MS) {
    thCache = dht.getTempAndHumidity();
    lastDhtRead = now;
  }
  float humi = thCache.humidity;
  float temp = thCache.temperature;

  bool humiValid = !isnan(humi) && humi >= 0 && humi <= 100;
  bool tempValid = !isnan(temp) && temp > -40 && temp < 85;

  // 4) Decisão de irrigação
  bool nutritionOK      = (N_ok && P_ok && K_ok);
  bool phOK_to_turn_on  = (ph >= PH_ON_MIN   && ph <= PH_ON_MAX);   // faixa "estreita" para ligar
  bool phOK_to_keep_on  = (ph >= PH_KEEP_MIN && ph <= PH_KEEP_MAX); // faixa "larga" para manter

  bool wantOn  = false;
  bool wantOff = false;

  // 4.1) Emergências primeiro
  if (humiValid && humi < HUMI_EMERGENCY) {
    // Seca extrema → liga independente de pH/NPK
    wantOn = true;
    heatEmergency = false; // não foi por calor
  }
  else if (tempValid && temp >= TEMP_EMERGENCY_ON) {
    // Calor extremo → liga independente de pH/NPK
    wantOn = true;
    heatEmergency = true;  // marcou que ligou por calor (importante pro "esfriou")
  }
  else {
    // 4.2) Se a bomba tinha sido ligada por CALOR, só desligo quando esfriar
    if (heatEmergency && tempValid && temp <= TEMP_EMERGENCY_OFF) {
      wantOff = true;
      heatEmergency = false; // desarmou a emergência térmica
    }

    // 4.3) Operação normal com histerese (umidade + NPK + pH)
    if (humiValid) {
      if (!pumpOn) {
        // Liga quando umidade baixa e agronomia OK
        if (humi < HUMI_ON && nutritionOK && phOK_to_turn_on) wantOn = true;
      } else {
        // Desliga quando umidade recupera OU algum requisito saiu
        if (humi >= HUMI_OFF || !nutritionOK || !phOK_to_keep_on) wantOff = true;
      }
    }
  }

  // 4.4) Anti-chatter: só permite mudar se respeitou o tempo mínimo no estado anterior
  unsigned long since = now - lastPumpChange;
  if (!pumpOn && wantOn  && since >= PUMP_MIN_OFF_MS) setPump(true);
  if ( pumpOn && wantOff && since >= PUMP_MIN_ON_MS ) setPump(false);

  // 5) Telemetria no Serial (1 Hz)
  if (now - lastPrint > 1000) {
    lastPrint = now;
    Serial.print(F("N:")); Serial.print(N_ok);
    Serial.print(F(" P:")); Serial.print(P_ok);
    Serial.print(F(" K:")); Serial.print(K_ok);
    Serial.print(F(" | pH:")); Serial.print(ph, 2);
    Serial.print(F(" (ADC ")); Serial.print(ldrAdc); Serial.print(F(")"));
    Serial.print(F(" | Umid(%):")); if (humiValid) Serial.print(humi, 1); else Serial.print(F("ERR"));
    Serial.print(F(" | Temp(C):"));  if (tempValid) Serial.print(temp, 1); else Serial.print(F("ERR"));
    Serial.print(F(" | Bomba: "));   Serial.println(pumpOn ? F("ON") : F("OFF"));
    // Debug extra (se precisar, descomentar)
    // Serial.print(F(" | NPKok:")); Serial.print(nutritionOK);
    // Serial.print(F(" phOn:")); Serial.print(phOK_to_turn_on);
    // Serial.print(F(" phKeep:")); Serial.print(phOK_to_keep_on);
    // Serial.print(F(" heat:")); Serial.println(heatEmergency);
  }

  // 6) LCD – só atualizo quando muda (para não "piscar" e nem floodar o I2C)
  static bool prevPump  = !pumpOn; // força uma primeira atualização
  static int  prevHumi10 = -1000;  // guardo umidade em décimos (%*10)

  if (pumpOn != prevPump) {
    prevPump = pumpOn;
    lcd.setCursor(0, 0);
    lcd.print("BOMBA: ");
    if (pumpOn) lcd.print("ON  "); else lcd.print("OFF ");
  }

  int humi10 = (humiValid ? (int)(humi * 10.0f + 0.5f) : -1);
  if (humi10 != prevHumi10) {
    prevHumi10 = humi10;
    lcd.setCursor(0, 1);
    lcd.print("Umid: ");
    if (humiValid) {
      lcd.print(humi, 1);
      lcd.print("%   ");
    }
    else           {
      lcd.print("--.-%   ");
    }
  }
}