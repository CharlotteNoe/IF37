#include <HardwareSerial.h>
#include <DFRobotDFPlayerMini.h>

// === Définitions des broches ===
#define BOUTON_SPEAKER    13   // Bouton pour le son
#define PIN_MOSFET        14   // Contrôle de la pompe (MOSFET)
#define BOUTON_POMPE      12   // Bouton pour la pompe
#define ELECTROVANNE_PIN 27    // Contrôle de l'électrovanne (MOSFET)
#define PIN_POTEN        33    // Pin pour le potentiomètre
#define DFPLAYER_RX 16  // ESP32 RX (DFPlayer TX)
#define DFPLAYER_TX 17  // ESP32 TX (DFPlayer RX)

HardwareSerial dfplayerSerial(1);  // UART1
DFRobotDFPlayerMini dfplayer;

// === États internes ===
bool speaker_on = false; // Etat des hauts parleurs
bool pompe_on   = false; // Etat de la pompe

int last_btn_speaker = HIGH;
int last_btn_pompe   = HIGH;

unsigned long cycleStartTime = 0;
int phase = 0; // Différentes phases de la respiration :  0: off, 1: gonfle, 2: pause, 3: dégonfle

void setup() {
  Serial.begin(115200);

  // Initialisation UART pour DFPlayer
  dfplayerSerial.begin(9600, SERIAL_8N1, DFPLAYER_RX, DFPLAYER_TX);

  if (!dfplayer.begin(dfplayerSerial)) {
    Serial.println("Erreur DFPlayer");
  } else {
    Serial.println("DFPlayer OK");
  }

  // Attribution des pinmode
  pinMode(BOUTON_SPEAKER, INPUT_PULLUP);
  pinMode(BOUTON_POMPE, INPUT_PULLUP);
  pinMode(PIN_MOSFET, OUTPUT);
  pinMode(ELECTROVANNE_PIN, OUTPUT);

  digitalWrite(ELECTROVANNE_PIN, HIGH);
  digitalWrite(PIN_MOSFET, LOW); // Ouvre le système d'air pour éviter des soucis techniques
}

void loop() {
  checkSpeakerButton();
  checkPompeButton();
  if (speaker_on) {
    checkPotentiometre();
  }
  if (pompe_on) {
    updateRespirationCycle();
  }
  delay(5);
}

void checkSpeakerButton() {
  int btn = digitalRead(BOUTON_SPEAKER);
  if (btn != last_btn_speaker) {
    if (btn == LOW) {
      speaker_on = !speaker_on;
      if (speaker_on) {
        dfplayer.play(1); // joue le premier fichier mp3 sur la carte SD
        Serial.println("🔊 Lecture son DFPlayer ON");
      } else {
        dfplayer.pause();
        Serial.println("🔇 Lecture son DFPlayer OFF");
      }
    }
    last_btn_speaker = btn;
  }
}

void checkPompeButton() {
  int btn = digitalRead(BOUTON_POMPE);
  if (btn != last_btn_pompe) {
    if (btn == LOW) {
      pompe_on = !pompe_on;
      if(pompe_on){
        phase = 1; // Démarre le cycle
        cycleStartTime = millis();
        Serial.println("💨 Respiration START");
        digitalWrite(ELECTROVANNE_PIN, LOW);
        digitalWrite(PIN_MOSFET, HIGH); 
        Serial.println("🔁 Gonflage (4s)");
      }   
      else {
        digitalWrite(ELECTROVANNE_PIN, HIGH);
        digitalWrite(PIN_MOSFET, LOW);
        phase = 0;
        Serial.println("💨 Respiration STOP"); 
      }
    }
     last_btn_pompe = btn;
  }
}


void checkPotentiometre() {
  static int last_volume = -1;
  int analog_value = analogRead(PIN_POTEN); // 0 - 4095
  int volume = map(analog_value, 0, 4095, 0, 30); // Volume 0 - 30
  if (volume != last_volume) {
    dfplayer.volume(volume);  // règle le volume DFPlayer
    last_volume = volume;
  }
}


void updateRespirationCycle() {
  unsigned long now = millis();
  unsigned long elapsed = now - cycleStartTime;

  switch (phase) {
    case 1: // Gonflage (4s)
      if (elapsed >= 4000) {
        digitalWrite(PIN_MOSFET, LOW);        // Pompe OFF
        phase = 2;
        cycleStartTime = now;
        Serial.println("⏸️ Pause (7s)");
      }
      break;

    case 2: // Pause (7s)
      if (elapsed >= 7000) {
        phase = 3;
        cycleStartTime = now;
        Serial.println("⬇️ Dégonflage (8s)");
        digitalWrite(ELECTROVANNE_PIN, HIGH); // Ferme la vanne (pression relâchée)
      }
      break;

    case 3: // Dégonflage (8s)
      if (elapsed >= 8000) {
        phase = 1;
        cycleStartTime = now;
        digitalWrite(ELECTROVANNE_PIN, LOW); 
        digitalWrite(PIN_MOSFET, HIGH);
        Serial.println("✅ Respiration terminée");
        Serial.println("🔁 Gonflage (4s)");
      }
      break;
  }
}
