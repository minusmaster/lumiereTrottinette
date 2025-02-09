#include <Arduino.h>
#include <U8g2lib.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_NeoPixel.h>

// Si le SPI est disponible, inclure la bibliothèque SPI
#ifdef U8X8_HAVE_HW_SPI
#include <SPI.h>
#endif

// Si l'I2C est disponible, inclure la bibliothèque Wire
#ifdef U8X8_HAVE_HW_I2C
#include <Wire.h>
#endif

// Définition des broches pour le MPU6050
#define MPU6050_SDA_PIN 5
#define MPU6050_SCL_PIN 6

// Définition des broches pour les fonctions (commenté pour le moment)
//#define SDA_PIN 21
//#define SCL_PIN 22
//#define KLAXON_PIN 20
//#define SWITCH_KLAXON_PIN 21
#define SWITCH_BRAKE_RIGHT_PIN 0
#define SWITCH_BRAKE_LEFT_PIN 1
#define SWITCH_RIGHT_PIN 3
#define SWITCH_LEFT_PIN 4
#define REAR_RIGHT_LED_DATA_PIN 7
#define REAR_LEFT_LED_DATA_PIN 8

// Définition des constantes pour le MPU6050 et les NeoPixels
#define MOTION_THRESHOLD 2.0
#define INACTIVITY_TIMEOUT 300000
#define BLINK_INTERVAL 500
#define TAILLIGHT_INTENSITY 10
#define BRAKELIGHT_INTENSITY 100
#define NUM_LEDS_PER_RING 7
#define NUM_RINGS 2
#define debug 0
#define actifMPU 0

// Initialisation des objets pour le MPU6050, les NeoPixels et l'écran OLED
Adafruit_MPU6050 mpu;
Adafruit_NeoPixel rightRing(NUM_LEDS_PER_RING * 2, REAR_RIGHT_LED_DATA_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel leftRing(NUM_LEDS_PER_RING * 2, REAR_LEFT_LED_DATA_PIN, NEO_GRB + NEO_KHZ800);
U8G2_SSD1306_72X40_ER_F_HW_I2C u8g2(U8G2_R0, /* reset=*/U8X8_PIN_NONE);

// Variable pour la visibilité de la boîte sur l'écran OLED
bool isBoxVisible = true;

// Initialisation des variables de temps et d'état
unsigned long lastMotionTime = 0;
bool isBlinking = false;
bool isLeftSwitchClosed = false;
bool isRightSwitchClosed = false;
bool isLeftBrakeSwitchClosed = false;
bool isRightBrakeSwitchClosed = false;
bool isKlaxonSwitchClosed = false;
unsigned long lastBlinkTime = 0;

// Préparer l'affichage sur l'écran OLED
void u8g2_prepare(void) {
  u8g2.setFont(u8g2_font_6x10_tf);
  u8g2.setFontRefHeightExtendedText();
  u8g2.setDrawColor(1);
  u8g2.setFontPosTop();
  u8g2.setFontDirection(0);
}

// Dessiner un cadre sur l'écran OLED
void u8g2_box_frame() {
  u8g2.drawStr(0, 0, "drawBox");
  u8g2.drawBox(5, 10, 20, 10);
  u8g2.drawBox(10, 15, 30, 7);
  u8g2.drawStr(0, 30, "drawFrame");
  u8g2.drawFrame(5, 10 + 30, 20, 10);
  u8g2.drawFrame(10, 15 + 30, 30, 7);
}

// Remplir les NeoPixels avec une couleur spécifiée
void fillLedsWithColor(Adafruit_NeoPixel& strip, uint32_t color, uint8_t brightnessPercent) {
  strip.setBrightness(map(brightnessPercent, 0, 100, 0, 255));
  for (int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
  }
  strip.show();
}

// Clignoter les NeoPixels en orange et noir
void blinkOrangeBlack(String direction, Adafruit_NeoPixel& ring) {
  unsigned long currentTime = millis();
  static unsigned long lastBlinkTime = 0;
  static bool isOrange = true;

  int numPixels = ring.numPixels();
  int startIndex = (direction == "LEFT") ? 0 : numPixels / 2; // Index de début pour la moitié gauche ou droite du ring

  if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
    lastBlinkTime = currentTime;
    uint32_t color = isOrange ? ring.Color(255, 165, 0) : 0; // Orange ou éteint

    for (int j = startIndex; j < startIndex + numPixels / 2; j++) {
      ring.setPixelColor(j % numPixels, color); // Utiliser le modulo pour s'assurer que l'index reste dans la plage
    }
    ring.setBrightness(BRAKELIGHT_INTENSITY);
    ring.show();
    isOrange = !isOrange;
  }
}

// Afficher un texte centré sur l'écran OLED
void afficherTexteCentre(const char* texte, unsigned long tempsAffichage) {
  u8g2.clearBuffer();
  int textWidth = u8g2.getStrWidth(texte);
  int startX = (u8g2.getWidth() - textWidth) / 2;
  int startY = 20;
  u8g2.drawStr(startX, startY, texte);
  u8g2.sendBuffer();
  delay(tempsAffichage);
}

void tailLight() {
  fillLedsWithColor(leftRing, 0xFF0000, TAILLIGHT_INTENSITY);
  fillLedsWithColor(rightRing, 0xFF0000, TAILLIGHT_INTENSITY);
  // Dessiner les hachures sur la zone du feux stop
    for(int i = 0; i < 50; i+=2){
      u8g2.drawLine(11 + i, 30, 11 + i, 40);
    }
}

void stopLight() {
  fillLedsWithColor(leftRing, 0xFF0000, BRAKELIGHT_INTENSITY);
  fillLedsWithColor(rightRing, 0xFF0000, BRAKELIGHT_INTENSITY);
  u8g2.drawBox(11, 30, 50, 10);  // feux stop
}

// Initialisation des périphériques et des capteurs
void setup() {
  Wire.begin(MPU6050_SDA_PIN, MPU6050_SCL_PIN);
  u8g2.begin();
  u8g2.setFont(u8g2_font_6x10_tf);
  afficherTexteCentre("ALLUMAGE", 500);

  leftRing.begin();
  rightRing.begin();
  if (!leftRing.getPixels() || !rightRing.getPixels()) {
    afficherTexteCentre("! NeoPixels non détectés !", 2000);
    while (1)
      ;  // Boucle infinie en cas de problème de connexion
  }
  leftRing.setBrightness(20);
  rightRing.setBrightness(20);

  afficherTexteCentre("LUMIERES", 500);
  leftRing.begin();
  rightRing.begin();
  leftRing.setBrightness(20);
  rightRing.setBrightness(20);

  if (actifMPU) {
    if (actifMPU && debug && !mpu.begin()) {
      afficherTexteCentre("! MPU6050 !", 2000);
      while (1)
        ;  // Boucle infinie si le MPU6050 n'est pas connecté
    } else {
      afficherTexteCentre("GYRO", 500);
      mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
      mpu.setGyroRange(MPU6050_RANGE_500_DEG);
      mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
    }
  }
  afficherTexteCentre("OK", 500);

  //on met en marche les feux arrièresf (taillight)
  tailLight();
}

void loop(void) {
  u8g2.clearBuffer();  // clear the internal memory
  //u8g2.setFont(u8g2_font_ncenB08_tr);	// choose a suitable font
  //u8g2.drawStr(0,10,"Hello World!");	// write something to the internal memory
  if (isBoxVisible) {
    u8g2.drawBox(0, 0, 10, 19);  // clignotant avant gauche
  }
  // Dessiner des boîtes sur l'écran OLED pour simuler les feux
  u8g2.drawBox(0, 0, 10, 19);    // clignotant avant gauche
  u8g2.drawBox(0, 20, 10, 19);   // clignotant avant droit
  u8g2.drawBox(62, 0, 10, 19);   // clignotant a droit
  u8g2.drawBox(62, 20, 10, 19);  // clignotant arriere gauche
  u8g2.drawBox(11, 0, 50, 10);   // feux avant
  //u8g2.drawBox(11, 30, 50, 10);  // feux stop

  

  // Remplir les NeoPixels avec des couleurs spécifiques
  fillLedsWithColor(leftRing, 0xFF0000, 50);   // Rouge à 50% de luminosité
  fillLedsWithColor(rightRing, 0x00FF00, 50);  // Vert à 70% de luminosité

  // Lire les données du gyroscope
  if (actifMPU) {
    sensors_event_t accelEvent, gyroEvent, tempEvent;
    mpu.getEvent(&accelEvent, &gyroEvent, &tempEvent);

    // Vérifier si le vélo est en mouvement en fonction des lectures du gyroscope
    if (mpu.begin()) {
      if (abs(gyroEvent.gyro.x) > MOTION_THRESHOLD || abs(gyroEvent.gyro.y) > MOTION_THRESHOLD || abs(gyroEvent.gyro.z) > MOTION_THRESHOLD) {
        afficherTexteCentre("MOVE", 500);
        u8g2.drawBox(11, 0, 50, 10);                                  // feux avant
        fillLedsWithColor(leftRing, 0xFF0000, TAILLIGHT_INTENSITY);   // Rouge à 20% de luminosité
        fillLedsWithColor(rightRing, 0xFF0000, TAILLIGHT_INTENSITY);  // Rouge à 20% de luminosité
        lastMotionTime = millis();
      } else {
        // Le vélo n'est pas en mouvement, vérifier le délai d'inactivité
        unsigned long currentTime = millis();
        //afficherTexteCentre("----", 500);
        if (currentTime - lastMotionTime >= INACTIVITY_TIMEOUT) {
          // Convertir la différence de temps en une chaîne de caractères
          char timeDifferenceStr[20];
          sprintf(timeDifferenceStr, "%lu", currentTime - lastMotionTime);

          // Afficher la différence de temps
          afficherTexteCentre(timeDifferenceStr, 500);
          Serial.print((String) "INACTIVITY_TIMEOUT:" + currentTime + "-" + lastMotionTime + "\n");
          // Le délai d'inactivité est atteint, éteindre les lumières
          // -> code pour éteindre la lumière avant
          fillLedsWithColor(leftRing, 0xFF0000, 0);
          fillLedsWithColor(rightRing, 0xFF0000, 0);
        }
      }
    }
  }
  
  // Vérifier l'état du clignotant gauche
  isLeftSwitchClosed = digitalRead(SWITCH_LEFT_PIN) == LOW;
  isRightSwitchClosed = digitalRead(SWITCH_RIGHT_PIN) == LOW;

  // Si le clignotant gauche est fermé, activer le clignotement orange pour le premier anneau
  if (isLeftSwitchClosed) {
    isBlinking = true;
    blinkOrangeBlack("LEFT", leftRing);
    afficherTexteCentre("cli gauche", 500);
    lastMotionTime = millis();
  }
  else if (isRightSwitchClosed) {
    isBlinking = true;
    blinkOrangeBlack("RIGHT", rightRing);
    afficherTexteCentre("cli droit", 500);
    lastMotionTime = millis();
  }
  else {
    isBlinking = false;                       
    tailLight();
  }

  // Vérifier l'état de l'interrupteur de frein
  isLeftBrakeSwitchClosed = digitalRead(SWITCH_BRAKE_LEFT_PIN) == LOW;
  isRightBrakeSwitchClosed = digitalRead(SWITCH_BRAKE_RIGHT_PIN) == LOW;

  // Si l'interrupteur de frein est fermé, activer le clignotement orange pour le premier anneau
  if (isLeftBrakeSwitchClosed || isRightBrakeSwitchClosed) {
    stopLight();
    afficherTexteCentre("frein!", 500);
    lastMotionTime = millis();
  } else if (isBlinking == false) {                     
    tailLight();
    afficherTexteCentre("taillight", 500);
  }

  // Vérifier l'état de l'interrupteur du klaxon
  //isKlaxonSwitchClosed = digitalRead(SWITCH_KLAXON_PIN) == LOW;
  // Si l'interrupteur du klaxon est fermé, activer la sirène
  //if (isKlaxonSwitchClosed) {
  //  digitalWrite(KLAXON_PIN, LOW);
  //} else {
  //  digitalWrite(KLAXON_PIN, HIGH);
 // }

  u8g2.sendBuffer();  // transfer internal memory to the display

  // Attendre un certain délai avant de répéter la boucle
  delay(1000);
}
