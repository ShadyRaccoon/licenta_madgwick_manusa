    #include <Wire.h>
    #include <Preferences.h>
    #include <MadgwickAHRS.h>

    #define RED   5
    #define GREEN 18
    #define BLUE  23

    #define THUMB  4
    #define INDEX  2
    #define LITTLE 15

    #define WAIT                    3000
    #define FLEX_REFERENCE_TIME     10000
    #define NEUTRAL_REFERENCE_TIME  10000
    #define PALM_ROM_REFERENCE_TIME 10000

    #define CAL_SAMPLES 150

    #define PALM 7

    #define INTERVAL    10
    #define SAMPLE_FREQ 100

    #define DEBOUNCE_DELAY      100
    #define DOUBLE_CLICK_WINDOW 2000
    #define PAUSE_EXIT_HOVER    500

    //flex commands and led output state tracking
    #define COOLDOWN_MS 2000

    #define BLINK_INTERVAL       300   // 300 ms on/off
    #define PAUSE_EXIT_DURATION  500   // 0.5 s hover after unpause

    String gesture_name = "DEFAULT_GESTURE";
    String pause_gesture = "";

    bool exiting_pause = false;

    long idx_last_press_time, idx_cooldown_until;
    bool idx_prev_pressed, idx_require_release;
    int  idx_single_pending;

    long lit_last_press_time, lit_cooldown_until;
    bool lit_prev_pressed, lit_require_release;

    unsigned long blink_last_toggle = 0;
    bool blink_on = false;

      //led color at moment of pause
    int pause_r, pause_g, pause_b;

    //cooldown to prevent single click after double click toggle
    unsigned long pause_exit_deadline = 0;

    //flash handler object
    Preferences prefs;

    enum MODE{
      MODE_NORMAL,
      MODE_HOVER, 
      MODE_PAUSE,
      MODE_LAND
    };

    MODE crt_mode = MODE_NORMAL;

    bool last_raw_index = false, debounced_index = false;
    bool last_raw_little = false, debounced_little = false;

    int last_debounce_time_index  = 0;
    int last_debounce_time_little = 0;

    int index_click_count = 0;
    int index_first_time_click = 0;

    int little_click_count = 0;
    int little_first_time_click = 0;

    int pause_exit_time = 0;

    int thumb_min = 4095, thumb_max = 0;
    int index_min = 4095, index_max = 0;
    int little_min = 4095, little_max = 0;

    int index_threshold = 2048;
    int little_threshold = 2048;

    int palm_r = 255, palm_g = 255, palm_b = 255;

    float pitch_min = 10000, pitch_max = -10000;
    float roll_min = 10000, roll_max = -10000;

    float pitch_neutral, roll_neutral;

    long last_print = 0;

    float gX_palm, gY_palm, gZ_palm;

    float offX_palm, offY_palm, offZ_palm;

    float aX_palm, aY_palm, aZ_palm;

    float roll_palm, pitch_palm;

    float pitch_lo_dead, pitch_hi_dead;
    float roll_lo_dead, roll_hi_dead;

    void led_color(int red, int green, int blue){
      analogWrite(RED, red);
      analogWrite(GREEN, green);
      analogWrite(BLUE, blue);
    }

    void led_wait(){
      unsigned long start = millis();
      unsigned long lastToggle = start;
      bool on = true;

      while (millis() - start < WAIT) {
        if (millis() - lastToggle >= 300) {
          on = !on;
          lastToggle = millis();
          if (on) {
            led_color(255, 165, 0);
          } else {
            led_color(0, 0, 0);
          }
        }
      }
    }

    void led_blink(int r, int g, int b){
      unsigned long lastToggle = millis();
      bool on = true;

      while (true) {
        if (millis() - lastToggle >= 300) {
          on = !on;
          lastToggle = millis();
          if (on) {
            led_color(r, g, b);
          } else {
            led_color(0, 0, 0);
          }
        }
      }
    }

    void led_hold(){
      led_color(255, 0, 255);
    }

    void mpu_config(){
      Wire.beginTransmission(0x68);
      Wire.write(0x6B);
      Wire.write(0x00); //sleep bit -> 0
      Wire.endTransmission();

      Wire.beginTransmission(0x68); //config gyro range
      Wire.write(0x1B);
      Wire.write(0x8);
      Wire.endTransmission();

      Wire.beginTransmission(0x68); //turn on LPF
      Wire.write(0x1A);
      Wire.write(0x05);
      Wire.endTransmission();

      Wire.beginTransmission(0x68); //config accel range
      Wire.write(0x1C);
      Wire.write(0x10);
      Wire.endTransmission();
    }

    void config_all() {
      mux_channel(PALM);
      delay(10);
      mpu_config();
      delay(10);
    }

    void mux_channel(uint8_t channel){
      Wire.beginTransmission(0x70);
      Wire.write(1 << channel);
      Wire.endTransmission();
    }

    void gyro_data(float *gX, float *gY, float *gZ){
      Wire.beginTransmission(0x68);
      Wire.write(0x43);
      Wire.endTransmission();

      Wire.requestFrom(0x68,6);

      int16_t rawX = Wire.read() << 8 | Wire.read();
      int16_t rawY = Wire.read() << 8 | Wire.read();
      int16_t rawZ = Wire.read() << 8 | Wire.read();

      //convert raw value to deg/s
      *gX = (float)rawX / 65.5;
      *gY = (float)rawY / 65.5;
      *gZ = (float)rawZ / 65.5;
    }

    void print_gyro(float gX, float gY, float gZ){
      Serial.print("== Viteza unghiulara pe X pentru este: ");
      Serial.print(gX);
      Serial.println(" ==");

      Serial.print("== Viteza unghiulara pe Y pentru este: ");
      Serial.print(gY);
      Serial.println(" ==");

      Serial.print("== Viteza unghiulara pe Z pentru este: ");
      Serial.print(gZ);
      Serial.println(" ==");
    }

    void get_gyro_offsets(float *offX, float *offY, float *offZ){
      float x = 0, y = 0, z = 0;
      float gX, gY, gZ;
      for(int i = 0 ; i < CAL_SAMPLES ; i++){
        gyro_data(&gX, &gY, &gZ);
        x += gX;
        y += gY;
        z += gZ;
      }

      *offX = (float) x / CAL_SAMPLES;
      *offY = (float) y / CAL_SAMPLES;
      *offZ = (float) z / CAL_SAMPLES; 
    }

    void calibrate_gyro(float *gX, float *gY, float *gZ, float offX, float offY, float offZ){
      *gX -= offX;
      *gY -= offY;
      *gZ -= offZ;
    }

    void accel_data(float *aX, float *aY, float *aZ){
      Wire.beginTransmission(0x68);
      Wire.write(0x3B);
      Wire.endTransmission();

      Wire.requestFrom(0x68,6);

      int16_t rawX = Wire.read() << 8 | Wire.read();
      int16_t rawY = Wire.read() << 8 | Wire.read();
      int16_t rawZ = Wire.read() << 8 | Wire.read();

      //convert raw value to deg/s
      *aX = (float)rawX / 4096;
      *aY = (float)rawY / 4096;
      *aZ = (float)rawZ / 4096;
    }

    void print_accel(float aX, float aY, float aZ){
      Serial.print("== Acceleratia pe X pentru este: ");
      Serial.print(aX);
      Serial.println(" ==");

      Serial.print("== Acceleratia pe Y pentru este: ");
      Serial.print(aY);
      Serial.println(" ==");

      Serial.print("== Acceleratia pe Z pentru este: ");
      Serial.print(aZ);
      Serial.println(" ==");
    }

    void compute_angles(){
      roll_palm = atan(aY_palm / sqrt(aX_palm * aX_palm + aZ_palm * aZ_palm)) / (PI / 180);
      pitch_palm = atan(aX_palm / sqrt(aY_palm * aY_palm + aZ_palm * aZ_palm)) / (PI / 180);
    }

    void get_flex_values(){
      int thumb_val = analogRead(THUMB);
      int index_val = analogRead(INDEX);
      int little_val = analogRead(LITTLE);

      thumb_min = min(thumb_min, thumb_val);
      thumb_max = max(thumb_max, thumb_val);

      index_min = min(index_min, index_val);
      index_max = max(index_max, index_val);

      little_min = min(little_min, little_val);
      little_max = max(little_max, little_val);
    }

    void save_flex_range(){
      prefs.begin("flexRange", false);

      prefs.putInt("thumb_min", thumb_min);
      prefs.putInt("thumb_max", thumb_max);

      prefs.putInt("index_min", index_min);
      prefs.putInt("index_max", index_max);

      prefs.putInt("little_min", little_min);
      prefs.putInt("little_max", little_max);

      prefs.putBool("flexRoM", true);
      prefs.end();

      Serial.println("RoM of flex sensors saved.");
    }

    void load_flex_range(){
      prefs.begin("flexRange", true);
      thumb_min = prefs.getInt("thumb_min", 4095);
      thumb_max = prefs.getInt("thumb_max", 0);
      index_min = prefs.getInt("index_min", 4095);
      index_max = prefs.getInt("index_max", 0);
      little_min = prefs.getInt("little_min", 4095);
      little_max = prefs.getInt("little_max", 0);
      prefs.end();
    }

    void load_gyro_offsets() {
      prefs.begin("gyroCal", true);
      offX_palm = prefs.getFloat("pX", 0.0f);
      offY_palm = prefs.getFloat("pY", 0.0f);
      offZ_palm = prefs.getFloat("pZ", 0.0f);
      prefs.end();
    }

    void calibrate_flex_rom(){
        Serial.println("Calibrare flex RoM");
        led_wait();

        int start = millis();

        led_hold();
        Serial.println("Miscati degetele pentru RoM a senzorilor de rezistenta");
        led_hold();

        while(millis() - start < FLEX_REFERENCE_TIME){
          get_flex_values();
        }

        save_flex_range();
    }

    void print_flex_rom(){
        Serial.println("== RoM senzori de rezistenta salvat ==");

        Serial.print("== Thumb Sensor RoM: Min = ");
        Serial.print(thumb_min);
        Serial.print(", Max = ");
        Serial.print(thumb_max);
        Serial.println(" ==");

        Serial.print("== Index Sensor RoM: Min = ");
        Serial.print(index_min);
        Serial.print(", Max = ");
        Serial.print(index_max);
        Serial.println(" ==");

        Serial.print("== Little Sensor RoM: Min = ");
        Serial.print(little_min);
        Serial.print(", Max = ");
        Serial.print(little_max);
        Serial.println(" ==");

        Serial.println("== Reporniti alimentarea ==");
    }

    void save_gyro_offsets(){
      prefs.begin("gyroCal", false);
      prefs.putFloat("pX", offX_palm);
      prefs.putFloat("pY", offY_palm);
      prefs.putFloat("pZ", offZ_palm);
      prefs.putBool("calibrated", true);
      prefs.end();
    }

    void capture_gyro_offsets(){
        Serial.println("== Calibrare flex RoM ==");

        led_wait();
        led_hold();

        Serial.println("== Calibrare senzor palma ==");
        mux_channel(PALM);
        delay(10);
        mpu_config();
        delay(10);
        get_gyro_offsets(&offX_palm, &offY_palm, &offZ_palm);
        save_gyro_offsets();
    }

    void print_offset(float oX, float oY, float oZ){
      Serial.print("== Offset pe X pentru este: ");
      Serial.print(oX);
      Serial.println(" ==");

      Serial.print("== Offset pe Y pentru este: ");
      Serial.print(oY);
      Serial.println(" ==");

      Serial.print("== Offset pe Z pentru este: ");
      Serial.print(oZ);
      Serial.println(" ==");
    }

    void print_gyro_offsets(){
      print_offset(offX_palm, offY_palm, offZ_palm);
    }

    void save_neutral_pose(){
      prefs.begin("neutralPose", false);
      prefs.putFloat("nPitch", pitch_neutral);
      prefs.putFloat("nRoll", roll_neutral);
      prefs.putBool("neutral_pose", true);
      prefs.end();
    }

    void load_neutral_pose(){
      prefs.begin("neutralPose", true);
      roll_neutral = prefs.getFloat("nRoll", 0.0f);
      pitch_neutral = prefs.getFloat("nPitch", 0.0f);
      prefs.end();
    }

    void print_neutral_pose() {
      Serial.println(F("=== Neutral Pose Reference ==="));

      Serial.printf("== Palm Neutral Angles [Roll, Pitch] : [%.3f, %.3f] ==\n", roll_neutral, pitch_neutral);
    }

    void capture_neutral_pose(){
        Serial.println("Obtinere date de referinta pt pozitie neutra");

        led_wait();

        int start = millis();
        int samples = 0;
        float sumRoll = 0, sumPitch = 0;

        led_hold();

        while(millis() - start < NEUTRAL_REFERENCE_TIME){
          mux_channel(PALM);
          gyro_data(&gX_palm, &gY_palm, &gZ_palm);
          accel_data(&aX_palm, &aY_palm, &aZ_palm);
          compute_angles();

          sumRoll += roll_palm;
          sumPitch += pitch_palm;

          samples++;

          delay(10);
        }

        roll_neutral = sumRoll / samples;
        pitch_neutral = sumPitch / samples;

        save_neutral_pose();
    }

    void capture_palm_pitch_ref(){
      Serial.println("== Capture Palm Pitch Reference ==");
      led_wait();
      led_hold();

      long start = millis();

      while( millis() - start < PALM_ROM_REFERENCE_TIME){
        mux_channel(PALM); //PALM_ROM_REFERENCE_TIME
        gyro_data(&gX_palm, &gY_palm, &gZ_palm);
        accel_data(&aX_palm, &aY_palm, &aZ_palm);

        compute_angles();

        if (pitch_palm < pitch_min) {
          pitch_min = pitch_palm;
        }
        if (pitch_palm > pitch_max) {
          pitch_max = pitch_palm;
        }

        delay(INTERVAL);
      } 

      Serial.printf("== Pitch  min: %.1f° ==\n", pitch_min);
      Serial.printf("== Pitch  max: %.1f° ==\n", pitch_max);

      prefs.begin("palmPitchRef", false);
      prefs.putFloat("minPitch", pitch_min);
      prefs.putFloat("maxPitch", pitch_max);
      prefs.putBool ("captured", true);
      prefs.end();
    }

    void capture_palm_roll_ref(){
      Serial.println("== Capture Palm Roll Reference ==");
      led_wait();
      led_hold();

      long start = millis();

      while( millis() - start < PALM_ROM_REFERENCE_TIME){
        mux_channel(PALM); //PALM_ROM_REFERENCE_TIME
        gyro_data(&gX_palm, &gY_palm, &gZ_palm);
        accel_data(&aX_palm, &aY_palm, &aZ_palm);

        compute_angles();

        if (roll_palm < roll_min) {
          roll_min = roll_palm;
        }
        if (roll_palm > roll_max) {
          roll_max = roll_palm;
        }

        delay(INTERVAL);
      } 

      Serial.printf("== Roll min: %.1f° ==\n", roll_min);
      Serial.printf("== Roll max: %.1f° ==\n", roll_max);

      prefs.begin("palmRollRef", false);
      prefs.putFloat("minRoll", roll_min);
      prefs.putFloat("maxRoll", roll_max);
      prefs.putBool ("captured", true);
      prefs.end();
    }


    void determine_gesture(){
      //check for pronatio/supination
      // => flexion/extension is in neutral
      bool pitch_neutral_zone = (pitch_palm >= pitch_lo_dead) && (pitch_palm <= pitch_hi_dead);
      bool roll_neutral_zone = (roll_palm >= roll_lo_dead) && (roll_palm <= roll_hi_dead);

      if (!pitch_neutral_zone && roll_neutral_zone){
        if(pitch_palm > pitch_hi_dead){
          //Serial.println("EXTENSION");
          gesture_name = "EXTENSION";
          led_color(255,0,0);
          palm_r = 255;
          palm_g = 0;
          palm_b = 0;
        } else {
          //Serial.println("FLEXION");
          gesture_name = "FLEXION";
          led_color(0,0,255);
          palm_r = 0;
          palm_g = 0;
          palm_b = 255;
        }
      } else if (!roll_neutral_zone && pitch_neutral_zone){
        if(roll_palm > roll_hi_dead){
          //Serial.println("SUPINATION");
          gesture_name = "SUPINATION";
          led_color(0,255,0);
          palm_r = 0;
          palm_g = 255;
          palm_b = 0;
        } else {
          //Serial.println("PRONATION");
          gesture_name = "PRONATION";
          led_color(255,0,255);
          palm_r = 255;
          palm_g = 0;
          palm_b = 255;
        }
      } else if (roll_neutral_zone && pitch_neutral_zone){
        //Serial.println("NEUTRAL");
        gesture_name = "NEUTRAL";
        led_color(255,255,255);
        palm_r = 255;
        palm_g = 255;
        palm_b = 255;
      } else {
        //Serial.println("UNKNOWN");
        gesture_name = "UNKNOWN";
        led_color(0,255,255);
        palm_r = 255;
        palm_g = 255;
        palm_b = 255;
      }
    }

    void load_palm_reference(){
      prefs.begin("palmRollRef", true);
      roll_min = prefs.getFloat("minRoll", -180);
      roll_max = prefs.getFloat("maxRoll", 180);
      prefs.end();

      prefs.begin("palmPitchRef", true);
      pitch_min = prefs.getFloat("minPitch", -180);
      pitch_max = prefs.getFloat("maxPitch", 180);
      prefs.end();
    }

  /////////////////////////////// NEW ///////////////////////////////
  void processClicks(int idxClick, int litClick) {
    unsigned long now = millis();

    // LAND has top priority
    if (litClick == 2 && crt_mode != MODE_LAND) {
      crt_mode = MODE_LAND;
      blink_on = false; 
      blink_last_toggle = now;
      lit_cooldown_until = now + COOLDOWN_MS;
      lit_require_release = true;
      return;
    }

    // HOVER toggle (single index-click)
    if (idxClick == 1 && 
      (crt_mode == MODE_HOVER || crt_mode == MODE_NORMAL && crt_mode != MODE_LAND) &&
      !exiting_pause
    ) {
      crt_mode = (crt_mode == MODE_HOVER ? MODE_NORMAL : MODE_HOVER);  
      blink_on = false;
      blink_last_toggle = now;
      idx_cooldown_until = now + COOLDOWN_MS;
      idx_require_release = true;
      return;
    }

    // PAUSE toggle (double index-click)
    if (idxClick == 2 && crt_mode != MODE_LAND) {
      if (crt_mode != MODE_PAUSE) {
      // entering pause
      crt_mode = MODE_PAUSE;
      pause_gesture = gesture_name;
      pause_r = palm_r; pause_g = palm_g; pause_b = palm_b;
      } else {
        // exiting pause → go into hover for 0.5s
        crt_mode = MODE_HOVER;
        pause_exit_deadline = now + PAUSE_EXIT_HOVER;
        exiting_pause = true;  
      }
      blink_on = false;
      blink_last_toggle = now;
      idx_cooldown_until = now + COOLDOWN_MS;
      idx_require_release = true;
      return;
    }
  }

  /////////////////////////////// NEW ///////////////////////////////
  void renderLEDs() {
    unsigned long now = millis();

    auto toggleBlink = [&]() {
      if (now - blink_last_toggle >= BLINK_INTERVAL) {
        blink_on = !blink_on;
        blink_last_toggle = now;
      }
    };

    switch (crt_mode) {
      case MODE_LAND:
        toggleBlink();
        led_color(blink_on ? 255 : 0, 0, 0);
        break;

      case MODE_PAUSE:
        toggleBlink();
        // blink between stored pause color ↔ magenta (255,0,255)
        led_color(blink_on ? pause_r : 255,
                blink_on ? pause_g :   0,
                blink_on ? pause_b : 255);
        break;

      case MODE_HOVER:
        // did we just exit pause and are still in the hover‐timeout window?
        if (millis() < pause_exit_deadline) {
          toggleBlink();
          // blink white ↔ current palm gesture
          led_color(blink_on ? 255 : palm_r,
                  blink_on ? 255 : palm_g,
                  blink_on ? 255 : palm_b);
        } else {
          toggleBlink();
          led_color(blink_on ? 255 : 0,
                  blink_on ? 255 : 0,
                  blink_on ? 255 : 0);
        }
        break;

      case MODE_NORMAL:
      default:
        // steady palm gesture color
        led_color(palm_r, palm_g, palm_b);
        break;
    }
  }

//count clicks per finger
  int indexClickNo() {
    unsigned long now = millis();

    if (exiting_pause) return 0;
    // 1) Cooldown: clear any pending & ignore
    if (now < idx_cooldown_until) {
      idx_prev_pressed   = false;
      idx_single_pending = 0;
      return 0;
    }
    int v = analogRead(INDEX);
    // 2) Require full release after pause
    if (idx_require_release) {
      if (v > index_threshold) return 0;
      idx_require_release = false;
    }
    int result = 0;
    // 3) Detect press & double‐press
    if (v > index_threshold && !idx_prev_pressed) {
      if (now - idx_last_press_time < DOUBLE_CLICK_WINDOW) {
        result = 2;
      } else {
        idx_single_pending = 1;
      }
      idx_last_press_time = now;
      idx_prev_pressed    = true;
    } else if (v <= index_threshold) {
      idx_prev_pressed = false;
    }
    // 4) Single‐click timeout
    if (idx_single_pending && now - idx_last_press_time > DOUBLE_CLICK_WINDOW) {
      result = 1;
      idx_single_pending = 0;
    }
    return result;
  }

  // Returns 0 = no click, 2 = double‐click on LITTLE (single clicks ignored)
  int littleClickNo() {
    unsigned long now = millis();
    // 1) Cooldown
    if (now < lit_cooldown_until) {
      lit_prev_pressed = false;
      return 0;
    }
    int v = analogRead(LITTLE);
    // 2) Require full release after pause
    if (lit_require_release) {
      if (v > little_threshold) return 0;
      lit_require_release = false;
    }
    int result = 0;
    // 3) Detect only double‐press
    if (v > little_threshold && !lit_prev_pressed) {
      if (now - lit_last_press_time < DOUBLE_CLICK_WINDOW) {
      result = 2;
      }
      lit_last_press_time = now;
      lit_prev_pressed    = true;
    } else if (v <= little_threshold) {
      lit_prev_pressed = false;
    }
    return result;
  }

  void setup() {
    // put your setup code here, to run once:

    Serial.begin(115200);

    pinMode(RED, OUTPUT);
    pinMode(GREEN, OUTPUT);
    pinMode(BLUE, OUTPUT);

    Wire.setClock(400000);
    Wire.begin();
    delay(250);

    prefs.begin("gyroCal", true);
    bool isCalibrated = prefs.getBool("calibrated", false);
    prefs.end();
    if(!isCalibrated){
      capture_gyro_offsets();
      print_gyro_offsets();
      while(true){}
    }

    // ===== FLEX SENSORS ROM CAPTURE =====
    prefs.begin("flexRange", true);
    bool flexRoM = prefs.getBool("flexRoM", false);
    prefs.end();
    if(!flexRoM){
      calibrate_flex_rom();
      print_flex_rom();
      while(true){};
    }

    // ===== NEUTRAL POSE REFERENCEE CAPTURE =====
    prefs.begin("neutralPose", true);
    bool neutral_pose = prefs.getBool("neutral_pose", false);
    prefs.end();
    if(!neutral_pose){
      capture_neutral_pose();
      print_neutral_pose();
      while(true){};
    }

    // ===== PALM MIN/MAX ROLL/PITCH
    // palm pitch RoM
    prefs.begin("palmPitchRef", true);
    if (!prefs.getBool("captured", false)) {
      capture_palm_pitch_ref();
      while(true){};
    }
    prefs.end();

    // palm roll RoM
    prefs.begin("palmRollRef", true);
    if (!prefs.getBool("captured", false)) {
      capture_palm_roll_ref();
      while(true){};
    }
    prefs.end();

    // ===== CONFIGURE IMU SENSORS =====
    config_all();

    Serial.println("== Calibrare... ==");

    // ===== LOAD REFERENCE DATA FROM FLASH =====
    load_gyro_offsets();
    load_flex_range();
    load_neutral_pose();
    load_palm_reference();
    Serial.println(">>> Senzori calibrati & referinte incarcate <<<");

    pitch_lo_dead  = (pitch_neutral + pitch_min) / 2.0f;
    pitch_hi_dead  = (pitch_neutral + pitch_max) / 2.0f;
    roll_lo_dead   = (roll_neutral  + roll_min)  / 2.0f;
    roll_hi_dead   = (roll_neutral  + roll_max)  / 2.0f;

    index_threshold  = (index_min  + index_max ) / 2;
    little_threshold = (little_min + little_max) / 2;

    //prevent starting in land or hover mode
    idx_prev_pressed    = false;
    idx_single_pending  = 0;
    idx_cooldown_until  = 0;
    idx_require_release = true;

    lit_prev_pressed    = false;
    lit_cooldown_until  = 0;
    // assuming "released" finger position - straight, so no chance of catching unwanted clicks
    lit_require_release = true;

    crt_mode = MODE_NORMAL;
  }

  void loop() {
    // put your main code here, to run repeatedly:

    mux_channel(PALM);
    delay(5);
    gyro_data(&gX_palm, &gY_palm, &gZ_palm);
    accel_data(&aX_palm, &aY_palm, &aZ_palm);
    calibrate_gyro(&gX_palm, &gY_palm, &gZ_palm, offX_palm, offY_palm, offZ_palm);
    compute_angles();

    if (crt_mode != MODE_PAUSE) {
      determine_gesture();
    }

    if (exiting_pause && millis() >= pause_exit_deadline) {
      crt_mode = MODE_NORMAL;
      exiting_pause = false;
    }

    int idxClick = indexClickNo(); // 0/1/2
    int litClick = littleClickNo(); // 0/2

    processClicks(idxClick, litClick);  

    //[land, takeoff, hover, fwd, back, left, right]
    bool cmd[7] = {0};

    switch (crt_mode) {
    case MODE_PAUSE:
      if (pause_gesture == "FLEXION")    cmd[3] = true;
      else if (pause_gesture == "EXTENSION") cmd[4] = true;
      else if (pause_gesture == "PRONATION") cmd[5] = true;
      else if (pause_gesture == "SUPINATION") cmd[6] = true;
      Serial.print("PAUSE - ");
      Serial.println(pause_gesture);
      break;

    case MODE_HOVER:
      cmd[2] = true;
      Serial.println("HOVER");
      break;

    case MODE_LAND:
      cmd[0] = true;
      Serial.println("LAND");
      break;

    case MODE_NORMAL:
    default:
      if (gesture_name == "FLEXION")    cmd[3] = true;
      else if (gesture_name == "EXTENSION") cmd[4] = true;
      else if (gesture_name == "PRONATION") cmd[5] = true;
      else if (gesture_name == "SUPINATION") cmd[6] = true;
      Serial.println(gesture_name);
      break;
    }
      
    Serial.print("[");
    for (int i = 0; i < 7; i++) {
      Serial.print(cmd[i] ? "1" : "0");
      if (i < 6) Serial.print(";");
    }
    Serial.println("]");

    renderLEDs();

    if( millis() - last_print >= 1000){
      print_gyro(gX_palm, gY_palm, gZ_palm);
      print_accel(aX_palm, aY_palm, aZ_palm);
      Serial.printf("Unghiurile de roratie sunt: [ %.3f , %.3f ]", roll_palm, pitch_palm);

      last_print = millis();  
    }
  }