
const int DBG_ENABLE = 1;

const int SET_MODE = 4;
// 0: Enter normal number of 32us pulses          - Send small pulses
// 1: Enter time in us and send one pulse         - Send one long pule
// 2: Enter start lenght and end lenght of pulse  - Setup random
// 3: Enter start lenght                          - Setup random (Better and faster)
// 4: Enter PWM


// LED Strip Modes
// 1 : All on
// 2 : OFF
// 3 : Red
// 4 : Green
// 5 : Yellow
// 6 : Blue
// 7 : Purple
// 8 : Light blue


// PINS
const byte ALIVE_PIN = LED_BUILTIN;
const byte SENSOR_PIN = A0;
const byte PULSE_PIN = 2;

// Data
//const int SAMPLE_TICKS = 64;
const int SAMPLE_TICKS = 32;
unsigned int data[SAMPLE_TICKS];
unsigned int data_in;

// Time
unsigned long t_start;
unsigned long t_stop;
unsigned long t_end;
unsigned long t_diff;
unsigned long t_data[SAMPLE_TICKS];

// State
int state_sensor;
int state_next;




void send_pulse(int n) {

  if (n == 0) {
    return;
  }
  
  unsigned long time_start;
  unsigned long time_end;
  unsigned long time_diff;
  //const unsigned long pw = 32;
  const unsigned long pw = 5;

  // Send pulses of 32us each
  int i = 0;
  time_start = micros();
  while (true) {
    // Send low pulse
    digitalWrite(PULSE_PIN, LOW); 
    // Wait 32 us
    while (true) {
      time_end = micros();
      time_diff = time_end - time_start;
      if (time_diff > pw) {
        time_start = time_end;
        break;
      }
    }
    // Send high pulse
    digitalWrite(PULSE_PIN, HIGH); 
    // Wait 32 us
    while (true) {
      time_end = micros();
      time_diff = time_end - time_start;
      if (time_diff > pw) {
        time_start = time_end;
        break;
      }
    }
    // Check if time to exit
    i = i + 1;
    if (i == n) {
      // Done
      break;
    }
  }
  return;
}


// the setup function runs once when you press reset or power the board
void setup() {

  // Indicate alive
  pinMode(ALIVE_PIN, OUTPUT);
  digitalWrite(ALIVE_PIN, LOW); 

  // Select Analog pin
  pinMode(SENSOR_PIN, INPUT);

  // Pulse pin
  pinMode(PULSE_PIN, OUTPUT);
  digitalWrite(PULSE_PIN, LOW); 

  
  // Setup serial
  if (DBG_ENABLE == 1) {
    Serial.begin(9600);
  }
  
  // Wait some time
  delay(1000);

  // Set pin levels
  digitalWrite(ALIVE_PIN, HIGH); 
  digitalWrite(PULSE_PIN, HIGH); 

  // Setup done
  Serial.println("---- Setup complete! ----");

  // Detect state after reset 
  Serial.println("---- State after reset ----");
  Serial.print("State analogRead: ");
  data_in = analogRead(SENSOR_PIN);
  Serial.println(data_in);
  
   
}

//unsigned long t_start;
//unsigned long t_stop;
//unsigned long t_diff;
//int data_in;
//int data_max;
//int data[SAMPLE_TICKS];
//long data_max_a;
//long data_max_w;

int sum_h;
int sum_l;

unsigned long num_pulses;
unsigned long time_int;
unsigned long time_int2;
unsigned long time_step;


// the loop function runs over and over again forever
void loop() {


  if (SET_MODE==4) {

    // Wait some time
    delay(100);
  
    //Prompt User for Input
    Serial.println("Enter configuration form PWM");
    Serial.print("Start: ");
    time_int = 0;
    while (time_int == 0) {
      while (Serial.available() == 0) {
      }
      time_int = Serial.parseInt(); 
      while (Serial.available() == 1) {
        Serial.parseInt(); // Empty
      }
    }
    Serial.println("");
    Serial.print("Start is pulses: ");
    int start_pulses = time_int/100%10;
    Serial.println(start_pulses);
    Serial.print("PWM step: ");
    int pwm_step = time_int/10%10;
    Serial.print(pwm_step);
    Serial.print("/");
    int pwm_step_neg = 8-pwm_step;
    Serial.println(pwm_step_neg);
    Serial.print("PWM color: ");
    unsigned long pwm_color = time_int%10;
    Serial.println(pwm_color);
    Serial.println("-");
    unsigned long pwm_period = 10000;
    unsigned long pwm_change = pwm_color * (pwm_period/4);
    Serial.print("PWM period: ");
    Serial.println(pwm_period);
    Serial.print("PWM change: ");
    Serial.println(pwm_change);
    Serial.println("");

    // Send start pulses
    send_pulse(start_pulses);

    // Send PWM

    while (true) {
      // Sample start time
      t_start = micros();
      // Set other color
      send_pulse(pwm_step);
      // Wait 1.
      while (true) {
        t_end = micros();
        t_diff = t_end - t_start;
        if (t_diff > pwm_change) {
          break;
        }
      }
      // Set start color
      send_pulse(pwm_step_neg);
      // Exit of incoming input from serial
      if (Serial.available() == 1) {
        Serial.println("Stopped.");
        break;
      }
      // Wait 2.
      while (true) {
        t_end = micros();
        t_diff = t_end - t_start;
        if (t_diff > pwm_period) {
          break;
        }
      }
      // Exit of incoming input from serial
      if (Serial.available() == 1) {
        Serial.println("Stopped.");
        break;
      }
    }
   
    Serial.println("Done!");
  }


  if (SET_MODE==3) {

    // Good Mix
    // 100000

    // Wait some time
    delay(100);
  
    //Prompt User for Input
    Serial.println("Enter start and stop in us fir sweep");
    Serial.print("Start: ");
    time_int = 0;
    while (time_int == 0) {
      while (Serial.available() == 0) {
      }
      time_int = Serial.parseInt(); 
      while (Serial.available() == 1) {
        Serial.parseInt(); // Empty
      }
    }
    Serial.println("");
    Serial.print("Start is ");
    Serial.print(time_int);
    Serial.println("");

    for (unsigned long i = time_int; i > 50; i = i - i/10) {
      // Wait some time
      delay(1);
      // Wait long pulse
      t_start = micros();
      // Send low pulse
      digitalWrite(PULSE_PIN, LOW); 
      // Wait 32 us
      while (true) {
        t_end = micros();
        t_diff = t_end - t_start;
        if (t_diff > i) {
          break;
        }
      }
      // Send high pulse
      digitalWrite(PULSE_PIN, HIGH); 
    }

    Serial.println("Done!");
  }

  
  if (SET_MODE==2) {

    // Good Mix
    // 80000
    // 5000
    // 1000

    // Wait some time
    delay(100);
  
    //Prompt User for Input
    Serial.println("Enter start and stop in us fir sweep");
    Serial.print("Start: ");
    time_int = 0;
    while (time_int == 0) {
      while (Serial.available() == 0) {
      }
      time_int = Serial.parseInt(); 
      while (Serial.available() == 1) {
        Serial.parseInt(); // Empty
      }
    }
    Serial.println("");
    Serial.print("Start is ");
    Serial.print(time_int);
    Serial.println("");

    //Prompt User for Input
    Serial.print("Stop: ");
    time_int2 = 0;
    while (time_int2 == 0) {
      while (Serial.available() == 0) {
      }
      time_int2 = Serial.parseInt(); 
      while (Serial.available() == 1) {
        Serial.parseInt(); // Empty
      }
    }
    Serial.println("");
    Serial.print("end is ");
    Serial.print(time_int2);
    Serial.println("");

    //Prompt User for Input
    Serial.print("Step: ");
    time_step = 0;
    while (time_step == 0) {
      while (Serial.available() == 0) {
      }
      time_step = Serial.parseInt(); 
      while (Serial.available() == 1) {
        Serial.parseInt(); // Empty
      }
    }
    Serial.println("");
    Serial.print("Step is ");
    Serial.print(time_step);
    Serial.println("");

    for (unsigned long i = time_int; i > time_int2; i = i - time_step) {
      // Wait some time
      delay(1);
      // Wait long pulse
      t_start = micros();
      // Send low pulse
      digitalWrite(PULSE_PIN, LOW); 
      // Wait 32 us
      while (true) {
        t_end = micros();
        t_diff = t_end - t_start;
        if (t_diff > i) {
          break;
        }
      }
      // Send high pulse
      digitalWrite(PULSE_PIN, HIGH); 
    }

    Serial.println("Done!");
  }


  if (SET_MODE==1) {

    // Wait some time
    delay(100);
  
    //Prompt User for Input
    Serial.print("Enter time in ms for long pulse: ");
    
    time_int = 0;
    while (time_int == 0) {
      while (Serial.available() == 0) {
      }
      time_int = Serial.parseInt(); 
      while (Serial.available() == 1) {
        Serial.parseInt(); // Empty
      }
    }
    Serial.println("");
    Serial.print("Sending ");
    Serial.print(time_int);
    Serial.print("ms pulse...");
    Serial.println("");

    // Wait long pulse
    t_start = micros();
    // Send low pulse
    digitalWrite(PULSE_PIN, LOW); 
    // Wait 32 us
    while (true) {
      t_end = micros();
      t_diff = t_end - t_start;
      if (t_diff > time_int) {
        t_start = t_end;
        break;
      }
    }
    // Send high pulse
    digitalWrite(PULSE_PIN, HIGH); 

    Serial.println("Done!");
  }

  if (SET_MODE==0) {
    // Wait some time
    delay(100);
  
    //Prompt User for Input
    Serial.print("Enter number of pulses: ");
    
    num_pulses = 0;
    while (num_pulses == 0) {
      while (Serial.available() == 0) {
      }
      num_pulses = Serial.parseInt(); 
      while (Serial.available() == 1) {
        Serial.parseInt(); // Empty
      }
    }
    Serial.println("");
    Serial.print("Sending ");
    Serial.print(num_pulses);
    Serial.print(" pulses...");
    Serial.println("");
  
    // Send pulses of 32us each
    int i = 0;
    t_start = micros();
    while (true) {
  
      // Send low pulse
      digitalWrite(PULSE_PIN, LOW); 
  
      // Wait 32 us
      while (true) {
        t_end = micros();
        t_diff = t_end - t_start;
        if (t_diff > 32) {
          t_start = t_end;
          break;
        }
      }
  
      // Send high pulse
      digitalWrite(PULSE_PIN, HIGH); 
  
      // Check if time to exit
      i = i + 1;
      if (i == num_pulses) {
        Serial.println("Done!");
        break;
      }
  
      // Wait 32 us
      while (true) {
        t_end = micros();
        t_diff = t_end - t_start;
        if (t_diff > 32) {
          t_start = t_end;
          break;
        }
      }
    }
  }
  

//  if (true) {
//
//    for (int i = 0; i < SAMPLE_TICKS-1; i++) {
//      data[i] = 0;
//    }
//    
//    Serial.println("---- Armed and Ready! ----");
//    
//    state_sensor = digitalRead(SENSOR_PIN); 
//    // Set next state
//    if (state_sensor == 0) {
//      state_next = 1;
//    } else {
//      state_next = 0;
//    }
//    // Wait for state changes
//    while (state_sensor != state_next) {
//      state_sensor = digitalRead(SENSOR_PIN);
//    }
//    
//    int pulses = 0;
//    if (state_sensor == 0) {
//      pulses = 1;
//    } 
//    
//    t_start = micros();
//    int i = 0;
//    while (true) {
//      // Set next state
//      if (state_sensor == 0) {
//        state_next = 1;
//      } else {
//        state_next = 0;
//      }
//      // Wait for state changes
//      int break_again = 0;
//      while (state_sensor != state_next) {
//
//        // Break?
//        t_end = micros();
//        t_diff = t_end - t_start;
//        if (t_diff > 20000) {
//          i = i + 1;
//          data[i] = 32768;
//          Serial.println("---- Timed Out! ----");
//          break_again = 1;
//          break;
//        }
//      
//        state_sensor = digitalRead(SENSOR_PIN);
//      }
//      // Break after TO
//      if (break_again==1) {
//        break;
//      }
//
//      if (state_sensor == 0) {
//        pulses = pulses + 1;
//        data[i] = pulses;
//      } 
//      // Read time diff
//      t_end = micros();
//      t_diff = t_end - t_start;
//      //t_data[i] = t_diff;
//      t_start = t_end;
//
//      // Check if long time
//      if (t_diff > 500) {
//        i = i + 1;
//        pulses = 0;
//        data[i] = t_diff;
//        i = i + 1;
//      }
//      
//      // Read state
//      //data[i] = state_sensor;
//      // Check if overflow
//      if (i == SAMPLE_TICKS) {
//        Serial.println("---- End reached! ----");
//        break;
//      }
//    }
//
//    Serial.println("---- Done! ----");
//  
//    delay(1000); 
//  
//    for (int i = 0; i < SAMPLE_TICKS-1; i++) {
//      Serial.print("i: ");
//      Serial.print(i);
//      Serial.print(", state: ");
//      Serial.print(data[i]);
//      Serial.println("");
//      data[i] = 0;
//    }
//  
//  }

  
//   for (int i = 0; i < SAMPLE_TICKS; i++) {
//      data_in = analogRead(SENSOR_PIN);
//      data[i] = data_in;
//      if (data_in > data_max) {
//        data_max = data_in;
//      }
//    }




      
//  delay(1000); 
//  digitalWrite(ALIVE_PIN, LOW); 
//  delay(1000); 
//  digitalWrite(ALIVE_PIN, HIGH); 


//    // Start Measurement
//    t_start = millis();
//    // Collect data
//    data_max = 0;
//    for (int i = 0; i < SAMPLE_TICKS; i++) {
//      data_in = analogRead(SENSOR_PIN);
//      data[i] = data_in;
//      if (data_in > data_max) {
//        data_max = data_in;
//      }
//    }
//    // End measurement
//    t_stop = millis();
//    // Calc mili ampere
//    data_max_a = data_max * TRIG_GAIN;
//    data_max_w = (data_max_a * 23) / 100;
//    // Enable relay
//    if (digitalRead(SW_PIN_PULL)) {
//      if (TRIG_LEVEL_MA < data_max_a) {
//        // ON
//        digitalWrite(RELAY_PIN, HIGH); 
//        digitalWrite(LED_ON_HIGH, HIGH);
//        digitalWrite(LED_OFF_HIGH, LOW);
//        hyst_cnt = HYST_SEC_ON * 5;
//      } else {
//        if (hyst_cnt == 0) {
//          // OFF
//          digitalWrite(RELAY_PIN, LOW);
//          digitalWrite(LED_ON_HIGH, LOW);
//          digitalWrite(LED_OFF_HIGH, HIGH);
//        } else {
//          // ON
//          digitalWrite(RELAY_PIN, HIGH);
//          digitalWrite(LED_ON_HIGH, HIGH);
//          //digitalWrite(LED_OFF_HIGH, LOW);
//          digitalWrite(LED_OFF_HIGH, HIGH);
//          hyst_cnt = hyst_cnt - 1;  
//        }
//      }
//    } else {
//      digitalWrite(RELAY_PIN, HIGH); 
//      digitalWrite(LED_ON_HIGH, HIGH);
//      digitalWrite(LED_OFF_HIGH, LOW);
//    }
//    // Print
//    t_diff = t_stop - t_start;
//    if (DBG_ENABLE == 1) {
//      Serial.println("--");
//      Serial.print("Time: ");
//      Serial.println(t_diff);
//      Serial.print("Max: ");
//      Serial.println(data_max);
//      Serial.print("Amp: ");
//      Serial.print(data_max_a);
//      Serial.println("mA");
//      Serial.print("Watt: ");
//      Serial.print(data_max_w);
//      Serial.println("w");
//    }
  
}
