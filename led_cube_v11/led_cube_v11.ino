
///////////////////////////////////////////////////
// CONFIGURATION
///////////////////////////////////////////////////

// Enable print over serial
const int DBG_ENABLE = 0;

// Mode Setting
int set_mode = 5;
// 0: Control Color Intensity           - Z-axis controls intensity
// 1: Control Fade Two Colors           - Z-axis controls fade between two colors
// 2: Control Color Angle Set           - Z-axis controls angle on color circle
// 3: Control Color Angle Fade          - Z-axis controls fade speed of angle on color circle
// 4: Random Colors                     - Z-axis control color change speed
// 5: Color Angle and Intensity         - Z-axis control intensity, XY-axis control color angle
// 6: Control White Intensity and Blink - Z-axis control intensity, XY-axis control blink rate  
// 7: Control Angle Fade and Intensity  - Z-axis controls fade speed of angle on color circle and XY-axis controls intensity

// Operational Mode
const int OPERATIONAL_EN = 1;
// List of operationel modes
const int OPERATIONAL_MODE[]  = {6,5,7,4,99};
const int OPERATIONAL_COLOR[] = {2,3,5,6,99};
// Start operational mode index
int opt_mode_index = 0;

// LED Strip Modes
// 1 : All on
// 2 : OFF
// 3 : Red
// 4 : Green
// 5 : Yellow
// 6 : Blue
// 7 : Purple
// 8 : Light blue
const int COLOR_ALL    = 0;
const int COLOR_OFF    = 1;
const int COLOR_RED    = 2;
const int COLOR_GREEN  = 3;
const int COLOR_YELLOW = 4;
const int COLOR_BLUE   = 5;
const int COLOR_PURPLE = 6;
const int COLOR_L_BLUE = 7;
const int COLOR_IGNORE = 127;


///////////////////////////////////////////////////
// PIN CONFIGURATION
///////////////////////////////////////////////////

//const int PULSE_PIN = 7; // DEV Board
const int PULSE_PIN = 5; // Cube


///////////////////////////////////////////////////
// Gyro and Accelometer
///////////////////////////////////////////////////

#include <Wire.h>
#include <MPU6050.h>
MPU6050 mpu;

int gx_min = 0;
int gx_max = 0;
int gy_min = 0;
int gy_max = 0;
int gz_min = 0;
int gz_max = 0;
int ax_min = 0;
int ax_max = 0;
int ay_min = 0;
int ay_max = 0;
int az_min = 0;
int az_max = 0;

///////////////////////////////////////////////////
// PWM Coding
///////////////////////////////////////////////////

const unsigned long PWM_PERIOD = 10000;

const unsigned long intensity_div = 10;
const int           intensity_mul = 5;
unsigned long       intensity_real;
unsigned long       intensity;

const unsigned long color_angle_div = 10;
const int           color_angle_mul = 5;
unsigned long       color_angle_real;
unsigned long       color_angle;

const unsigned long rate_div = 10;
const int           rate_mul = 5;
unsigned long       rate_real;
unsigned long       rate;


///////////////////////////////////////////////////
// Control
///////////////////////////////////////////////////

int color_curr = COLOR_ALL;
unsigned long time_action1_next;
unsigned long time_action2_next;
unsigned long time_action3_next;

const int MODE_RUNNING     = 0;
const int MODE_WAIT_CHANGE = 1;
int change_mode_state = MODE_RUNNING;


///////////////////////////////////////////////////
// Auto Reste
///////////////////////////////////////////////////

#include <EEPROM.h>
void(* resetFunc) (void) = 0; //declare reset function @ address 0
const int HARD_RESET_PIN = 4;


void setup() {

  // Make sure reset is not initiated from wire
  digitalWrite(HARD_RESET_PIN, HIGH);

  // Start LED chain
  pinMode(PULSE_PIN, OUTPUT);
  digitalWrite(PULSE_PIN, HIGH); 

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }

  delay(500);
  
  // Start serial
  start_puts();

  // Initialize MPU6050
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring!");
    delay(500);
  }
  
  // Calibrate gyroscope. The calibration must be at rest.
  mpu.calibrateGyro();
  
  // Set threshold sensivty. Default 3.
  mpu.setThreshold(3);

  // Check if operationel mode is activated
  if (OPERATIONAL_EN == 1) {
    set_mode = OPERATIONAL_MODE[opt_mode_index];
  }
}

void loop() {
  
  // Others
  unsigned long time_current;
  int update_stage;

  intensity = 0;
  color_angle = 0;
  rate = 0;
  
  unsigned long t1;
  unsigned long t2;
  unsigned long t3;

  int s1_color;
  int s2_color;
  int s3_color;

  int first_cycle = 1;

  // Local counters
  unsigned long color_angle_local;
  unsigned long local_counter;
  
  
  while(true) {  

    ///////////////////////////////////////////////////
    // Check if its time to change mode
    ///////////////////////////////////////////////////

    if (change_mode_state == MODE_WAIT_CHANGE) {

      // Power Cycle LED chain
      color_curr = COLOR_ALL;
      digitalWrite(PULSE_PIN, LOW);
      delay(1000); 
      digitalWrite(PULSE_PIN, HIGH); 
      delay(200); 
      
      // Set some color
      set_color(OPERATIONAL_COLOR[opt_mode_index]);
      
      // Wait
      while (true) {
        int ax, ay, az;
        Vector normAccel = mpu.readNormalizeAccel();
        ax = normAccel.XAxis;
        ay = normAccel.YAxis;
        az = normAccel.ZAxis;
        // Detect Gravity Pull
        const int GRAVITY_LIM = 1;
        int gravity_found = 0;
        int gravity_sign  = 1;
        int grav_x   = abs(ax) > GRAVITY_LIM;
        int grav_y   = abs(ay) > GRAVITY_LIM;
        int grav_z   = abs(az) > GRAVITY_LIM;
        int grav_all = grav_x + grav_y + grav_z;
        if (grav_all == 1) {
          if (grav_x == 1) {gravity_found = 1;}
          if (grav_y == 1) {gravity_found = 2;}
          if (grav_z == 1) {gravity_found = 3;}
          if ((grav_x*ax + grav_y*ay + grav_z*az) < 0) {
            gravity_sign = 0;
          }
        } 
        // Print
        if (1) {
          if (gravity_found == 0) {puts_s_ln("ERROR: Gravity not found!");}
        }

        // Correct position found again
        if (gravity_found == 3 && gravity_sign == 0) {
          break;
        } else {
          puts_s_ln("Please set in correct position!");
        }
      }

      // Point to next index
      opt_mode_index = opt_mode_index + 1;
      if (OPERATIONAL_MODE[opt_mode_index] == 99) {
        opt_mode_index = 0;
      }

      // Set color off
      set_color(COLOR_OFF);

      // Set new mode
      if (OPERATIONAL_EN == 1) {
        set_mode = OPERATIONAL_MODE[opt_mode_index];
      }
      
      // Restart mode
      first_cycle = 1;

      // Change state
      change_mode_state = MODE_RUNNING;
    }

    ///////////////////////////////////////////////////
    // Reset timer and set periods
    ///////////////////////////////////////////////////

    // Reset milisecond counter
    reset_millis();


    ///////////////////////////////////////////////////
    // Configuration of Mode 0 and 1
    ///////////////////////////////////////////////////

    if (set_mode == 0 || set_mode == 1) {

      // Set intesity MAX
      if (first_cycle == 1) {
        intensity = PWM_PERIOD;
        intensity_real = intensity * intensity_div;
      }
      
      // Set PWM periods
      time_action1_next = intensity;
      time_action2_next = PWM_PERIOD;
      time_action3_next = PWM_PERIOD;

      // Determine color settings
    
      // Set first color (RED) - If not zero
      s1_color = COLOR_IGNORE;
      if (intensity > (PWM_PERIOD/20)) {
        if (set_mode == 0) {
          s1_color = COLOR_ALL;
        } else {
          s1_color = COLOR_RED;
        }
      } 
      // Set second color (OFF/Blue) - Skip if fully saturated
      s2_color = COLOR_IGNORE;
      if (intensity < (PWM_PERIOD-PWM_PERIOD/20)) {
        if (set_mode == 0) {
          s2_color = COLOR_OFF;
        } else {
          s2_color = COLOR_BLUE;
        }
      }
      // Always ignore
      s3_color = COLOR_IGNORE;
    }


    ///////////////////////////////////////////////////
    // Configuration of Mode 2 and 3
    ///////////////////////////////////////////////////

    if (set_mode == 2 || set_mode == 3) {

      if (set_mode == 3) {
        if (first_cycle == 1) {
          color_angle_local = 0;
        } else {
          color_angle_local = (color_angle_local + intensity/25 + 1) % PWM_PERIOD;
        }
      } else {
        color_angle_local = color_angle;
      }

      unsigned long color_angle3 = (color_angle_local%(PWM_PERIOD/3))*3;
      unsigned long color_sel3   =  color_angle_local/(PWM_PERIOD/3);
      //puts_int_ln(color_angle3);
      //puts_int_ln(color_sel3);
      
      // Set PWM periods
      time_action1_next = color_angle3;
      time_action2_next = PWM_PERIOD;
      time_action3_next = PWM_PERIOD;

      // Determine color settings

      // Select the two colors - RED,BLUE,GREEN
      int s1_color_local;
      int s2_color_local;
      if (color_sel3 == 0) {
        s1_color_local = COLOR_RED;
        s2_color_local = COLOR_BLUE;
      } else if (color_sel3 == 1) {
        s1_color_local = COLOR_GREEN;
        s2_color_local = COLOR_RED;
      } else {
        s1_color_local = COLOR_BLUE;
        s2_color_local = COLOR_GREEN;
      }
    
      // Set first color - If not zero
      s1_color = COLOR_IGNORE;
      if (color_angle3 > (PWM_PERIOD/20)) {
        s1_color = s1_color_local;
      } 
      // Set second color - Skip if fully saturated
      s2_color = COLOR_IGNORE;
      if (color_angle3 < (PWM_PERIOD-PWM_PERIOD/20)) {
        s2_color = s2_color_local;
      }
      // Always ignore
      s3_color = COLOR_IGNORE;
    }


    ///////////////////////////////////////////////////
    // Configuration of Mode 4
    ///////////////////////////////////////////////////

    if (set_mode == 4) {

      // Preset random colors
      if (first_cycle == 1) {
        intensity = 1500;
        intensity_real = intensity * intensity_div;
        set_random_async();
      }
      
      // Set PWM periods
      time_action1_next = 0;
      time_action2_next = PWM_PERIOD;
      time_action3_next = PWM_PERIOD;

      // Determine color settings

      // Calc went to toggle colors
      if (first_cycle == 1) {
        local_counter = 0;
      } else {
        local_counter = local_counter + (intensity*intensity/PWM_PERIOD);
        // Send 1 pulse
        if (local_counter >= PWM_PERIOD) {
          send_pulse(1);
        } 
        local_counter = local_counter % PWM_PERIOD;
      }
    
      // Always ignore
      s1_color = COLOR_IGNORE;
      s2_color = COLOR_IGNORE;
      s3_color = COLOR_IGNORE;
    }

    
    ///////////////////////////////////////////////////
    // Configuration of Mode 5
    ///////////////////////////////////////////////////

    if (set_mode == 5) {

      // Set intesity MAX
      if (first_cycle == 1) {
        intensity = PWM_PERIOD;
        intensity_real = intensity * intensity_div;
        color_angle = 0;
        color_angle_real = color_angle * color_angle_div;
      }

      // Calc Color angle and color preset
      color_angle_local = color_angle;
      unsigned long color_angle3 = (color_angle_local%(PWM_PERIOD/3))*3;
      unsigned long color_sel3   =  color_angle_local/(PWM_PERIOD/3);
      //puts_int_ln(color_angle3);
      //puts_int_ln(color_sel3);

      // Set PWM periods
      
      time_action1_next = color_angle3*intensity/PWM_PERIOD;
      time_action2_next = time_action1_next + (PWM_PERIOD-color_angle3)*intensity/PWM_PERIOD;
      time_action3_next = PWM_PERIOD;

      //puts_int_ln(time_action1_next);
      //puts_int_ln(time_action2_next);

      // Select the two colors - RED,BLUE,GREEN
      int s1_color_local;
      int s2_color_local;
      if (color_sel3 == 0) {
        s1_color_local = COLOR_RED;
        s2_color_local = COLOR_BLUE;
      } else if (color_sel3 == 1) {
        s1_color_local = COLOR_GREEN;
        s2_color_local = COLOR_RED;
      } else {
        s1_color_local = COLOR_BLUE;
        s2_color_local = COLOR_GREEN;
      }
    
      // Set first and second color - If not zero
      s1_color = COLOR_IGNORE;
      s2_color = COLOR_IGNORE;
      if (time_action2_next > (PWM_PERIOD/20)) {
        s1_color = s1_color_local;
        s2_color = s2_color_local;
      }
      // Set Ignore if fully saturated
      if ((PWM_PERIOD-time_action2_next) < (PWM_PERIOD/20)) {
        s3_color = COLOR_IGNORE;
      } else {
        s3_color = COLOR_OFF;
      }
    }


    ///////////////////////////////////////////////////
    // Configuration of Mode 6
    ///////////////////////////////////////////////////

    if (set_mode == 6) {

      // Set intesity MAX
      if (first_cycle == 1) {
        intensity = PWM_PERIOD;
        intensity_real = intensity * intensity_div;
        rate = 0;
        rate_real = rate * rate_div;
        local_counter = 0;
      }
      
      // Set PWM periods
      time_action1_next = intensity;
      time_action2_next = PWM_PERIOD;
      time_action3_next = PWM_PERIOD;

      // Determine color settings
    
      // Set first color (White) - If not zero
      unsigned long rate_local = rate_real/750;
      s1_color = COLOR_IGNORE;
      if (intensity > (PWM_PERIOD/50)) {
        if (rate_local < 2) {
          // No blinks
          s1_color = COLOR_ALL;
        } else {
          // Set blinking
          local_counter = local_counter + 1;
          if (local_counter > (rate_local*2)) {
            s1_color = COLOR_ALL;
            local_counter = 0;
          } else if (local_counter > (rate_local*1)) {
            s1_color = COLOR_OFF;
          } else {
            s1_color = COLOR_ALL;
          }
        }
      } 
      // Set second color (OFF) - Skip if fully saturated
      s2_color = COLOR_IGNORE;
      if (intensity < (PWM_PERIOD-PWM_PERIOD/20)) {
        s2_color = COLOR_OFF;
      }
      // Always ignore
      s3_color = COLOR_IGNORE;
    }

    
    ///////////////////////////////////////////////////
    // Configuration of Mode 7
    ///////////////////////////////////////////////////

    if (set_mode == 7) {

      if (first_cycle == 1) {
        color_angle_local = 0;
        intensity = PWM_PERIOD;
        intensity_real = intensity * intensity_div;
        rate = 750;
        rate_real = rate * rate_div;
      } else {
        color_angle_local = (color_angle_local + rate/25 + 1) % PWM_PERIOD;
      }

      unsigned long color_angle3 = (color_angle_local%(PWM_PERIOD/3))*3;
      unsigned long color_sel3   =  color_angle_local/(PWM_PERIOD/3);
      //puts_int_ln(color_angle3);
      //puts_int_ln(color_sel3);
      
      // Set PWM periods

      time_action1_next = color_angle3*intensity/PWM_PERIOD;
      time_action2_next = time_action1_next + (PWM_PERIOD-color_angle3)*intensity/PWM_PERIOD;
      time_action3_next = PWM_PERIOD;

      //puts_int_ln(time_action1_next);
      //puts_int_ln(time_action2_next);

      // Determine color settings

      // Select the two colors - RED,BLUE,GREEN
      int s1_color_local;
      int s2_color_local;
      if (color_sel3 == 0) {
        s1_color_local = COLOR_RED;
        s2_color_local = COLOR_BLUE;
      } else if (color_sel3 == 1) {
        s1_color_local = COLOR_GREEN;
        s2_color_local = COLOR_RED;
      } else {
        s1_color_local = COLOR_BLUE;
        s2_color_local = COLOR_GREEN;
      }
      
      // Set first and second color - If not zero
      s1_color = COLOR_IGNORE;
      s2_color = COLOR_IGNORE;
      if (time_action2_next > (PWM_PERIOD/20)) {
        s1_color = s1_color_local;
        s2_color = s2_color_local;
      }
      // Set Ignore if fully saturated
      if ((PWM_PERIOD-time_action2_next) < (PWM_PERIOD/20)) {
        s3_color = COLOR_IGNORE;
      } else {
        s3_color = COLOR_OFF;
      }
    }

    
    ///////////////////////////////////////////////////
    // Determine when to read compas at it takes ~2ms
    ///////////////////////////////////////////////////
    
    if (time_action1_next > PWM_PERIOD/3) {
      update_stage = 1;
    } else if ((time_action2_next-time_action1_next) > PWM_PERIOD/3) {
      update_stage = 2;
    } else {
      update_stage = 3;
    }
    

    ///////////////////////////////////////////////////
    // Action 1 - Main Color (Always Set)
    ///////////////////////////////////////////////////

    // Read first current time
    time_current = micros();

    // Set Color 1
    set_color(s1_color);

    // Manage Gyre and Acc
    if (update_stage == 1) {manage_gyro_acc(update_stage);}
    
    // Wait for Action 1. is done
    while (time_current <= time_action1_next) {
      time_current = micros();
    }


    ///////////////////////////////////////////////////
    // Action 2 - Second Color
    ///////////////////////////////////////////////////
 
    // Set Color 2
    set_color(s2_color);

    // Manage Gyre and Acc
    if (update_stage == 2) {manage_gyro_acc(update_stage);}
     
    // Wait untill Action 2. is done
    while (time_current <= time_action2_next) {
      time_current = micros();
    }


    ///////////////////////////////////////////////////
    // Action 3 - No color
    ///////////////////////////////////////////////////

    // Set Color 3
    set_color(s3_color);

    // Manage Gyre and Acc
    if (update_stage == 3) {manage_gyro_acc(update_stage);}
    
    // Wait untill Action 3. is done
    while (time_current <= time_action3_next) {
      time_current = micros();
    }

    // First cycle is over
    first_cycle = 0;
  }
}

///////////////////////////////////////////////////
// DEBUG FUNCTIONS
///////////////////////////////////////////////////

void start_puts() {
  // Setup serial
    Serial.begin(115200);
    delay(200);
}

void puts_s(String s) {
  if (DBG_ENABLE == 1) {
    Serial.print(s);
  }
}
void puts_s_ln(String s) {
  if (DBG_ENABLE == 1) {
    Serial.println(s);
  }
}
void puts_float(float s) {
  if (DBG_ENABLE == 1) {
    Serial.print(s);
  }
}
void puts_float_ln(float s) {
  if (DBG_ENABLE == 1) {
    Serial.println(s);
  }
}
void puts_int(int s) {
  if (DBG_ENABLE == 1) {
    Serial.print(s);
  }
}
void puts_int_ln(int s) {
  if (DBG_ENABLE == 1) {
    Serial.println(s);
  }
}
void puts_long(long s) {
  if (DBG_ENABLE == 1) {
    Serial.print(s);
  }
}
void puts_long_ln(long s) {
  if (DBG_ENABLE == 1) {
    Serial.println(s);
  }
}

///////////////////////////////////////////////////
// SET LED FUNCTIONS
///////////////////////////////////////////////////

void set_random_async() {
  unsigned long t_start;
  unsigned long t_end;
  unsigned long t_diff;
  for (unsigned long i = 100000; i > 50; i = i - i/10) {
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
  puts_s_ln("set_random_asycn() Completed");
}

void send_pulse(int n) {
  unsigned long time_start;
  unsigned long time_end;
  unsigned long time_diff;
  const unsigned long pw = 5;
  // Stop if under 1
  if (n < 1) {
    return;
  }
  // Send [n] pulses of [pw] micro second each
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

void set_color(int color) {
  if (color == COLOR_IGNORE) {return;}
  int color_next;
  int color_diff;
  color_next = color;
  color_diff = color_next - color_curr;
  if (color_diff < 0) {
    color_diff = color_diff + 8;
  }
  color_curr = color_next;
  send_pulse(color_diff);
}

    
///////////////////////////////////////////////////
// Gyro and Acceleometer
///////////////////////////////////////////////////

void manage_gyro_acc(int update_stage) {

  int gx, gy, gz, ax, ay, az; 
  
  unsigned long t_start;
  unsigned long t_end;
  unsigned long t_diff;
  t_start = micros();

  const int LOCAL_DEBUG = 0;

  // TODO: Read acc to find gravity
  Vector normAccel = mpu.readNormalizeAccel();
  ax = normAccel.XAxis;
  ay = normAccel.YAxis;
  az = normAccel.ZAxis;

  // Read gyro
  Vector norm = mpu.readNormalizeGyro();
  gx = norm.XAxis;
  gy = norm.YAxis;
  gz = norm.ZAxis;

  // Print
  if (LOCAL_DEBUG) {
    puts_s_ln("--");
    puts_s("gx = ");
    puts_int(gx);
    puts_s(" gy = ");
    puts_int(gy);
    puts_s(" gz = ");
    puts_int_ln(gz);
    puts_s("ax = ");
    puts_int(ax);
    puts_s(" ay = ");
    puts_int(ay);
    puts_s(" az = ");
    puts_int_ln(az);
  }

  // Print min/max
  if (0) {
    if (gx > gx_max) { gx_max = gx; } 
    if (gx < gx_min) { gx_min = gx; } 
    if (gy > gy_max) { gy_max = gy; } 
    if (gy < gy_min) { gy_min = gy; } 
    if (gz > gz_max) { gz_max = gz; } 
    if (gz < gz_min) { gz_min = gz; } 
    if (ax > ax_max) { ax_max = ax; } 
    if (ax < ax_min) { ax_min = ax; } 
    if (ay > ay_max) { ay_max = ay; } 
    if (ay < ay_min) { ay_min = ay; } 
    if (az > az_max) { az_max = az; } 
    if (az < az_min) { az_min = az; } 
    
    puts_s("gx = ");
    puts_int(gx_max);
    puts_s("/");
    puts_int(gx_min);
    puts_s(" gy = ");
    puts_int(gy_max);
    puts_s("/");
    puts_int(gy_min);
    puts_s(" gz = ");
    puts_int(gz_max);
    puts_s("/");
    puts_int(gz_min);

    puts_s(" ax = ");
    puts_int(ax_max);
    puts_s("/");
    puts_int(ax_min);
    puts_s(" ay = ");
    puts_int(ay_max);
    puts_s("/");
    puts_int(ay_min);
    puts_s(" az = ");
    puts_int(az_max);
    puts_s("/");
    puts_int(az_min);
    
    puts_s_ln("");
  }

  // Detect Gravity Pull
  const int GRAVITY_LIM = 9;
  int gravity_found = 0;
  int gravity_sign  = 1;
  int grav_x   = abs(ax) >= GRAVITY_LIM;
  int grav_y   = abs(ay) >= GRAVITY_LIM;
  int grav_z   = abs(az) >= GRAVITY_LIM;
  int grav_all = grav_x + grav_y + grav_z;
  // Print
  if (LOCAL_DEBUG && 1) {
    puts_s("grav_x = ");
    puts_int(grav_x);
    puts_s(" grav_y = ");
    puts_int(grav_y);
    puts_s(" grav_z = ");
    puts_int(grav_z);
    puts_s(" grav_all = ");
    puts_int_ln(grav_all);
  }
  if (grav_all == 1) {
    if (grav_x == 1) {gravity_found = 1;}
    if (grav_y == 1) {gravity_found = 2;}
    if (grav_z == 1) {gravity_found = 3;}
    if ((grav_x*ax + grav_y*ay + grav_z*az) < 0) {
      gravity_sign = 0;
    }
  } else if (grav_all > 1) {
    //puts_s_ln("ERROR: Gravity found on more!");
  } else {
    //puts_s_ln("ERROR: Gravity not found!");
  }
  // Print
  if (LOCAL_DEBUG && 0) {
    if (gravity_found == 1) {puts_s("Gravity found in X");}
    if (gravity_found == 2) {puts_s("Gravity found in Y");}
    if (gravity_found == 3) {puts_s("Gravity found in Z");}
    if (grav_all == 1) {
      if (gravity_sign == 1)  {puts_s_ln(" negative!");} else {puts_s_ln(" positive!");}
    }
    if (gravity_found == 0) {puts_s_ln("ERROR: Gravity not found!");}
  }
  if (1) {
    if (gravity_found == 0) {puts_s_ln("ERROR: Gravity not found!");}
  }


  // Decide what to update
  
  int update_intensity = 0;
  int update_color = 0;
  int update_rate = 0;
  
  if (set_mode == 0 || set_mode == 1 || set_mode == 3 || set_mode == 4) {
    if (gravity_found == 3 && gravity_sign == 0) {
      update_intensity = 1;
    }
  }

  if (set_mode == 2) {
    if (gravity_found == 3 && gravity_sign == 0) {
      update_color = 1;
    }
  }

  if (set_mode == 5) {
    if (gravity_found == 3 && gravity_sign == 0) {
      update_color = 1;
    }
    if (gravity_found == 1) {
      update_intensity = 1;
      // Fwd X gyro date instead
      gz = gx;
    }
    if (gravity_found == 2) {
      update_intensity = 1;
      // Fwd Y gyro date instead
      gz = gy;
    }
  }
  
  if (set_mode == 6) {
    if (gravity_found == 3 && gravity_sign == 0) {
      update_intensity = 1;
    }
    if (gravity_found == 1) {
      update_rate = 1;
      // Fwd X gyro date instead
      gz = gx;
    }
    if (gravity_found == 2) {
      update_rate = 1;
      // Fwd Y gyro date instead
      gz = gy;
    }
  }

  if (set_mode == 7) {
    if (gravity_found == 3 && gravity_sign == 0) {
      update_rate = 1;
    }
    if (gravity_found == 1) {
      update_intensity = 1;
      // Fwd X gyro date instead
      gz = gx;
    }
    if (gravity_found == 2) {
      update_intensity = 1;
      // Fwd Y gyro date instead
      gz = gy;
    }
  }

  // Check if mode change is initiated
  if (gravity_found == 3 && gravity_sign == 1) {
    change_mode_state = MODE_WAIT_CHANGE;
  }
  
  // Set intensity

  if (update_intensity == 1) {
    // Multiply input
    gz = gz * intensity_mul ;
    // Check if intensity will be negative
    // Then add or saturate
    if (gz < 0) {
      int z_pos = gz * -1;
      unsigned long z_pos_uint = z_pos;
      if (z_pos_uint > intensity_real) {
        intensity_real = 0;
      } else {
        intensity_real = intensity_real + gz;
      }
    } else {
      intensity_real = intensity_real + gz;
      if (intensity_real >= (PWM_PERIOD*intensity_div)) {
        intensity_real = (PWM_PERIOD*intensity_div)-1;
      }
    }
    intensity = intensity_real/intensity_div;
  }

  // Set color angle

  if (update_color == 1) {
    // Multiply input
    gz = gz * color_angle_mul;
    // add color
    if (gz < 0) {
      int z_pos = gz * -1;
      unsigned long z_pos_uint = z_pos;
      if (z_pos_uint > color_angle_real) {
        color_angle_real = (PWM_PERIOD*color_angle_div) + color_angle_real - z_pos_uint;
      } else {
        color_angle_real = (color_angle_real - z_pos_uint) % (PWM_PERIOD*color_angle_div);
      }
    } else {
      unsigned long z_uint = gz;
      color_angle_real = (color_angle_real + z_uint) % (PWM_PERIOD*color_angle_div);
    }
    color_angle = color_angle_real/color_angle_div;
  }

  // Set rate

  if (update_rate == 1) {
    // Multiply input
    gz = gz * rate_mul;
    // Check if rate will be negative
    // Then add or saturate
    if (gz < 0) {
      int z_pos = gz * -1;
      unsigned long z_pos_uint = z_pos;
      if (z_pos_uint > rate_real) {
        rate_real = 0;
      } else {
        rate_real = rate_real + gz;
      }
    } else {
      rate_real = rate_real + gz;
      if (rate_real >= (PWM_PERIOD*rate_div)) {
        rate_real = (PWM_PERIOD*rate_div)-1;
      }
    }
    rate = rate_real/rate_div;
  }

  // Measure runtime of function
  if (0) {
    t_end = micros();
    t_diff = t_end - t_start;
    puts_int_ln(int(t_diff));
  }

  // Summary - Real
  if (0) {
    puts_s_ln("--");
    puts_s("    ");
    puts_s(", int: ");
    puts_long(intensity_real);
    puts_s(", col: ");
    puts_long(color_angle_real);
    puts_s(", rat: ");
    puts_long_ln(rate_real);
  }

  // Summary
  puts_s_ln("--");
  puts_s("S: ");
  puts_int(update_stage);
  puts_s(", int: ");
  puts_int(intensity);
  puts_s(", col: ");
  puts_int(color_angle);
  puts_s(", rat: ");
  puts_int_ln(rate);


  // Measure runtime of function
  if (0) {
    t_end = micros();
    t_diff = t_end - t_start;
    puts_int_ln(int(t_diff));
  }
}


///////////////////////////////////////////////////
// OTHER
///////////////////////////////////////////////////

void reset_millis()
{
  extern volatile unsigned long timer0_millis, timer0_overflow_count;
  noInterrupts();
  timer0_millis = timer0_overflow_count = 0;
  interrupts();  
}
