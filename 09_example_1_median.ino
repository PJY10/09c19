// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0     // sound velocity at 24 celsius degree (unit: m/sec)
#define INTERVAL 25       // sampling interval (unit: msec)
#define PULSE_DURATION 10 // ultra-sound Pulse Duration (unit: usec)
#define _DIST_MIN 100     // minimum distance to be measured (unit: mm)
#define _DIST_MAX 300     // maximum distance to be measured (unit: mm)

#define TIMEOUT ((INTERVAL / 2) * 1000.0) // maximum echo waiting time (unit: usec)
#define SCALE (0.001 * 0.5 * SND_VEL)     // coefficent to convert duration to distance

#define _EMA_ALPHA 0.5    // EMA weight of new sample (range: 0 to 1)

// -----------------------[ Median filter params ]-----------------------
// Change this number to 3, 10, or 30 for screenshots requested in the PDF.
#ifndef MEDIAN_WINDOW
#define MEDIAN_WINDOW 30
#endif
// ---------------------------------------------------------------------

// aliases to match the PDF terminology exactly
#define TRIG PIN_TRIG
#define ECHO PIN_ECHO

// globals
unsigned long last_ms = 0;
float dist_raw = 0.0f;
float dist_ema = 0.0f;
float dist_median = 0.0f;
bool ema_initialized = false;

// median ring buffer
float m_buf[MEDIAN_WINDOW];
unsigned int m_count = 0;   // number of valid samples loaded so far (<= MEDIAN_WINDOW)
unsigned int m_head = 0;    // next insert position

// utility: trigger ultrasonic and read echo duration then convert to distance in mm
static float read_ultrasonic_mm() {
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);

  // issue a 10us trigger pulse
  digitalWrite(TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  // convert pulse width to distance
  unsigned long dur = pulseIn(ECHO, HIGH, (unsigned long)TIMEOUT);
  // dur is round-trip microseconds. distance(mm) = dur * SCALE
  return ((float)dur) * SCALE; // may be 0 on timeout
}

// utility: insert into circular buffer and compute median
static float median_update_and_get(float x) {
  // insert value
  m_buf[m_head] = x;
  if (m_count < MEDIAN_WINDOW) m_count++;
  m_head = (m_head + 1) % MEDIAN_WINDOW;

  // copy valid values to temp
  const unsigned int n = m_count;
  float tmp[MEDIAN_WINDOW];
  for (unsigned int i = 0; i < n; ++i) {
    tmp[i] = m_buf[i];
  }

  // simple insertion sort since N is small
  for (unsigned int i = 1; i < n; ++i) {
    float key = tmp[i];
    int j = i - 1;
    while (j >= 0 && tmp[j] > key) {
      tmp[j + 1] = tmp[j];
      j--;
    }
    tmp[j + 1] = key;
  }

  // return middle value
  if (n == 0) return x;
  if (n % 2 == 1) {
    return tmp[n / 2];
  } else {
    // even count: mean of the two middle values
    return 0.5f * (tmp[n / 2 - 1] + tmp[n / 2]);
  }
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(TRIG, OUTPUT);
  pinMode(ECHO, INPUT);
  Serial.begin(57600);
  // optional LED blink to indicate boot
  digitalWrite(PIN_LED, HIGH);
  delay(50);
  digitalWrite(PIN_LED, LOW);
}

void loop() {
  const unsigned long now = millis();
  if (now - last_ms < INTERVAL) return;
  last_ms = now;

  // 1) get raw distance (timeout yields 0). Do NOT replace with previous valid.
  dist_raw = read_ultrasonic_mm();

  // 2) EMA from PDF. Keep identical symbol names.
  if (!ema_initialized) {
    dist_ema = dist_raw;
    ema_initialized = true;
  } else {
    dist_ema = _EMA_ALPHA * dist_raw + (1.0f - _EMA_ALPHA) * dist_ema; // EMAk = a*dk + (1-a)*EMAk-1
  }

  // 3) Median filter only. No "previous valid" substitution. No range gating.
  dist_median = median_update_and_get(dist_raw);

  // 4) Emit values for Serial Plotter, matching slide 20.
  Serial.print("Min:");     Serial.print(_DIST_MIN);
  Serial.print(",raw:");    Serial.print(dist_raw);
  Serial.print(",ema:");    Serial.print(dist_ema);
  Serial.print(",median:"); Serial.print(dist_median);
  Serial.print(",Max:");    Serial.print(_DIST_MAX);
  Serial.println("");
}
