#include <SPI.h>
#include <TimerOne.h>

// ===== Pin definitions =====
#define LED1h 23
#define LED1k 22
#define LED2h 19
#define LED2k 18

#define SS1h 7
#define SS1k 38
#define SS2h 6
#define SS2k 5

//#define ENC2RAD 0.00153398  // Conversion from encoder counts to rad (adjust as needed)
#define Kt 0.043             // Torque constant [Nm/A]
#define Kp 0.5              // Proportional gain for torque error
#define Ki 0.001              // Integral gain (set to 0 initially)

// Friction model parameters (to be adjusted from your identification)
const float ENC2RAD = 2*PI/524288;
// Friction model parameters
const float Tc_fric = 0.011f;  // Coulomb friction [Nm]
const float Ts_fric = 0.0f;          // Static friction [Nm]
const float omega_s = 0.3469f;   // Stribeck velocity [rad/s]
const float alpha_s   = 0.444f;    // Stribeck shape factor
const float B       = 0.000111f; // Viscous friction [Nm/(rad/s)]


// === Global constants for control frequency ===
const int control_freq_hz = 300;         // Control frequency in Hz
//const float dt = 1.0 / control_freq_hz;   // Time step in seconds (e.g., 0.002 for 500Hz)
const float dt_ms = 1000/control_freq_hz;
float t_ms = 0;
float cur_cmd = 0;
float cur_act = 0;
long prev_enc = 0;
long enc_now = 0;
float omega = 0.0;
float e_int = 0.0;
unsigned long prev_print_t = 4294967295;  // For steady printing. The first print time will never be the same as this num.
volatile unsigned long tick_count = 0;


// --- Low-frequency bias estimator (for residual push/pull near 0 Nm) ---
float tau_bias = 0.0f;         // Estimated low-frequency torque bias [Nm]
float bias_alpha = 0.0f;       // IIR coefficient computed from cutoff and Ts
const float bias_cutoff_hz = 0.5f;  // Bias estimator cutoff (~0.2–1.0 Hz is typical)

// Update conditions (only learn bias near "neutral")
const float tau_ref_th = 0.01f;     // Learn bias only when |Tp_ref| is small [Nm]
const float omega_th   = 0.1f;      // And |omega| is small [rad/s] (~5.7 deg/s)

// ==== Globals (add) ====
const int ENC_BITS = 19;
const long ENC_MOD  = 1L << ENC_BITS;   // 524288
const long ENC_HALF = ENC_MOD >> 1;     // 262144

long enc_prev1 = 0;  // x[n-1]
long enc_prev2 = 0;  // x[n-2]

const float dt = 1.0f / float(control_freq_hz); // control period [s]

// Reusable IIR alpha
inline float iirAlpha(float cutoff_hz, float dt) {
  return expf(-2.0f * PI * cutoff_hz * dt);
}

// Velocity LPF
float omega_lpf    = 0.0f;
const float omega_fc_hz = 25.0f;                 // 20–30 Hz
float omega_alpha  = iirAlpha(omega_fc_hz, dt);

// Unwrap encoder delta (counts)
inline long unwrapEncDelta(long now, long prev) {
  long d = now - prev;
  if (d >  ENC_HALF) d -= ENC_MOD;
  if (d < -ENC_HALF) d += ENC_MOD;
  return d;
}

SPISettings settings(100000, MSBFIRST, SPI_MODE3); //1MHz max

// ===== Globals for SPI comm =====
bool  conn1;
long  enc1;
float cur1;
byte last_rx_buf[12];
byte last_tx_buf[12];

// float refT[] = {
//   -0.001627031189353, -0.010078134806660, -0.019665055287961, -0.028449318847842,
//   -0.035490434652401, -0.040735705640484, -0.044278299173914, -0.046483940264661,
//   -0.047862586378117, -0.048865240917389, -0.049824949487050, -0.050939523812327,
//   -0.052302587460691, -0.053956909666418, -0.055916233613297, -0.058138984270470,
//   -0.060535773640330, -0.062966925474941, -0.065243978655166, -0.067132944173804,
//   -0.068349359525542, -0.068588261887857, -0.067571582397193, -0.064978413580525,
//   -0.060575065155178, -0.054480797499138, -0.047138104002056, -0.039079301695244,
//   -0.030977734228921, -0.023438668348530, -0.016788037942653, -0.011143652372293,
//   -0.006623949318000, -0.003236190508745, -0.000910088917431,  0.000458748984804,
//    0.001073982824656,  0.001214655788092,  0.001175950182825,  0.001181999947384,
//    0.001345780427386,  0.001695567089574,  0.002206015874143,  0.002808956358927,
//    0.003390201082491,  0.003804974224416,  0.003985610244484,  0.004088539502501,
//    0.004036081121161,  0.001432784028159
// };

float refT[] = {
  -0.024044409379305, -0.057379170811646, -0.082347726221913, -0.101766214933962,
  -0.113655220028023, -0.117663416354635, -0.115056381548052, -0.108405281732599,
  -0.099988540207362, -0.090879528747806, -0.081642672434475, -0.072527546212679,
  -0.063458960718276, -0.054249479016750, -0.044811952196179, -0.035216063994700,
  -0.025540612547182, -0.015911614705552, -0.006501919148448,  0.002432548302564,
   0.010745191351587,  0.018194139928874,  0.024496791284796,  0.029439389639198,
   0.032744632722659,  0.034241708096117,  0.034058915272989,  0.032568936986831,
   0.030153271810964,  0.027117638786424,  0.023741043145104,  0.020194030904439,
   0.016649395025736,  0.013355774514323,  0.010522464005195,  0.008247017600510,
   0.006424851063036,  0.004835031486204,  0.003273337451531,  0.001722633564767,
   0.000388970497158, -0.000470791700955, -0.000641985659551, -0.000067311244177,
   0.001179679523487,  0.002736159666071,  0.004406850039273,  0.007230227976409,
   0.009152320921940, -0.006956574288570
}; //hip

int indexRefT = 0;
int refTsize = sizeof(refT) / sizeof(refT[0]);
float tor_cmd = 0.0f;

void setup() {
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT); //MOSI
  pinMode(12, INPUT);  //MISO
  pinMode(13, OUTPUT); //SCK

  pinMode(38, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(5, OUTPUT);

  pinMode(23, OUTPUT);
  pinMode(22, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(18, OUTPUT);
  // --- Compute IIR coefficient for bias estimator ---
  // alpha = exp(-2*pi*fc*Ts);
  float dt = 1.0f / float(control_freq_hz);
  bias_alpha = expf(-2.0f * PI * bias_cutoff_hz * dt);

  Serial.begin(57600);
  SPI.begin();
  delay(100);

  // === Initialize encoder histories safely ===
  bool init_conn = false;

  // Keep trying until connection is established
  while (!init_conn) {
    send_current_command(38, 0, 0.0f, &init_conn, &enc1, &cur1);
    delay(10);  // small delay to avoid hammering SPI too fast
  }

  // Once connected, read multiple times to stabilize values
  for (int i = 0; i < 5; i++) {
    send_current_command(38, 0, 0.0f, &conn1, &enc1, &cur1);
    delay(5);
  }
  enc_prev1 = enc1;
  enc_prev2 = enc1;

  // If you compute bias_alpha elsewhere, use dt:
  bias_alpha = iirAlpha(bias_cutoff_hz, dt);

  Timer1.initialize(1000000/control_freq_hz);
  Timer1.attachInterrupt(timerISR); // blinkLED to run every 0.15 seconds
}

void loop() {

  tor_cmd = refT[indexRefT]/2;

  indexRefT++;

  // Wrap back to 0 if past last index
  if (indexRefT >= refTsize) {
    indexRefT = 0;
  }

  //Serial.println(tor_cmd, 6);

  delay(10);
}

// === Friction model ===
float computeFriction(float omega) {
  float sign_w = (omega > 0) - (omega < 0);
  float strb = (Ts_fric - Tc_fric) * exp(-pow(abs(omega) / omega_s, alpha_s));
  return (Tc_fric + strb) * sign_w + B * omega;
}

// Optional: quick flip to test friction sign if needed
const int FRIC_SIGN = +1;  // try -1 only for debugging

void timerISR() {
  // === I/O ===
  send_current_command(38, 1, cur_cmd, &conn1, &enc1, &cur1);
  long enc_now = enc1;
  float i_meas = cur1;               // [A]
  float Tm_meas = Kt * i_meas;       // measured motor torque [Nm]

  // === Velocity estimation (your improved diff + LPFをここで計算してomegaへ) ===
  // ... omega already computed above ...

  // === Friction feedforward (model on cable; opposes motion) ===
  float Tfric_hat = FRIC_SIGN * computeFriction(omega);

  // === Reference motor torque (pulley ref + friction FF) ===
  float Tp_ref = 0.0f;                  // desired pulley torque [Nm]
  float Tm_ref = Tp_ref + Tfric_hat;       // target motor torque [Nm]

  // --- Low-frequency bias learning near neutral ---
  // Learn residual motor-side bias so that Tm_ref absorbs offsets
  if (fabsf(Tp_ref) < tau_ref_th && fabsf(omega) < omega_th) {
      float err_slow = Tm_meas - Tm_ref;   // positive if we over-deliver torque
      tau_bias = bias_alpha * tau_bias + (1.0f - bias_alpha) * err_slow;
  }
  Tm_ref -= tau_bias; // compensate learned bias on the reference side

  // === Motor-torque-domain PI ===
  float e = Tm_ref - Tm_meas;
  e_int += e * dt;

  // Anti-windup (conditional integration)
  float Tm_cmd_pi = Kp * e + Ki * e_int;
  float Tm_cmd = Tm_ref + Tm_cmd_pi;       // FF-in-reference, PI tracks it

  // === Current command & saturation ===
  cur_cmd = Tm_cmd / Kt;
  if (cur_cmd >  2.0f) { cur_cmd =  2.0f; if (e > 0) e_int -= e * dt; } // back out I when saturated
  if (cur_cmd < -2.0f) { cur_cmd = -2.0f; if (e < 0) e_int -= e * dt; }

  // shift histories etc...
  //Serial.println(conn1);
  Serial.println(cur_cmd);
  //Serial.println(cur1, 5);
}

// void timerISR() {
//   // SPI exchange (read encoder, etc.)
//   send_current_command(38, 1, cur_cmd, &conn1, &enc1, &cur1);
//   long enc_now = enc1;

//   // 3-point backward difference with wrap handling
//   long num_counts = (3L*enc_now - 4L*enc_prev1 + enc_prev2);
//   float omega_raw = (num_counts * ENC2RAD) / (2.0f * dt);

//   // First-order IIR LPF
//   omega      = omega_alpha * omega_lpf + (1.0f - omega_alpha) * omega_raw;
//   omega_lpf  = omega;

//   // Shift history
//   enc_prev2 = enc_prev1;
//   enc_prev1 = enc_now;

//   // === Read actual current and compute torque ===
//   //float i_meas = cur1/11000000.0;
//   float i_meas = cur1;
//   float Tm_meas = Kt * i_meas;
//   float Tfric = computeFriction(omega);
//   float Tp_meas = Tm_meas - Tfric;

//   // === Torque control ===
//   float Tp_ref = 0.0f; //Nm -0.013

//   // --- Low-frequency bias update (only near neutral) ---
//   if (fabsf(Tp_ref) < tau_ref_th && fabsf(omega) < omega_th) {
//       float err_slow = (Tp_meas - Tp_ref); // Residual push/pull
//       tau_bias = bias_alpha * tau_bias + (1.0f - bias_alpha) * err_slow;
//   }
  
//   // Use bias-compensated measured torque in error calc: Tp_meas - tau_bias
//   //float error = Tp_ref - Tp_meas;
//   float error = Tp_ref - (Tp_meas - tau_bias);// bias rejection
//   e_int += error * (dt_ms / 1000.0);

//   float Tm_cmd = Kp * error + Ki * e_int + Tfric;
//   cur_cmd = Tm_cmd / Kt;

//   // Saturate output
//   if (cur_cmd > 2.0) cur_cmd = 2.0;
//   if (cur_cmd < -2.0) cur_cmd = -2.0;

//   //Serial.println(conn1);
//   //Serial.println(cur_cmd);
//   //Serial.println(cur1, 5);
// }

void send_current_command(int SS, bool servo_on, float current_cmd, bool* connection, long* val_enc, float* val_cur) {
  byte tx_buf[12] = {0};
  byte rx_buf[12] = {0};

  // --- Build TX packet ---
  tx_buf[0] = servo_on ? 0x08 : 0x00;
  memcpy(&tx_buf[3], &current_cmd, sizeof(float));

  // Calculate checksum
  uint16_t checksum = 0;
  for (int i = 0; i < 10; i += 2) {
    uint16_t w = tx_buf[i] | (tx_buf[i + 1] << 8);
    checksum += w;
  }
  checksum &= 0x7FFF;
  checksum |= 0x8000;
  tx_buf[10] = checksum & 0xFF;
  tx_buf[11] = (checksum >> 8) & 0xFF;

  // --- SPI Transfer ---
  SPI.beginTransaction(settings);
  delayMicroseconds(5);
  digitalWrite(SS, LOW);
  delayMicroseconds(10);
  for (int i = 0; i < 12; i++) {
    rx_buf[i] = SPI.transfer(tx_buf[i]);
  }
  delayMicroseconds(10);
  digitalWrite(SS, HIGH);
  SPI.endTransaction();

  // Save last TX/RX
  memcpy(last_tx_buf, tx_buf, 12);
  memcpy(last_rx_buf, rx_buf, 12);

  // --- Parse RX ---
  uint16_t rx_checksum = rx_buf[10] | (rx_buf[11] << 8);
  uint16_t computed = 0;
  for (int i = 0; i < 10; i += 2) {
    uint16_t w = rx_buf[i] | (rx_buf[i + 1] << 8);
    computed += w;
  }
  computed &= 0x7FFF;
  computed |= 0x8000;

  *connection = (rx_checksum == computed);

  int32_t enc_raw = *((int32_t*)&rx_buf[1]);
  *val_enc = enc_raw;

  // float cur_f;
  // memcpy(&cur_f, &rx_buf[5], sizeof(float));
  // *val_cur = (long)(cur_f * 1e6);

  memcpy(val_cur, &rx_buf[5], sizeof(float)); // directly store float amps
}

void validate(byte* rx) {
  // Print TX
  Serial.print("TX: ");
  for (int i = 0; i < 12; i++) {
    Serial.print(last_tx_buf[i], HEX); Serial.print(" ");
  }
  Serial.println();

  // Print RX
  Serial.print("RX: ");
  for (int i = 0; i < 12; i++) {
    Serial.print(rx[i], HEX); Serial.print(" ");
  }
  Serial.println();

  // Checksum validation
  uint16_t rx_checksum = rx[10] | (rx[11] << 8);
  uint16_t computed = 0;
  for (int i = 0; i < 10; i += 2) {
    uint16_t w = rx[i] | (rx[i + 1] << 8);
    computed += w;
  }
  computed &= 0x7FFF;
  computed |= 0x8000;

  Serial.print("Received checksum: 0x"); Serial.println(rx_checksum, HEX);
  Serial.print("Computed checksum: 0x"); Serial.println(computed, HEX);
}
