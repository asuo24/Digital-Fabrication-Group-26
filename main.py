import machine
import utime
import network

# === Configuration ===
# Pin assignments
PIR_PIN = 20        # Motion sensor pin
TRIG_PIN = 3        # Ultrasonic trigger pin
ECHO_PIN = 2        # Ultrasonic echo pin 
SERVO_PIN = 11      # Servo control pin

# Servo settings
SERVO_FREQ = 50     # PWM frequency (Hz)
SERVO_MIN = 1600    # Duty cycle for 0 degrees
SERVO_MAX = 8500    # Duty cycle for 180 degrees
SERVO_CLOSED = 0    # Lid closed position (degrees)
SERVO_OPEN = 90     # Lid open position (degrees)

# Timing (milliseconds)
ACTIVE_TIME = 10000  # How long to stay active after motion
POLL_INTERVAL = 50  # How often to check distance
CLOSE_DELAY = 4000   # Delay before closing after person leaves

# Ultrasonic settings
MAX_FAILURES = 5            # Bad readings before forcing lid open
DISTANCE_THRESHOLD = 5      # cm difference to trigger detection
CALIBRATION_SAMPLES = 5     # Readings to average during calibration

# Logging level: 0=ERROR, 1=WARN, 2=INFO, 3=DEBUG
LOG_LEVEL = 3

# === Simple Logger ===
def log(msg, level=2):
    if level <= LOG_LEVEL:
        levels = ["ERR", "WRN", "INFO", "DBG"]
        print(f"{levels[level]}:{utime.ticks_ms():>8} | {msg}")

# === Helper Functions ===
def disable_wifi():
    """Disable WiFi to save power"""
    try:
        # Disable station mode
        sta = network.WLAN(network.STA_IF)
        if sta.active():
            sta.active(False)
            log("WiFi station disabled", 1)
        
        # Disable access point mode
        ap = network.WLAN(network.AP_IF)
        if ap.active():
            ap.active(False)
            log("WiFi AP disabled", 1)
    except Exception as e:
        log(f"WiFi disable failed: {e}", 0)

# === Hardware Classes ===
class Servo:
    """Servo motor controller"""
    def __init__(self, pin):
        self.pwm = machine.PWM(machine.Pin(pin))
        self.pwm.freq(SERVO_FREQ)
        self.angle = None

    def move(self, angle):
        """Move servo to specified angle (0-180)"""
        angle = min(180, max(0, angle))
        duty = int(SERVO_MIN + (SERVO_MAX - SERVO_MIN) * angle / 180)
        self.pwm.duty_u16(duty)
        self.angle = angle
        log(f"Servo → {angle}° (duty={duty})", 3)
        utime.sleep_ms(600)  # Allow time to reach position

class Ultrasonic:
    """HC-SR04 ultrasonic distance sensor"""
    def __init__(self, trig_pin, echo_pin):
        self.trig = machine.Pin(trig_pin, machine.Pin.OUT)
        self.echo = machine.Pin(echo_pin, machine.Pin.IN)

    def measure(self):
        """Measure distance in cm, returns negative value on error"""
        # Send trigger pulse
        self.trig.low()
        utime.sleep_us(2)
        self.trig.high()
        utime.sleep_us(10)
        self.trig.low()
        
        # Measure response time
        try:
            pulse_time = machine.time_pulse_us(self.echo, 1, 35000)
        except OSError:
            return -1  # Timeout error
            
        if pulse_time < 0:
            return -1  # Reading error
            
        # Convert to distance (cm)
        distance = pulse_time / 58.3
        
        # Validate reading
        if not 2 <= distance <= 400:
            return -2  # Out of valid range
            
        return distance

# === Main Controller ===
class LidController:
    """Smart lid controller"""
    def __init__(self):
        # Disable wifi to save power
        disable_wifi()

        # Initialize hardware
        self.motion = machine.Pin(PIR_PIN, machine.Pin.IN, machine.Pin.PULL_DOWN)
        self.distance = Ultrasonic(TRIG_PIN, ECHO_PIN)
        self.servo = Servo(SERVO_PIN)

        # Initialize state
        self.active = False          # System active state
        self.lid_open = False        # Current lid position
        self.person_present = False  # Person detection state
        self.fail_count = 0          # Consecutive bad distance readings
        
        # Timestamps
        self.last_motion = 0         # Last motion detection time
        self.last_poll = 0           # Last distance measurement time
        self.person_left_at = 0      # When person was last detected leaving
        
        # Calibrate and initialize
        self.baseline = self.calibrate()
        self.test_servo()
        
    def calibrate(self):
        """Determine baseline distance to ground"""
        log("Calibrating baseline distance...", 1)
        readings = []
        attempts = 0
        
        # Take multiple readings
        while len(readings) < CALIBRATION_SAMPLES and attempts < 10:
            attempts += 1
            dist = self.distance.measure()
            
            if dist > 0:  # Valid reading
                readings.append(dist)
                log(f"Calibration reading {len(readings)}: {dist:.1f} cm", 2)
            else:
                log(f"Invalid calibration reading: {dist}", 2)
                
            utime.sleep_ms(200)
        
        # Calculate average
        if readings:
            avg = sum(readings) / len(readings)
            log(f"Baseline distance: {avg:.1f} cm (±{DISTANCE_THRESHOLD} cm)", 1)
            return avg
        else:
            log("Calibration failed, using default value", 0)
            return 50.0  # Default fallback
            
    def test_servo(self):
        """Test servo with initialization sequence"""
        self.servo.move(SERVO_CLOSED)
        self.servo.move(SERVO_OPEN)         # Open and close test
        self.servo.move(SERVO_CLOSED)
        log("Servo initialization complete", 2)
        
    def open_lid(self, reason):
        """Open the lid if closed"""
        if not self.lid_open:
            log(f"Opening lid: {reason}", 1)
            self.servo.move(SERVO_OPEN)
            self.lid_open = True
            
    def close_lid(self):
        """Close the lid if open"""
        if self.lid_open:
            log("Closing lid", 1)
            self.servo.move(SERVO_CLOSED)
            self.lid_open = False

    def run(self):
        """Main control loop"""
        log("Smart lid system started", 2)
        
        while True:
            now = utime.ticks_ms()
            
            # --- Check for motion ---
            if self.motion.value():
                if not self.active:
                    log("Motion detected → ACTIVE", 2)
                    self.active = True
                    self.fail_count = 0
                self.last_motion = now
            
            # --- Process active state ---
            if self.active:
                # Check distance periodically
                if utime.ticks_diff(now, self.last_poll) >= POLL_INTERVAL:
                    self.last_poll = now
                    dist = self.distance.measure()
                    
                    # Log the measurement
                    if dist >= 0:
                        log(f"Distance: {dist:.1f} cm", 2)
                    else:
                        log(f"Invalid distance reading: {dist}", 2)
                        
                    if dist < 0:  # Error reading
                        self.fail_count += 1
                        log(f"Ultrasonic fail {self.fail_count}/{MAX_FAILURES} (code {dist})", 2)
                        
                        # Open lid after too many failures
                        if self.fail_count >= MAX_FAILURES and not self.lid_open:
                            self.open_lid("consecutive reading failures")
                            self.fail_count = 0
                            
                    else:  # Valid reading
                        if self.fail_count:
                            log(f"Valid reading obtained, resetting fail counter", 3)
                        self.fail_count = 0
                        
                        # Check for person presence
                        diff = abs(dist - self.baseline)
                        if diff > DISTANCE_THRESHOLD:
                            # Person detected
                            if not self.person_present:
                                log(f"Person detected (diff: {diff:.1f} cm)", 2)
                                self.person_present = True
                                
                            # Open lid if needed
                            if not self.lid_open:
                                self.open_lid(f"distance changed by {diff:.1f} cm")
                                
                            # Reset absence tracking
                            self.person_left_at = 0
                            
                        else:
                            # No person detected
                            if self.person_present:
                                log("Person moved away", 2)
                                self.person_present = False
                                self.person_left_at = now
                
                # Close lid if person has been gone for a while
                if (self.lid_open and 
                    not self.person_present and 
                    self.person_left_at > 0 and 
                    utime.ticks_diff(now, self.person_left_at) > CLOSE_DELAY):
                    log("Person absent, closing lid", 1)
                    self.close_lid()
                    self.person_left_at = 0
                
                # Check for overall inactivity timeout
                if (not self.lid_open and 
                    utime.ticks_diff(now, self.last_motion) > ACTIVE_TIME):
                    log("Inactivity timeout → SLEEP", 2)
                    self.active = False
                    self.close_lid()
                    self.person_present = False
                    self.person_left_at = 0
                    continue  # Skip delay
                
                utime.sleep_ms(10)  # 100Hz when active
                
            # --- Inactive state, low power ---
            else:
                utime.sleep_ms(100)  # 10Hz when idle

# === Program Entry Point ===
def main():
    log("="*34, 2)
    log("SMART LID CONTROLLER STARTING", 2)
    log("="*34, 2)
    
    controller = LidController()
    controller.run()

if __name__ == "__main__":
    main()
