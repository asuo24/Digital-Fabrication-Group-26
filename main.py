import machine
import utime
import network
import math

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

# Delay in milliseconds between each 1-degree step.
SERVO_START_DELAY_MS = 15 # Initial delay (slowest)
SERVO_END_DELAY_MS = 5    # Final delay (fastest)

# Timing (milliseconds)
ACTIVE_TIME = 10000  # How long to stay active after motion
POLL_INTERVAL = 50  # How often to check distance
CLOSE_DELAY = 4000   # Delay before closing after person leaves

# Ultrasonic settings
MAX_FAILURES = 5            # Bad readings before forcing lid open
DISTANCE_THRESHOLD = 5      # cm difference to trigger detection
CALIBRATION_SAMPLES = 5     # Readings to average during calibration
CALIBRATION_DELAY = 10      # delay calibration by this amount of seconds

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
        sta = network.WLAN(network.STA_IF)
        if sta.active():
            sta.active(False)
            log("WiFi station disabled", 1)

        ap = network.WLAN(network.AP_IF)
        if ap.active():
            ap.active(False)
            log("WiFi AP disabled", 1)
    except Exception as e:
        log(f"WiFi disable failed: {e}", 0)

# === Hardware Classes ===
class Servo:
    """Servo motor controller with smooth, accelerating movement"""
    def __init__(self, pin):
        self.pwm = machine.PWM(machine.Pin(pin))
        self.pwm.freq(SERVO_FREQ)
        self.current_angle = None
        self.start_delay = max(SERVO_START_DELAY_MS, SERVO_END_DELAY_MS)
        self.end_delay = SERVO_END_DELAY_MS
        if self.start_delay < SERVO_END_DELAY_MS:
             log(f"Servo Warning: START_DELAY ({SERVO_START_DELAY_MS}) < END_DELAY ({SERVO_END_DELAY_MS}). Using {self.start_delay} for both.", 1)
             self.end_delay = self.start_delay


    def _angle_to_duty(self, angle):
        """Converts angle (0-180) to PWM duty cycle"""
        # Clamp angle to valid range just in case
        angle = max(0, min(180, angle))
        return int(SERVO_MIN + (SERVO_MAX - SERVO_MIN) * angle / 180)

    def move(self, target_angle):
        """Move servo smoothly to specified angle (0-180) with acceleration"""
        target_angle = min(180, max(0, target_angle))

        # If it's the first move or angle is unknown, jump directly
        if self.current_angle is None:
            log(f"Servo → {target_angle}° (initial jump)", 3)
            duty = self._angle_to_duty(target_angle)
            self.pwm.duty_u16(duty)
            self.current_angle = target_angle
            utime.sleep_ms(600) # Allow time for initial move
            return

        # Smooth movement only if target is different
        if target_angle == self.current_angle:
            log(f"Servo already at {target_angle}°", 3)
            return

        log(f"Servo accelerating: {self.current_angle}° → {target_angle}°", 3)

        initial_angle = self.current_angle
        total_angle_change = abs(target_angle - initial_angle)

        # Determine step direction (+1 or -1 degree)
        step = 1 if target_angle > initial_angle else -1

        # Move incrementally from current angle to target angle
        start_loop = initial_angle + step
        stop_loop = target_angle + step

        for angle in range(start_loop, stop_loop, step):
            duty = self._angle_to_duty(angle)
            self.pwm.duty_u16(duty)

            # Calculate movement progress (0.0 to 1.0)
            angle_moved = abs(angle - initial_angle)
            progress = angle_moved / total_angle_change if total_angle_change > 0 else 1.0

            # Linear interpolation for delay
            current_delay = self.start_delay - (self.start_delay - self.end_delay) * progress
            current_delay = max(self.end_delay, int(current_delay))

            utime.sleep_ms(current_delay) # Use calculated delay

        final_duty = self._angle_to_duty(target_angle)
        self.pwm.duty_u16(final_duty)
        self.current_angle = target_angle
        log(f"Servo → {target_angle}° (move complete)", 3)

class Ultrasonic:
    """HC-SR04 ultrasonic distance sensor"""
    def __init__(self, trig_pin, echo_pin):
        self.trig = machine.Pin(trig_pin, machine.Pin.OUT)
        self.echo = machine.Pin(echo_pin, machine.Pin.IN)

    def measure(self):
        """Measure distance in cm, returns negative value on error"""
        self.trig.low()
        utime.sleep_us(2)
        self.trig.high()
        utime.sleep_us(10)
        self.trig.low()

        try:
            pulse_time = machine.time_pulse_us(self.echo, 1, 40000)
        except OSError as e:
            return -1 # Timeout error

        if pulse_time < 0:
            return -1 # Reading error

        distance = pulse_time / 58.3

        if not 2 <= distance <= 400:
            return -2 # Out of valid range

        return distance

# === Main Controller ===
class LidController:
    """Smart lid controller"""
    def __init__(self):
        disable_wifi()
        self.motion = machine.Pin(PIR_PIN, machine.Pin.IN, machine.Pin.PULL_DOWN)
        self.distance = Ultrasonic(TRIG_PIN, ECHO_PIN)
        self.servo = Servo(SERVO_PIN)

        self.active = False
        self.lid_open = False
        self.person_present = False
        self.fail_count = 0

        self.last_motion = 0
        self.last_poll = 0
        self.person_left_at = 0

        self.baseline = self.calibrate()
        self.test_servo()

    def calibrate(self):
        """Determine baseline distance to ground"""
        log("Calibrating baseline distance...", 1)
        readings = []
        attempts = 0

        while len(readings) < CALIBRATION_SAMPLES and attempts < 10:
            attempts += 1
            dist = self.distance.measure()

            if dist > 0:
                readings.append(dist)
                log(f"Calibration reading {len(readings)}: {dist:.1f} cm", 2)
            else:
                log(f"Invalid calibration reading: {dist}", 2)

            utime.sleep_ms(200)

        if readings:
            avg = sum(readings) / len(readings)
            log(f"Baseline distance: {avg:.1f} cm (±{DISTANCE_THRESHOLD} cm)", 1)
            return avg
        else:
            log("Calibration failed, using default value", 0)
            return 50.0

    def test_servo(self):
        """Test servo with initialization sequence"""
        # The first move will be a jump, subsequent moves will accelerate
        log("Servo test: Closing...", 2)
        self.servo.move(SERVO_CLOSED)
        log("Servo test: Opening...", 2)
        self.servo.move(SERVO_OPEN)
        log("Servo test: Closing...", 2)
        self.servo.move(SERVO_CLOSED)
        self.lid_open = False
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
                if utime.ticks_diff(now, self.last_poll) >= POLL_INTERVAL:
                    self.last_poll = now
                    dist = self.distance.measure()

                    if dist >= 0:
                        log(f"Distance: {dist:.1f} cm", 2)

                    if dist < 0:
                        self.fail_count += 1
                        log(f"Ultrasonic fail {self.fail_count}/{MAX_FAILURES} (code {dist})", 2)

                        if self.fail_count >= MAX_FAILURES and not self.lid_open:
                            self.open_lid("consecutive reading failures")
                            self.fail_count = 0 # Reset fail count after forcing open

                    else: # Valid reading
                        if self.fail_count > 0:
                            log(f"Valid reading obtained, resetting fail counter", 3)
                            self.fail_count = 0

                        diff = abs(dist - self.baseline)
                        if diff > DISTANCE_THRESHOLD:
                            if not self.person_present:
                                log(f"Person detected (diff: {diff:.1f} cm)", 2)
                                self.person_present = True

                            if not self.lid_open:
                                self.open_lid(f"distance changed by {diff:.1f} cm")

                            self.person_left_at = 0 # Reset timer when person is present

                        else: # No person detected (or back to baseline) 
                            if self.person_present:
                                log("Person moved away", 2)
                                self.person_present = False
                                self.person_left_at = now # Start close delay timer
                            elif self.lid_open and self.person_left_at == 0:
                                # Make sure timer is always set when no person is present and lid is open
                                log("No person detected and timer not set - setting close timer", 2)
                                self.person_left_at = now

                # Close lid logic
                if (self.lid_open and
                    not self.person_present and
                    self.person_left_at > 0 and
                    utime.ticks_diff(now, self.person_left_at) > CLOSE_DELAY):
                    log("Person absent timeout, closing lid", 1)
                    self.close_lid()
                    self.person_left_at = 0 # Reset timer

                # Inactivity timeout logic
                if (not self.lid_open and # Only go inactive if the lid is closed
                    utime.ticks_diff(now, self.last_motion) > ACTIVE_TIME):
                    log("Inactivity timeout → SLEEP", 2)
                    self.active = False
                    # Ensure lid is closed before sleeping (should already be)
                    if self.lid_open: self.close_lid()
                    self.person_present = False
                    self.person_left_at = 0
                    continue # Skip delay

                utime.sleep_ms(10) # Short delay when active

            # --- Inactive state ---
            else:
                # Check for motion even when inactive
                if self.motion.value():
                    log("Motion detected → ACTIVE", 2)
                    self.active = True
                    self.last_motion = now
                    self.fail_count = 0
                else:
                    utime.sleep_ms(100) # Longer delay when idle

# === Program Entry Point ===
def main():
    utime.sleep(CALIBRATION_DELAY)
    
    log("="*34, 2)
    log("SMART LID CONTROLLER STARTING", 2)
    log("="*34, 2)

    controller = LidController()
    controller.run()

if __name__ == "__main__":
    main()
