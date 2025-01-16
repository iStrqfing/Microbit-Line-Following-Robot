#include "MicroBit.h"

MicroBit uBit;

// Motor Address
#define MOTOR_ADDR 0x00

// Motor commands
#define MOTOR_LEFT 0x00
#define MOTOR_RIGHT 0x02
#define FORWARD 0x00
#define BACKWARD 0x01

// Pins
#define ULT_TRIGGER P1
#define ULT_ECHO P2
#define LED_LEFT P8
#define LED_RIGHT P12
#define GREYSCALE_LEFT P13
#define GREYSCALE_RIGHT P14

// #define DEBUG

// Flag for if there is an obstacle in the way
int obstacleDetected = 0;

// Method ajusted from robot_motors.cpp example file
static void setMotor(uint8_t motor, uint8_t direction, uint8_t speed)
{
    // Declare buffer and values
    uint8_t buf[3];
    buf[0] = motor;
    buf[1] = direction;
    buf[2] = speed;
    // Write buffer to address 0x20 with buffer containing three values
    // device address is 0x10 but must be left shifted for Micro:bit libraries.
    uBit.i2c.write(0x20, buf, 3);
}


// Update the motors when driven by an event (e.g. sensors changing)
void updateMotors(codal::Event e)
{
    // If there is an obstacle in the way, wait a little bit and return early
    if (obstacleDetected)
    {
        uBit.sleep(10);
        return;
    }

    // Read the two greyscale sensors
    int leftGreyscale = uBit.io.GREYSCALE_LEFT.getDigitalValue();
    int rightGreyscale = uBit.io.GREYSCALE_RIGHT.getDigitalValue();

    // Debugging purposes
    #ifdef DEBUG
        uBit.serial.printf("leftGreyscale: %d\n\r", leftGreyscale);
        uBit.serial.printf("rightGreyscale: %d\n\r", rightGreyscale);
    #endif

    const int NORMAL_SPEED = 25; // 50 for consistancy, 200 for fun
    const int TURN_SPEED = 25;   // 30 for consistancy, 200 for fun

    uBit.io.LED_LEFT.setDigitalValue(leftGreyscale);   // Left LED
    uBit.io.LED_RIGHT.setDigitalValue(rightGreyscale); // Right LED

    // Only left sensor on the line, go straight
    if (leftGreyscale == 0 && rightGreyscale == 1)
    {   
        setMotor(MOTOR_LEFT, FORWARD, NORMAL_SPEED);
        setMotor(MOTOR_RIGHT, FORWARD, NORMAL_SPEED);
    } else if (leftGreyscale == 1 && rightGreyscale == 0) { // Right sensor on the line without the left sensor, turn right to keep left sensor only on the line
        setMotor(MOTOR_LEFT, FORWARD, NORMAL_SPEED);
        setMotor(MOTOR_RIGHT, FORWARD, NORMAL_SPEED);

        // Sleep so we go straight through the intersection, otherwise turning may occur ////////////////////////////////////
        uBit.sleep(12500/NORMAL_SPEED);
    } else if (leftGreyscale == 1 && rightGreyscale == 1) { // Both sensors are off the line, turn right so we only keep one sensor on the line
        setMotor(MOTOR_LEFT, BACKWARD, TURN_SPEED);
        setMotor(MOTOR_RIGHT, FORWARD, TURN_SPEED);
    } else { // Both sensors are on the line (0), turn left to get the one sensor back on the line
        setMotor(MOTOR_LEFT, FORWARD, TURN_SPEED);
        setMotor(MOTOR_RIGHT, BACKWARD, TURN_SPEED);
    }
}

// Handles the distance value from the ult sensor
void handleUltValue(int distance)
{
    const int MIN_DISTANCE = 10; // Minimum distance an obstacle can be (cm)

    // If the distance of an object is too close (10cm or less, stop the motors)
    if (distance < MIN_DISTANCE) {
        // Set global obstacle detection flag to 1
        obstacleDetected = 1;

        // Stop the motors
        setMotor(MOTOR_LEFT, FORWARD, 0);
        setMotor(MOTOR_RIGHT, FORWARD, 0);
    } else if (obstacleDetected) {
        // Reset obstacle detected flag to 0
        obstacleDetected = 0;

        // Start motors again to trigger sensor events
        setMotor(MOTOR_LEFT, FORWARD, 20);
        setMotor(MOTOR_RIGHT, FORWARD, 20);
    }
}

// Time how long the pulse from the trigger took to come back on the echo pin
int timePulse() {
    MicroBitPin pin = uBit.io.ULT_ECHO; // The pin we are measuring

    const int MAX_DISTANCE = 500;

    uint64_t start = system_timer_current_time_us(); // Get the start time
    int timeout = MAX_DISTANCE * 58; // *58 to get the time it would take the pulse for the given distance = our timeout
    uint64_t end = start + timeout; // Figure out the time we will wait before giving up
    uint64_t pulseStart = 0;
    uint64_t pulseEnd = 0;

    int state = 1; // The state we are detecting (1 = High, 0 = Low)

    // Wait for the pulse to start
    while (pin.getDigitalValue() != state)
    {
        if (system_timer_current_time_us() > end) // Timeout waiting for pulse
        {
            return MAX_DISTANCE * 58; 
        }
    }

    pulseStart = system_timer_current_time_us(); // Record the start of the pulse

    // Wait for the pulse to end
    while (pin.getDigitalValue() == state)
    {
        if (system_timer_current_time_us() > end) // Timeout during pulse
        {
            return MAX_DISTANCE * 58; 
        }
    }
    pulseEnd = system_timer_current_time_us(); // Record the end of the pulse

    return (int)(pulseEnd - pulseStart); // Return the time the pulse took
}

// Funtion to continuously read distance data from the ultrasonic sensor
void readUltSensor()
{
    while (1)
    {
        int distance = 0;

        // Send a pulse on the trigger pin on the ultrasonic sensor
        uBit.io.ULT_TRIGGER.setDigitalValue(1);
        uBit.sleep(10);
        uBit.io.ULT_TRIGGER.setDigitalValue(0);

        // Read the echo pin
        int pulseDuration = timePulse(); 

        // Calculate distance based on the speed of sound (in cm)
        distance = pulseDuration / 58; // Distance in cm

        #ifdef DEBUG
            // Print distance to serial monitor for debugging
            uBit.serial.printf("Ult Distance: %d cm\n", distance);
        #endif  

        // Handle the returned distance
        handleUltValue(distance);

        uBit.sleep(500);
    } 
}

// Main function to initialise program and keep it running
int main()
{
    // Initialise the uBit
    uBit.init();
    uBit.display.setDisplayMode(DISPLAY_MODE_GREYSCALE);

    // Debugging purposes, shows start of program
    #ifdef DEBUG
        uBit.serial.printf("Running program\n\r");
    #endif

    // Make event listeners for the grey scale sensors on pin edges
    uBit.io.P13.eventOn(MICROBIT_PIN_EVENT_ON_EDGE);
    uBit.io.P14.eventOn(MICROBIT_PIN_EVENT_ON_EDGE);

    uBit.messageBus.listen(MICROBIT_ID_IO_P13, MICROBIT_PIN_EVT_RISE, updateMotors);
    uBit.messageBus.listen(MICROBIT_ID_IO_P13, MICROBIT_PIN_EVT_FALL, updateMotors);
    uBit.messageBus.listen(MICROBIT_ID_IO_P14, MICROBIT_PIN_EVT_RISE, updateMotors);
    uBit.messageBus.listen(MICROBIT_ID_IO_P14, MICROBIT_PIN_EVT_FALL, updateMotors);

    // Create a fiber for the ultra sonic sensor
    create_fiber(readUltSensor);

    release_fiber();
}