#include "MicroBit.h"

MicroBit uBit;

// Motor Address
#define MOTOR_ADDR 0x00

// Motor commands
#define MOTOR_LEFT 0x00
#define MOTOR_RIGHT 0x02
#define FORWARD 0x00
#define BACKWARD 0x01

//Pins 
#define LED_LEFT P8
#define LED_RIGHT P12
#define GREYSCALE_LEFT P13
#define GREYSCALE_RIGHT P14

// Define for debugging otherwise comment out
#define DEBUG

//Sensor variables 
int leftGreyscale = 0;
int rightGreyscale = 0;
int searchDirection = 0; // Determines which way robot turns to search for the line

// Method ajusted from robot_motors.cpp example file
static void setMotor(uint8_t motor, uint8_t direction, uint8_t speed)
{
    // Declare buffer and values
    uint8_t buf[3];
    buf[0] = motor;
    buf[1] = direction;
    buf[2] = speed;
    // Write buffer to address 0x20 with buffer containing three values
    uBit.i2c.write(0x20, buf, 3); // Device address is 0x10 but must be left shifted for Micro:bit libraries.

    // Debugging purposes
    #ifdef DEBUG
        // ret = uBit.i2c.write(0x20, buf, 3);
        // uBit.display.scroll(ret);
        // uBit.serial.printf("%c", ret);
    #endif
}

// Update the motors when driven by an event (e.g. sensors changing)
void updateMotors(codal::Event e)
{
    const int NORMAL_SPEED = 200; //50 for consistancy, 200 for fun
    const int TURN_SPEED = 200; // 30 for consistancy, 200 for fun

    // Read the two greyscale sensors
    int leftGreyscale = uBit.io.GREYSCALE_LEFT.getDigitalValue();
    int rightGreyscale = uBit.io.GREYSCALE_RIGHT.getDigitalValue();

    // Turn the corresponding LED's on
    uBit.io.LED_LEFT.setDigitalValue(leftGreyscale);   // Left LED
    uBit.io.LED_RIGHT.setDigitalValue(rightGreyscale); // Right LED

    // Debugging purposes
    #ifdef DEBUG
        uBit.serial.printf("leftGreyscale: %d\n\r", leftGreyscale);
        uBit.serial.printf("rightGreyscale: %d\n\r", rightGreyscale);
    #endif

    // Only left sensor on the line, go straight
    if (leftGreyscale == 0 && rightGreyscale == 1) {
        setMotor(MOTOR_LEFT, FORWARD, NORMAL_SPEED);
        setMotor(MOTOR_RIGHT, FORWARD, NORMAL_SPEED);
    } else if (leftGreyscale == 1 && rightGreyscale == 0) { // Right sensor on the line without the left sensor, turn right to keep left sensor only on the line
        setMotor(MOTOR_LEFT, FORWARD, NORMAL_SPEED);
        setMotor(MOTOR_RIGHT, BACKWARD, TURN_SPEED);
    } else if (leftGreyscale == 1 && rightGreyscale == 1) { // Both sensors are on the line, turn right so we only keep left sensor on the line
        setMotor(MOTOR_LEFT, BACKWARD, TURN_SPEED);
        setMotor(MOTOR_RIGHT, FORWARD, TURN_SPEED);
    } else {
        // Both sensors are off the line (0), turn left to get the left sensor back on the line
        setMotor(MOTOR_LEFT, FORWARD, TURN_SPEED);
        setMotor(MOTOR_RIGHT, BACKWARD, TURN_SPEED);
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

    // Loop to keep program running
    while (1)
    {
        uBit.sleep(10);
    }
}