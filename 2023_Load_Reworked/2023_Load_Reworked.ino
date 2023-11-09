#include <Wire.h>
#include <Adafruit_MCP4725.h>
#include <Adafruit_INA260.h>
#include <PA12.h>
#include "filter.h"

#define ID_NUM 0
#define Linear_Actuator_Enable 16
#define Linear_Actuator_Tx 1
#define Linear_Actuator_Rx 0
#define RPM_Pin 29
#define PCC_Relay_Pin 33
#define EStop_Pin 11
#define PCC_Disconnect_Pin 30

// State
typedef enum {
    Startup,
    EStop,
    PCC_Disconnect,
    Safety_Restart,
    Optimize,
    Regulate,
} STATES;
STATES state = Startup;

// Timers
unsigned int timer_coms = 0;
unsigned int timer_optimize = 0;

// RPM
struct Filter* rpm_filter = CreateFilter(10, 14);

// Linear Actuator
PA12 myServo(&Serial1, 16, 1);

// INA260
Adafruit_INA260 ina260 = Adafruit_INA260();

// DAC
Adafruit_MCP4725 dac;

// Theta Constants
const int theta_min = 0;
const int theta_max = 4095;
const int theta_break = 3186;
const int theta_cut_in = 992;

// Optimization Variables
int theta_current = 0;
int theta_previous = 0;
double rpm_last = 0;
int theta_step_size = 50;
const float theta_multiplier_flip = 0.25;
const float theta_multiplier_else = 1.5;


void setup () {
    Serial.begin(9600);
    while (!Serial) { // Wait for serial to be ready
        delay(10);
    }
    Serial.println("Starting...");

    // Init INA260
    ina260.begin();
    Serial.println("INA260 ready");

    // Init DAC
    dac.begin(0x66);
    Serial.println("DAC ready");

    //Linear Actuator
    myServo.begin(32);
    myServo.movingSpeed(ID_NUM, 750);
    Serial.println("Linear actuator ready");

    // Set Pin Modes
    pinMode(Linear_Actuator_Tx, OUTPUT);
    pinMode(Linear_Actuator_Rx, INPUT);
    pinMode(Linear_Actuator_Enable, OUTPUT);
    pinMode(RPM_Pin, INPUT);
    pinMode(PCC_Relay_Pin, OUTPUT);
    pinMode(EStop_Pin, INPUT);
    pinMode(PCC_Disconnect_Pin, INPUT);

    // Start with PCC on
    digitalWrite(PCC_Relay_Pin, HIGH);
    delay(500);
    Serial.println("PCC on");

    // Set to cut-in pitch
    myServo.goalPosition(ID_NUM, theta_cut_in);
    while (myServo.Moving(ID_NUM)) {
        delay(10); // wait for linear actuator to finish moving
    }
    Serial.println("Linear actuator at cut-in pitch");

    // Set DAC Voltage for cut-in
    dac.setVoltage(0, false);
    Serial.println("DAC set to cut-in voltage");

    // Start RPM Tracking
    attachInterrupt(digitalPinToInterrupt(RPM_Pin), RPM_Interrupt, RISING);

    // Turn PCC off
    digitalWrite(PCC_Relay_Pin, LOW);
    Serial.println("PCC off");

    // Init Timers
    unsigned int time = millis();
    timer_coms = time;
    timer_optimize = time;
}

void loop () {
    // Check E-Stop
    if (digitalRead(EStop_Pin) == HIGH) {
        state = EStop;
    }

    // TODO: check PCC disconnect
    if (Serial.available() > 0) {
        uint8_t cmd = Serial.read();

        switch (cmd)
        {
            case 't':
                //theta = Serial.parseFloat();
                break;

            case 'o':
                //load_Optimize_Enable = !load_Optimize_Enable;
                break;

            case 'p':
                if (digitalRead(PCC_Relay_Pin) == HIGH) {
                    digitalWrite(PCC_Relay_Pin, LOW);
                    Serial.println("PCC off");
                } else {
                    digitalWrite(PCC_Relay_Pin, HIGH);
                    Serial.println("PCC on");
                }
                //PCC_Relay = !PCC_Relay;
                //digitalWrite(24, PCC_Relay);
                break;

            case 'v':
                int temp_DAC_voltage = Serial.parseInt();
                Serial.print("Setting DAC to: ");
                Serial.println(temp_DAC_voltage);
                dac.setVoltage(temp_DAC_voltage, false);
                break;

            default:
                Serial.println("Command not recognized");
                break;
        }
    }

    // Read PC coms every 250ms
    if (millis() - timer_coms > 500) {
        timer_coms = millis();
        // TODO: PC coms
        Serial.print("State: ");
        Serial.println(state);
        Serial.print("RPM: ");
        Serial.println(GetRpmBuffered(rpm_filter));
        Serial.print("Load Power: ");
        Serial.println(ina260.readPower());
    }

    // State Machine
    switch (state) {
        case Startup:
            if (GetRpmBuffered(rpm_filter) > 500.0) {
                state = Optimize;
                theta_step_size = 50;
                dac.setVoltage(16, false);
            }
            break;
        case EStop:
            digitalWrite(PCC_Relay_Pin, HIGH);
            myServo.goalPosition(ID_NUM, theta_break);
            if (digitalRead(EStop_Pin) == LOW) {
                state = Safety_Restart;
            }
            break;
        case PCC_Disconnect:
            // TODO
            break;
        case Safety_Restart:
            myServo.goalPosition(ID_NUM, theta_cut_in);
            state = Startup;
            break;
        case Optimize:
            if (GetRpmBuffered(rpm_filter) > 3000.0) {
                state = Regulate;
                break;
            }

            if (GetRpmBuffered(rpm_filter) < 400.0) {
                state = Startup;
                break;
            }

            // Set optimal pitch
            if (timer_optimize - millis() > 250) {
                timer_optimize = millis();

                double rpm_cur = GetRpmBuffered(rpm_filter);
                if (rpm_last > rpm_cur) {
                    // if rpm is decreasing, flip direction of change
                    theta_step_size *= (-1 * theta_multiplier_flip);
                } else {
                    theta_step_size *= theta_multiplier_else;
                }

                rpm_last = rpm_cur;

                theta_current = myServo.presentPosition(ID_NUM);
                myServo.goalPosition(ID_NUM, theta_current + theta_step_size);
            }

            break;
        case Regulate:
            // TODO: regulate power/rpm with pitch
            myServo.goalPosition(ID_NUM, 1000);

            break;
        default:
            Serial.println("Error: Invalid State");
            break;
    }
}

void RPM_Interrupt () {
    Insert(rpm_filter, (int)micros());
}