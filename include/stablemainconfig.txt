// =====================================================================
//  LFR configuration - everything you may need to change is in here
// =====================================================================
#pragma once
#include <Arduino.h>

// ---------------- Pins (from pin connection diagram v2) ---------------
#define USE_MUX          0     // 1 = 74HC4051 (Z->A0, S0->D2, S1->D4, S2->D7)
                               // 0 = array wired straight to A0..A7
#define PIN_MUX_Z        A0
#define PIN_MUX_S0       2
#define PIN_MUX_S1       4
#define PIN_MUX_S2       7

#define PIN_L_FWD        11    // L298N IN1  (left motor)
#define PIN_L_REV        10    // L298N IN2
#define PIN_R_FWD        9     // L298N IN3  (right motor)
#define PIN_R_REV        6     // L298N IN4
#define PIN_LED          13

// ---------------- Direction fixes (no rewiring needed) ----------------
#define LEFT_MOTOR_INVERT   0  // 1 if the left wheel spins backwards
#define RIGHT_MOTOR_INVERT  0  // 1 if the right wheel spins backwards
#define SWAP_MOTORS         0  // 1 if left/right motors are exchanged
#define SENSORS_REVERSED    0  // 1 if index 0 is the RIGHT-most sensor
                               //   (check with the 's' serial command)
#define SENSOR_DISABLE_MASK 0x00  // ignore faulty sensors: bit0 = S1 ... bit7 = S8
                               //   e.g. 0x40 ignores S7, 0x41 ignores S1 and S7
#define LINE_READS_HIGH     1  // 1 = line gives the HIGHER ADC value
                               //   (normal for TCRT5000, black on white)

// ---------------- Track rules ------------------------------------------
#define AUTO_INVERT         1  // 1 = switch automatically between a black line
                               //   on white and a white line on black
#define POLARITY_SAMPLES    4  // consecutive readings needed to switch
#define JUNCTION_TURN       1  // where the line ends at a T: +1 = turn right,
                               //   -1 = turn left. At a crossing where the line
                               //   continues ahead the robot goes straight.
#define EDGE_MEMORY_MS      150 // how long a side branch is remembered
#define EDGE_CLEAR_SAMPLES   4  // narrow-line readings that cancel that memory

// ---------------- Robot geometry (your measurements) ------------------
#define SENSOR_PITCH_MM     15.0f   // sensor to sensor
#define SENSOR_LEVER_MM     147.0f  // sensor row to axle
#define TRACK_WIDTH_MM      170.0f  // tyre centre to tyre centre (8.5 + 8.5)
#define WHEEL_DIA_MM        45.0f
#define NUM_SENSORS         8
// Half span of the array = 3.5 * 15 = 52.5 mm  -> error range +-52.5 mm

// ---------------- Wheel matching ---------------------------------------
// A tank turn only spins on the spot if both wheels turn at the same speed.
// Run the wheel test (send 'w' in the serial monitor) and lower the
// percentage of whichever wheel is faster, forward and reverse separately.
#define LEFT_FWD_PCT        100     // 0..100 % of commanded power
#define LEFT_REV_PCT        100
#define RIGHT_FWD_PCT       100
#define RIGHT_REV_PCT       100
#define MOTOR_MIN_PWM       70      // any non-zero command is raised to at least
                                    //   this, so a wheel never stalls and drags
                                    //   the turn into a curve

// ---------------- PID (error is in millimetres) -----------------------
#define KP                  2.0f    // PWM per mm of error
#define KI                  0.0f    // PWM per mm*s      (usually 0)
#define KD                  0.08f   // PWM per mm/s
#define I_LIMIT             60.0f   // anti wind-up (PWM)
#define D_FILTER            0.3f    // 0..1, lower = smoother derivative

// 1200 rpm motors + 45 mm wheels: about 2.8 m/s with no load, so the robot
// covers roughly 4-6 mm per control loop. All the times below (gap, finish,
// junction memory) are set for that speed. If you slow the robot right down,
// lengthen GAP_MS and FULL_BAR_MS in proportion.

// ---------------- Speeds (PWM 0..255) ---------------------------------
#define BASE_SPEED          110     // straight-line speed
#define CURVE_SPEED         80      // speed when the line is at the array edge
#define MAX_SPEED           255
#define MAX_REVERSE         160     // inner wheel may reverse (needed: with a
                                    // 14.7 cm lever a 15 cm radius curve makes
                                    // the inner wheel run backwards)
#define TURN_SPEED          130     // pivot speed when the line left the array
#define GAP_SPEED           100     // straight speed while crossing a gap

#define PIVOT_ERR_MM        40.0f   // line this far from centre -> one wheel
                                    //   forward, the other backward (tank turn).
                                    //   Smaller = counter-rotates sooner;
                                    //   60 = never, PID curves only

// ---------------- Line-lost behaviour ---------------------------------
#define ON_LINE_LEVEL       500     // 0..1000, sensor counts as "on the line"
#define GROUP_LEVEL         250     // 0..1000, sensor counts as part of a blob
#define JUNCTION_WIDTH      5       // blob this many sensors wide = crossing /
                                    //   corner bar: hold the heading
#define GAP_CENTER_MM       16.0f   // line lost while within this of centre
                                    //   -> treat as a gap and go straight
#define GAP_MS              250     // max straight time for the 10 cm gap
#define LOST_STOP_MS        1500    // nothing found for this long -> stop

// Finish: stop when ALL sensors see black for FULL_BAR_MS (filled square).
// A 3 cm crossing line passes much faster than this, so it is ignored.
#define STOP_ON_FULL_BAR    1
#define FULL_BAR_MS         120
#define FINISH_ARM_MS       1000    // ignored for this long after the start
#define FINISH_EXTRA_MS     100     // keep rolling this long, then stop

// ---------------- Start-up sequence (no extra button needed) ----------
#define START_DELAY_MS      1500    // hands-off time after power on / reset
#define CAL_SPEED           150     // pivot speed during calibration
#define CAL_TIMEOUT_MS      2500    // max time for each calibration turn
#define CAL_OVERRUN_MS      300     // max extra turn while the edge sensor
                                    //   passes over the line and off the far side
#define CAL_SEEN_RANGE      300     // ADC swing that counts as "saw the line"
#define CAL_MIN_RANGE       150     // min (max-min) ADC counts per sensor
#define GO_DELAY_MS         1500    // pause after calibration, then go

#define LOOP_US             2000    // control period (500 Hz)