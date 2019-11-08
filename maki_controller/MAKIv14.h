#ifndef MAKIv14_h
#define MAKIv14_h

// see also fmorgner post on 6/3/2012 at http://forum.arduino.cc/index.php?PHPSESSID=fdkah6kc1g780cbr463mea4am5&topic=92364.msg814661#msg814661
// Added to top of /usr/share/arduino/hardware/arbotix/cores/arbotix/Arduino.h
// #define __AVR_LIBC_DEPRECATED_ENABLE__ 1

#define SERVO_COUNT 6  // MAKIv1.4 has 6 servos
#define ERROR_CHECK_MS 30000// every 30 seconds report on error values

#define USER_LED 0 // Pin 0 maps to the USER LED on the ArbotiX Robocontroller.

/* Dynamixel AX-12 enumerations corresponding to MAKIv1.4 assembly instructions  */
#define EYELID_RIGHT 1
#define EYELID_LEFT 2
#define EYE_PAN 3
#define EYE_TILT 4
#define HEAD_TILT 5 // vertical inclination of face; also known as pitch
#define HEAD_PAN 6 // horizontal inclination of face; also known as yaw
#define SERVO_NAMES_STR "LR|LL|EP|ET|HT|HP" // Every two characters, separated by |, identifies a servo with ID from 1 to 6.

#define MAX_POS_ID 1
#define MIN_POS_ID 2
#define PRESENT_POS_ID 3
#define GOAL_POS_ID 4
#define PRESENT_SPEED_ID 5
#define GOAL_SPEED_ID 6
#define PRESENT_TEMP_ID 7
#define PRESENT_LOAD_ID 8
#define TORQUE_MAX_ID 9
#define TORQUE_LIM_ID 10
#define TORQUE_ENABLE_ID 11
#define MOVING_ID 12
#define ERROR_ID 13
#define DEFAULT_POS_ID 14
#define DEFAULT_SPEED_ID 15
#define FEEDBACK_NAMES_STR "MX|MN|PP|GP|PS|GS|PT|PL|TM|TL|TS|MV|ER|DP|DS" // Identifies a feedback type from 1 to 12.

#define SET_POS_ID 1
#define SET_SPEED_ID 2
#define SET_MAX_TORQUE_ID 3
#define SET_TORQUE_LIM_ID 4
#define SET_TORQUE_ENABLE_ID 5
#define SET_NAMES_STR "GP|GS|TM|TL|TS" // 1 to 5

#define MOVEMENT_TIME_STR "IPT"

#define DEFAULT_POS 512    // Dynamixel Ax-12 value when centered; 150 degrees
#define DEFAULT_ANG 150
#define MAX_DYN_POS 1023  // Dynamixel Ax-12 value when max CCW; 300 degrees
#define MAX_DYN_ANG 300
#define MIN_DYN_POS 0     // Dynamixel Ax-12 value when min CW; 0 degrees
#define MIN_DYN_ANG 0

#define MAX_POS MAX_DYN_POS
#define MAX_ANG MAX_DYN_ANG
#define MIN_POS MIN_DYN_POS
#define MIN_ANG MIN_DYN_ANG

#define NEWER_FUNKY_MAKI

#ifdef NEWER_FUNKY_MAKI
// emperically determined reasonable values
int min_servo_pos[] = {484,   // EYELID_RIGHT
                       361,   // EYELID_LEFT
                       460,   // EYE_PAN
                       425,  //390,   // EYE_TILT
                       446,   // HEAD_TILT
                       189    //300   // HEAD_PAN
                      };
                      
int max_servo_pos[] = {666,   // EYELID_RIGHT
                       535,   // EYELID_LEFT
                       578,   // EYE_PAN
                       610,   // EYE_TILT
                       586,   // HEAD_TILT
                       615    //696   // HEAD_PAN
                      };
int default_servo_pos[] = {525,   // EYELID_RIGHT
                           500,   // EYELID_LEFT
                           DEFAULT_POS,   // EYE_PAN
                           DEFAULT_POS,   // EYE_TILT
                           500,   //490,  // HEAD_TILT
                           410   // HEAD_PAN
                      };

                        
#else
// emperically determined reasonable values
int min_servo_pos[] = {484,   // EYELID_RIGHT
                       361,   // EYELID_LEFT
                       460,   // EYE_PAN
                       425,  //390,   // EYE_TILT
                       446,   // HEAD_TILT
                       256    //300   // HEAD_PAN
                      };
                      
int max_servo_pos[] = {666,   // EYELID_RIGHT
                       535,   // EYELID_LEFT
                       578,   // EYE_PAN
                       610,   // EYE_TILT
                       586,   // HEAD_TILT
                       768    //696   // HEAD_PAN
                      };
int default_servo_pos[] = {525,   // EYELID_RIGHT
                           500,   // EYELID_LEFT
                           DEFAULT_POS,   // EYE_PAN
                           DEFAULT_POS,   // EYE_TILT
                           DEFAULT_POS,   //490,  // HEAD_TILT
                           DEFAULT_POS   // HEAD_PAN
                          };                
#endif
                      
                                                            

int default_goal_speed[] = {100,  // EYELID_RIGHT
                            100,  // EYELID_LEFT
                            100,  // EYE_PAN
                            200,  // EYE_TILT
                            15,    // HEAD_TILT
                            51     // HEAD_PAN
                           };

#endif

