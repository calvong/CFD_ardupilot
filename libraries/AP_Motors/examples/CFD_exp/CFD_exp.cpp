/*
 *  Example of AP_Motors library.
 *  Code by Randy Mackay. DIYDrones.com
 */

// Libraries
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include <AP_Motors/AP_Motors.h>
#include <AP_Notify/AP_Notify.h>
#include <AP_GPS/AP_GPS.h>
#include <DataFlash/DataFlash.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Baro/AP_Baro.h>
#include <Filter/Filter.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_Declination/AP_Declination.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_Mission/AP_Mission.h>
#include <StorageManager/StorageManager.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_BattMonitor/AP_BattMonitor.h>
#include <AP_RangeFinder/AP_RangeFinder.h>
#include <AP_Scheduler/AP_Scheduler.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// declare functions
void setup();
void loop();
void run_motor(int pwm);



#define NUM_OUTPUTS     4   // set to 6 for hexacopter, 8 for octacopter and heli

SRV_Channels srvs;

// uncomment the row below depending upon what frame you are using
//AP_MotorsTri	motors(400);
AP_MotorsMatrix   motors(400);
//AP_MotorsHeli_Single motors(rc7, rsc, h1, h2, h3, h4, 400);
//AP_MotorsSingle motors(400);
//AP_MotorsCoax motors(400);

AP_BattMonitor _battmonitor{0, nullptr, nullptr};

// setup
void setup()
{
    hal.console->printf("CFD experiment\n");

    // motor initialisation
    motors.set_update_rate(490);
    motors.init(AP_Motors::MOTOR_FRAME_QUAD, AP_Motors::MOTOR_FRAME_TYPE_X);

    motors.output_min();

    // setup radio
    SRV_Channels::srv_channel(2)->set_output_min(1000);
    SRV_Channels::srv_channel(2)->set_output_max(2000);


    hal.scheduler->delay(1000);
}

// loop
void loop()
{
    int16_t value;

    // wait for user to enter something
    while( !hal.console->available() ) {
        hal.scheduler->delay(20);
    }

    // get character from user
    value = hal.console->read();

    //int pwm = rc().channel(CH_3)->get_radio_in();
    int pwm = 1100;


    // start running the motors
    if (value == 'r' || value == 'R') {
        run_motor(pwm);
    }

}

// run motor
void run_motor(int pwm)
{
    hal.console->printf("Running motors ......\n");
    motors.armed(true);
    for (int8_t i=1; i <= 4; i++)
    {
        motors.output_test_seq(i, pwm);
        //hal.scheduler->delay(2000);
    }
    motors.armed(false);

}

AP_HAL_MAIN();
