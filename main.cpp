#include <iostream>		// Include all needed libraries here
#include <pigpio.h>

using namespace std;		// No need to keep using “std”

#include "ctre/phoenix6/TalonFX.hpp"
#include "UpLiftBase.hpp"
#include "Joystick.hpp"
#include "httplib.h"

using namespace ctre::phoenix6;

/**
 * This is the main uplift class
 */
class UpLift : public UpLiftBase {
private:
    /* This can be a CANivore name, CANivore serial number,*
     * SocketCAN interface, or "*" to select any CANivore. */
    static constexpr char const *CANBUS_NAME = "can0";

    /* devices */
    hardware::TalonFX leftLeader{0, CANBUS_NAME};
   // hardware::TalonFX leftFollower{1, CANBUS_NAME};
    hardware::TalonFX rightLeader{2, CANBUS_NAME};
  //  hardware::TalonFX rightFollower{3, CANBUS_NAME};

    /* control requests */
    controls::DutyCycleOut leftOut{0};
    controls::DutyCycleOut rightOut{0};





    /* joystick */
    Joystick joy{0};
    float motorspeed;
    bool buttonpressed = false;

public:
    /* main uplift interface */
    void UpLiftInit() override;
    void UpLiftPeriodic() override;

    bool IsEnabled() override;
    void EnabledInit() override;
    void EnabledPeriodic() override;

    void DisabledInit() override;
    void DisabledPeriodic() override;

};

/**
 * Runs once at code initialization.
 */
void UpLift::UpLiftInit()
{
    gpioInitialise();
    configs::TalonFXConfiguration fx_cfg{};


    fx_cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    leftLeader.GetConfigurator().Apply(fx_cfg);

  
    fx_cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
    rightLeader.GetConfigurator().Apply(fx_cfg);

    motorspeed = 0;

    /* set follower motors to follow leaders; do NOT oppose the leaders' inverts */
   // leftFollower.SetControl(controls::Follower{leftLeader.GetDeviceID(), false});
  //  rightFollower.SetControl(controls::Follower{rightLeader.GetDeviceID(), false});
}

/**
 * Runs periodically during program execution.
 */
void UpLift::UpLiftPeriodic()
{
    /* periodically check that the joystick is still good */
    joy.Periodic();
}

/**
 * Returns whether upLift should be enabled.
 */
bool UpLift::IsEnabled()
{
    /* enable while joystick is an Xbox controller (6 axes),
     * and we are holding the right bumper */
    if (joy.GetNumAxes() < 6) return false;
    return joy.GetButton(5); // SDL_CONTROLLER_BUTTON_RIGHTSHOULDER
}

/**
 * Runs when transitioning from disabled to enabled.
 */
void UpLift::EnabledInit() {}

/**
 * Runs periodically while enabled.
 */
void UpLift::EnabledPeriodic()
{
     gpioInitialise();

       if(gpioRead(22) == 0)
    {
        leftOut.Output = 0.1;

    } else if (gpioRead(23) == 0) {
        leftOut.Output = -0.1;
    } else if (buttonpressed = true) {
        buttonpressed = false;
       leftOut.Output = 0.0;
    }
        leftLeader.SetControl(leftOut);
        rightLeader.SetControl(leftOut);

}



/**
 * Runs when transitioning from enabled to disabled,
 * including after  startup.
 */
void UpLift::DisabledInit() {}

/**
 * Runs periodically while disabled.
 */
void UpLift::DisabledPeriodic()
{
    leftLeader.SetControl(controls::NeutralOut{});
    rightLeader.SetControl(controls::NeutralOut{});
}



/* ------ main function ------ */
int main()
{


    gpioInitialise();
    gpioSetMode(22, PI_INPUT);
    gpioSetMode(23, PI_INPUT);
   gpioSetPullUpDown(22, PI_PUD_UP);
   gpioSetPullUpDown(23, PI_PUD_UP);



    /* create and run uplift */
    UpLift uplift{};
    // uplift.SetLoopTime(20_ms); // optionally change loop time for periodic calls

    uplift.Run();
       httplib::Server svr;
   svr.Get("/go", [](const httplib::Request &, httplib::Response &res)
           { 
            res.set_content("Hello World!", "text/plain"); });

    svr.listen("0.0.0.0", 8080);
    return true;


}
