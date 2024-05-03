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
    hardware::TalonFX driverCabLeader{0, CANBUS_NAME};
  //  hardware::TalonFX passengerCabFollower{1, CANBUS_NAME};
    hardware::TalonFX driverTailLeader{2, CANBUS_NAME};
 //   hardware::TalonFX passengerTailFollower{3, CANBUS_NAME};

    /* control requests */
   // controls::DutyCycleOut cabOut{0};
   // controls::DutyCycleOut tailOut{0};
   ctre::phoenix6::controls::MotionMagicVoltage m_mmReq{0_tr};


    /* joystick */
    Joystick joy{0};
  //  float motorspeed;
    bool buttonpressed = false;
    double maxlift = 50;

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

  configs::TalonFXConfiguration cfg{};

  /* Configure current limits */
  configs::MotionMagicConfigs &mm = cfg.MotionMagic;
 // MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  mm.MotionMagicCruiseVelocity = 70; // 5 rotations per second cruise
  mm.MotionMagicAcceleration = 25; // Set to 250 to match what we were using on elevator
  // Take approximately 0.2 seconds to reach max accel 
  mm.MotionMagicJerk = 0;

  configs::Slot0Configs &slot0 = cfg.Slot0;
  slot0.kP = 4.9;
  slot0.kI = 0;
  slot0.kD = 0.0078125;
  slot0.kV = 0.009375;
  slot0.kS = 0.02; // Approximately 0.25V to get the mechanism moving

  configs::FeedbackConfigs &fdb = cfg.Feedback;
  fdb.SensorToMechanismRatio = 1.0;

  cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for(int i = 0; i < 5; ++i) {
    status = driverCabLeader.GetConfigurator().Apply(cfg);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }
/*
  cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
  status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for(int i = 0; i < 5; ++i) {
    status = passengerCabFollower.GetConfigurator().Apply(cfg);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }
*/
  cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
  status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for(int i = 0; i < 5; ++i) {
    status = driverTailLeader.GetConfigurator().Apply(cfg);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }
/*
  cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
  status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for(int i = 0; i < 5; ++i) {
    status = passengerTailFollower.GetConfigurator().Apply(cfg);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }
  */

    /* set follower motors to follow leaders; do NOT oppose the leaders' inverts */
 //   passengerCabFollower.SetControl(controls::Follower{driverCabLeader.GetDeviceID(), false});
 //   passengerTailFollower.SetControl(controls::Follower{driverTailLeader.GetDeviceID(), false});
   

    gpioInitialise();
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

    if (gpioRead(22) == 0)  // all up
    {
        driverCabLeader.SetControl(m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
        driverTailLeader.SetControl(m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
        buttonpressed = true;
    }
    else if (gpioRead(23) == 0) // all down
    {
        driverCabLeader.SetControl(m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
        driverTailLeader.SetControl(m_mmReq.WithPosition(0.0* 1_tr).WithSlot(0));
        buttonpressed = true;
    }
    else if (gpioRead(5) == 0)  // twist up
    {
        driverCabLeader.SetControl(m_mmReq.WithPosition(0.0* 1_tr).WithSlot(0));
        driverTailLeader.SetControl(m_mmReq.WithPosition(maxlift* 1_tr).WithSlot(0));
        buttonpressed = true;
    }
    else if (gpioRead(6) == 0) // twist down
    {
        driverCabLeader.SetControl(m_mmReq.WithPosition(maxlift* 1_tr).WithSlot(0));
        driverTailLeader.SetControl(m_mmReq.WithPosition(0.0* 1_tr).WithSlot(0));
        buttonpressed = true;
    }
    else if (buttonpressed = true)
    {
        buttonpressed = false;
        driverCabLeader.SetControl(controls::NeutralOut{});
        driverTailLeader.SetControl(controls::NeutralOut{});
    }

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
    driverCabLeader.SetControl(controls::NeutralOut{});
    driverTailLeader.SetControl(controls::NeutralOut{});
}

/* ------ main function ------ */
int main()
{

    gpioInitialise();
    gpioSetMode(22, PI_INPUT);
    gpioSetMode(23, PI_INPUT);
    gpioSetPullUpDown(22, PI_PUD_UP);
    gpioSetPullUpDown(23, PI_PUD_UP);


    gpioSetMode(5, PI_INPUT);
    gpioSetMode(6, PI_INPUT);
    gpioSetPullUpDown(5, PI_PUD_UP);
    gpioSetPullUpDown(6, PI_PUD_UP);
    /* create and run uplift */
    UpLift uplift{};
    // uplift.SetLoopTime(20_ms); // optionally change loop time for periodic calls

    uplift.Run();
    httplib::Server svr;
    svr.Get("/go", [](const httplib::Request &, httplib::Response &res)
            { res.set_content("Hello World!", "text/plain"); });

    svr.listen("0.0.0.0", 8080);
    return true;
}
