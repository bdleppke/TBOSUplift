

#define _CRT_SECURE_NO_DEPRECATE

#include <cpprest/http_listener.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <thread>
#include <chrono>
#include <ctime>

// cpprest provides macros for all streams but std::clog in basic_types.h
#ifdef _UTF16_STRINGS
// On Windows, all strings are wide
#define uclog std::wclog
#else
// On POSIX platforms, all strings are narrow
#define uclog std::clog
#endif // endif _UTF16_STRINGS

using namespace std;
using namespace web::http::experimental::listener;
using namespace web::http;
using namespace web;

#include <iostream> // Include all needed libraries here
#include <pigpio.h>
#include <cmath>
#include <limits>

using namespace std; // No need to keep using “std”
#include "ctre/phoenixpro/sensors/CANCoder.hpp"
#include "ctre/phoenixpro/sensors/CANCoderConfiguration.hpp"
#include "ctre/phoenixpro/signals/AbsoluteSensorRangeValue.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "UpLiftBase.hpp"
// #include "Joystick.hpp"

#include <ctre/phoenix6/CANcoder.hpp>

using namespace ctre::phoenix6;

/**
 * This is the main uplift class
 */
class UpLift : public UpLiftBase
{
private:
  /* This can be a CANivore name, CANivore serial number,*
   * SocketCAN interface, or "*" to select any CANivore. */
  static constexpr char const *CANBUS_NAME = "can0";

  /* devices */
  hardware::TalonFX driverCabLeader{0, CANBUS_NAME};
  hardware::TalonFX passengerCabFollower{1, CANBUS_NAME};
  hardware::TalonFX driverTailLeader{2, CANBUS_NAME};
  hardware::TalonFX passengerTailFollower{3, CANBUS_NAME};
  hardware::CANcoder cancoder1{1, CANBUS_NAME}; // on 32 tooth
  hardware::CANcoder cancoder2{2, CANBUS_NAME}; // on 39 tooth
  hardware::CANcoder cancoder3{3, CANBUS_NAME};
  hardware::CANcoder cancoder4{4, CANBUS_NAME};
  hardware::CANcoder cancoder5{5, CANBUS_NAME};
  hardware::CANcoder cancoder6{6, CANBUS_NAME};
  hardware::CANcoder cancoder7{7, CANBUS_NAME};
  hardware::CANcoder cancoder8{8, CANBUS_NAME};

  ctre::phoenix6::controls::MotionMagicVoltage m_mmReq{0_tr};

  bool buttonpressed = false;
  double maxlift = -877;
  int callCount = 0;
  // Gear ratios for the encoders

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

// Function to calculate the closest position
double calculateClosestPosition(double encoder1, double encoder2, int teeth1, int teeth2, int maxRotations)
{
  double minDifference = std::numeric_limits<double>::max();
  double closestPosition = 0.0;

  // Iterate through all possible rotations of the 32-tooth gear
  for (int i = 0; i <= maxRotations; ++i)
  {
    // Calculate the position of the 32-tooth gear
    double position1 = i + encoder1;

    // Calculate the corresponding position of the 39-tooth gear
    double position2 = position1 * teeth1 / teeth2;
    double fractionalPart = position2 - std::floor(position2);

    // Calculate the difference between the calculated position and the encoder reading
    double difference = std::abs(fractionalPart - encoder2);

    // If the difference is smaller than the minimum difference found so far, update the closest position
    if (difference < minDifference)
    {
      minDifference = difference;
      closestPosition = position1;
    }
  }

  return closestPosition;
}

/**
 * Runs once at code initialization.
 */
void UpLift::UpLiftInit()
{

  ctre::phoenixpro::sensors::CANCoderConfiguration config;
  config.AbsoluteSensorRange = ctre::phoenixpro::signals::AbsoluteSensorRangeValue::Unsigned_0To1;
  config.MagnetOffset = 0.0;
  cancoder1.GetConfigurator().Apply(config);

  config.MagnetOffset = 0.0;
  cancoder2.GetConfigurator().Apply(config);

  config.MagnetOffset = 0.0;
  cancoder3.GetConfigurator().Apply(config);

  config.MagnetOffset = 0.0;
  cancoder4.GetConfigurator().Apply(config);

  config.MagnetOffset = 0.0;
  cancoder5.GetConfigurator().Apply(config);

  config.MagnetOffset = 0.0;
  cancoder6.GetConfigurator().Apply(config);

  config.MagnetOffset = 0.0;
  cancoder7.GetConfigurator().Apply(config);

  config.MagnetOffset = 0.0;
  cancoder8.GetConfigurator().Apply(config);
  
  /* Speed up signals to an appropriate rate */
  cancoder1.GetPosition().SetUpdateFrequency(100_Hz);
  cancoder2.GetPosition().SetUpdateFrequency(100_Hz);
  cancoder3.GetPosition().SetUpdateFrequency(100_Hz);
  cancoder4.GetPosition().SetUpdateFrequency(100_Hz);
  cancoder5.GetPosition().SetUpdateFrequency(100_Hz);
  cancoder6.GetPosition().SetUpdateFrequency(100_Hz);
  cancoder7.GetPosition().SetUpdateFrequency(100_Hz);
  cancoder8.GetPosition().SetUpdateFrequency(100_Hz);

  configs::TalonFXConfiguration cfg{};

  configs::MotionMagicConfigs &mm = cfg.MotionMagic;
  mm.MotionMagicCruiseVelocity = 90; 
  mm.MotionMagicAcceleration = 200;  
  mm.MotionMagicJerk = 0;

  configs::Slot0Configs &slot0 = cfg.Slot0;
  slot0.kP = 4.9;
  slot0.kI = 0;
  slot0.kD = 0.0078125;
  slot0.kV = 0.009375;
  slot0.kS = 0.02; // Approximately 0.25V to get the mechanism moving

  configs::FeedbackConfigs &fdb = cfg.Feedback;
  fdb.SensorToMechanismRatio = 1.0;

  cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i)
  {
    status = driverCabLeader.GetConfigurator().Apply(cfg);
    if (status.IsOK())
      break;
  }
  if (!status.IsOK())
  {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }

  cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
  status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i)
  {
    status = passengerCabFollower.GetConfigurator().Apply(cfg);
    if (status.IsOK())
      break;
  }
  if (!status.IsOK())
  {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }

  cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
  status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i)
  {
    status = driverTailLeader.GetConfigurator().Apply(cfg);
    if (status.IsOK())
      break;
  }
  if (!status.IsOK())
  {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }

  cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
  status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i)
  {
    status = passengerTailFollower.GetConfigurator().Apply(cfg);
    if (status.IsOK())
      break;
  }
  if (!status.IsOK())
  {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }

  gpioInitialise();
}

/**
 * Runs periodically during program execution.
 */
void UpLift::UpLiftPeriodic()
{

}

/**
 * Returns whether upLift should be enabled.
 */
bool UpLift::IsEnabled()
{

  return true;
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

  if (gpioRead(22) == 0) // all up
  {
    driverCabLeader.SetControl(m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    driverTailLeader.SetControl(m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    passengerCabFollower.SetControl(m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    passengerTailFollower.SetControl(m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    buttonpressed = true;
  }
  else if (gpioRead(23) == 0) // all down
  {
    driverCabLeader.SetControl(m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    driverTailLeader.SetControl(m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    passengerCabFollower.SetControl(m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    passengerTailFollower.SetControl(m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    buttonpressed = true;
  }
  else if (gpioRead(5) == 0) // twist up
  {
    driverCabLeader.SetControl(m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    driverTailLeader.SetControl(m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    passengerCabFollower.SetControl(m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    passengerTailFollower.SetControl(m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    buttonpressed = true;
  }
  else if (gpioRead(6) == 0) // twist down
  {
    driverCabLeader.SetControl(m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    driverTailLeader.SetControl(m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    passengerCabFollower.SetControl(m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    passengerTailFollower.SetControl(m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    buttonpressed = true;
  }
  else if (buttonpressed = true)
  {
    buttonpressed = false;
    driverCabLeader.SetControl(controls::NeutralOut{});
    driverTailLeader.SetControl(controls::NeutralOut{});
    passengerCabFollower.SetControl(controls::NeutralOut{});
    passengerTailFollower.SetControl(controls::NeutralOut{});
  }
  callCount++;
  if (callCount == 50)
  {

    // Gear teeth
    int teeth1 = 32;
    int teeth2 = 39;

    // Maximum number of rotations to consider
    int maxRotations = 98;
    // Get the current absolute position from the CANCoder
    double encoder5 = cancoder5.GetAbsolutePosition().GetValueAsDouble();

    // Get the current absolute position from the CANCoder
    double encoder6 = cancoder6.GetAbsolutePosition().GetValueAsDouble();


    // Calculate the closest position for the 32-tooth gear
    double crtPosition = 9.0 * calculateClosestPosition(encoder5, encoder6, teeth1, teeth2, maxRotations);
    double position = driverTailLeader.GetPosition().GetValueAsDouble();

    // Print the encoder position
    std::cout << "Encoder position: " << position << " CRT position" << crtPosition << std::endl;

    callCount = 0;
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
  // Given encoder readings
  double encoder1 = 0.122179; // 32-tooth gear
  double encoder2 = 0.92079;  // 39-tooth gear

  // Gear teeth
  int teeth1 = 32;
  int teeth2 = 39;

  // Maximum number of rotations to consider.  
  int maxRotations = 98;

  // Calculate the number of turns of the 32 tooth gear that gives the closest value of what the encoder on the 39 tooth gear is reading
  // e.g. if the 32 tooth gear encoder reads .1, check .1, 1.1, 2.1, 3.1 ... maxRotations.1  and find the value of the 39 tooth gear encoder that is closest
  double closestPosition = calculateClosestPosition(encoder1, encoder2, teeth1, teeth2, maxRotations);

  // Output the result
  std::cout << "The closest position for the 32-tooth gear is " << closestPosition * 9.0 << " turns." << std::endl;

  // return 0;

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
  // return uplift.Run();
  // uplift.SetLoopTime(20_ms); // optionally change loop time for periodic calls
  // Synchronously bind the listener to all nics.

  uclog << U("Starting listener.") << endl;
  http_listener listener(U("http://localhost:8080"));
  listener.open().wait();

  // Handle incoming requests.
  uclog << U("Setting up JSON listener.") << endl;

  listener.support(methods::GET, [](http_request request)
                   {
		
 // Extract query parameters
        auto query_params = uri::split_query(request.request_uri().query());

        auto found_cab = query_params.find(U("cab"));
        auto found_tail = query_params.find(U("tail"));

        // Check if number1 and number2 are present
        if (found_cab == end(query_params) || found_tail == end(query_params)) {
            request.reply(status_codes::BadRequest, U("Both cab and tail are required."));
            return;
        }

        // Convert query parameters to integers
        int cab, tail;
        try {
            cab = std::stoi(query_params[U("cab")]);
            tail = std::stoi(query_params[U("tail")]);
        } catch (const std::invalid_argument&) {
            request.reply(status_codes::BadRequest, U("Invalid integer values for cab or tail."));
            return;
        }

        // Check if values are within the valid range
        if (cab < 0 || cab > 100 || tail < 0 || tail > 100) {
            request.reply(status_codes::BadRequest, U("Both numbers must be between 0 and 100."));
            return;
        }

        double cabSetting = cab * maxlift /100;
        double tailSetting = cab * maxlift /100;
        driverCabLeader.SetControl(m_mmReq.WithPosition(cabSetting* 1_tr).WithSlot(0));
        driverTailLeader.SetControl(m_mmReq.WithPosition(tailSetting* 1_tr).WithSlot(0));
        passengerCabFollower.SetControl(m_mmReq.WithPosition(cabSetting* 1_tr).WithSlot(0));
        passengerTailFollower.SetControl(m_mmReq.WithPosition(tailSetting* 1_tr).WithSlot(0));



        // Process the numbers (you can add your logic here)
        int sum = cab + tail;
            std::cout << "Received tail" << tail << " cab" << cab << std::endl;

        // Create a JSON response
        json::value response;
        response[U("result")] = json::value::number(sum);

        // Send the response
        request.reply(status_codes::OK, response); });

  // Wait while the listener does the heavy lifting.
  // TODO: Provide a way to safely terminate this loop.
  uclog << U("Waiting for incoming connection...") << endl;
  uplift.Run();

  // Nothing left to do but commit suicide.
  uclog << U("Terminating JSON listener.") << endl;
  listener.close();
  return 0;
}
