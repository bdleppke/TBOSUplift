

#define _CRT_SECURE_NO_DEPRECATE

#include <cpprest/http_listener.h>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
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
#include "ctre/phoenix6/CANcoder.hpp"
// #include "ctre/phoenix6/configs/CANcoderConfiguration.hpp"
// #include "ctre/phoenix6/signals/AbsoluteSensorRangeValue.hpp"
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

  // Gear ratios for the encoders
  static constexpr char const *CANBUS_NAME = "can0";

  /* devices */
  hardware::TalonFX driverCabLeader{0, CANBUS_NAME};
  hardware::TalonFX passengerCabFollower{1, CANBUS_NAME};
  hardware::TalonFX driverTailLeader{2, CANBUS_NAME};
  hardware::TalonFX passengerTailFollower{3, CANBUS_NAME};
  /*
  hardware::CANcoder cancoder1{1, CANBUS_NAME}; // on 32 tooth
  hardware::CANcoder cancoder2{2, CANBUS_NAME}; // on 39 tooth
  hardware::CANcoder cancoder3{3, CANBUS_NAME};
  hardware::CANcoder cancoder4{4, CANBUS_NAME};
  hardware::CANcoder cancoder5{5, CANBUS_NAME};
  hardware::CANcoder cancoder6{6, CANBUS_NAME};
  hardware::CANcoder cancoder7{7, CANBUS_NAME};
  hardware::CANcoder cancoder8{8, CANBUS_NAME};
*/
  ctre::phoenix6::controls::MotionMagicVoltage m_mmReq{0_tr};
  std::ofstream towerPositionsStream;

  const std::string towerPositionsFile = "towerPositions.txt";

  bool buttonpressed = false;
  bool sendbuttoncommand = false;
  double maxlift = -877;
  int callCount = 0;

public:
  /* main uplift interface */
  void UpLiftInit() override;
  void UpLiftPeriodic() override;

  bool IsEnabled() override;
  void EnabledInit() override;
  int EnabledPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;
  void SetPositionFrom0To100(double cab, double tail)
  {
    double cabSetting = cab * maxlift / 100;
    double tailSetting = tail * maxlift / 100;
    ::cout << "Processingtail" << tailSetting << " cab" << cabSetting << std::endl;
    if (!(driverCabLeader.SetControl(m_mmReq.WithPosition(cabSetting * 1_tr).WithSlot(0))).isOK())
    {
      std::cout << "Could not set drivecab position: " << status.GetName() << std::endl;
      return 1;
    }
    if (!(driverTailLeader.SetControl(m_mmReq.WithPosition(tailSetting * 1_tr).WithSlot(0))).isOK())
    {
      std::cout << "Could not set drivetail position: " << status.GetName() << std::endl;
      return 1;
    }
    if (!(passengerCabFollower.SetControl(m_mmReq.WithPosition(cabSetting * 1_tr).WithSlot(0))).isOK())
    {
      std::cout << "Could not set passenger cab position: " << status.GetName() << std::endl;
      return 1;
    }
    if (!(passengerTailFollower.SetControl(m_mmReq.WithPosition(tailSetting * 1_tr).WithSlot(0))).isOK())
    {
      std::cout << "Could not set passenger tail position: " << status.GetName() << std::endl;
      return 1;
    }
  }

  int GetCab()
  {
    return (int)(driverCabLeader.GetPosition().GetValueAsDouble() * 100 / maxlift);
  }
  int GetTail()
  {
    return (int)(driverTailLeader.GetPosition().GetValueAsDouble() * 100 / maxlift);
  }
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
    double position1 = i - 1 + encoder1;

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
  /*
    ctre::phoenix6::configs::CANcoderConfiguration config{};
    config.MagnetSensor.AbsoluteSensorRange = ctre::phoenix6::signals::AbsoluteSensorRangeValue::Unsigned_0To1;
    config.MagnetSensor.MagnetOffset = 0.0;
    cancoder1.GetConfigurator().Apply(config);

    config.MagnetSensor.MagnetOffset = 0.0;
    cancoder2.GetConfigurator().Apply(config);

    config.MagnetSensor.MagnetOffset = 0.0;
    cancoder3.GetConfigurator().Apply(config);

    config.MagnetSensor.MagnetOffset = 0.0;
    cancoder4.GetConfigurator().Apply(config);

    config.MagnetSensor.MagnetOffset =0.0;
    cancoder5.GetConfigurator().Apply(config);

    config.MagnetSensor.MagnetOffset = 0.0;
    cancoder6.GetConfigurator().Apply(config);

    config.MagnetSensor.MagnetOffset = 0.0;
    cancoder7.GetConfigurator().Apply(config);

    config.MagnetSensor.MagnetOffset = 0.0;
    cancoder8.GetConfigurator().Apply(config);







    cancoder1.GetPosition().SetUpdateFrequency(100_Hz);
    cancoder2.GetPosition().SetUpdateFrequency(100_Hz);
    cancoder3.GetPosition().SetUpdateFrequency(100_Hz);
    cancoder4.GetPosition().SetUpdateFrequency(100_Hz);
    cancoder5.GetPosition().SetUpdateFrequency(100_Hz);
    cancoder6.GetPosition().SetUpdateFrequency(100_Hz);
    cancoder7.GetPosition().SetUpdateFrequency(100_Hz);
    cancoder8.GetPosition().SetUpdateFrequency(100_Hz);
  */
  configs::TalonFXConfiguration cfg{};

  configs::MotionMagicConfigs &mm = cfg.MotionMagic;
  mm.MotionMagicCruiseVelocity = 90;
  mm.MotionMagicAcceleration = 200;
  mm.MotionMagicJerk = 0;

  configs::Slot0Configs &slot0 = cfg.Slot0;
  slot0.kP = 4.5;
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
    return 1;
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
    return 1;
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
    return 1;
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
    return 1;
  }

  gpioInitialise();

  double driverCabFromFile = 0.0;
  double driverTailFromFile = 0.0;
  double passengerCabFromFile = 0.0;
  double passengerTailFromFile = 0.0;

  // Open the file for reading
  std::ifstream infile(towerPositionsFile);
  if (infile.is_open())
  {
    infile >> driverCabFromFile >> driverTailFromFile >> passengerCabFromFile >> passengerTailFromFile;
    if (infile.fail())
    {
      std::cerr << "Error reading from file tower Positions File " std::endl;
      // Optionally, you can close the file here

      return 1;
    }
    infile.close();
  }
  else
  {
    std::cerr << "File not found. Starting with counter at 0.0." << std::endl;
  }

  std::cout << "Driver Cab File: " << driverCabFromFile << "Actual: " << driverCabLeader.GetPosition().GetValueAsDouble() << std::endl;
  std::cout << "Driver Tail File: " << driverTailFromFile << "Actual: " << driverTailLeader.GetPosition().GetValueAsDouble() << std::endl;
  std::cout << "Passenger Cab File: " << passengerCabFromFile << "Actual: " << passengerCabFollower.GetPosition().GetValueAsDouble() << std::endl;
  std::cout << "Passenger Tail File: " << passengerTailFromFile << "Actual: " << passengerTailFollower.GetPosition().GetValueAsDouble() << std::endl;

  status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i)
  {
    status = driverCabLeader.SetPosition(driverCabFromFile * 1_tr);
    if (status.IsOK())
      break;
  }
  if (!status.IsOK())
  {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
    return 1;
  }

  status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i)
  {
    status = driverTailLeader.SetPosition(driverTailFromFile * 1_tr);
    if (status.IsOK())
      break;
  }
  if (!status.IsOK())
  {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
    return 1;
  }

  status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i)
  {
    status = passengerCabFollower.SetPosition(passengerCabFromFile * 1_tr);
    if (status.IsOK())
      break;
  }
  if (!status.IsOK())
  {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
    return 1;
  }

  status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for (int i = 0; i < 5; ++i)
  {
    status = passengerTailFollower.SetPosition(passengerTailFromFile * 1_tr);
    if (status.IsOK())
      break;
  }
  if (!status.IsOK())
  {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
    return 1;
  }

  towerPositionsStream.open(towerPositionsFile, std::ios::trunc);
  if (!towerPositionsStream.is_open())
  {
    std::cerr << "Unable to open towerPositions file for writing." << std::endl;
    return 1;
  }
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
int UpLift::EnabledPeriodic()
{
  gpioInitialise();

  if (gpioRead(24) == 0) // shutdown
  {
    system("shutdown now");
  }
  if (gpioRead(17) == 0) // all up
  {

    auto dc = (m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    auto dt = (m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    auto pc = (m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    auto pt = (m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    buttonpressed = true;
    sendbuttoncommand = true;
  }
  else if (gpioRead(27) == 0) // all down
  {
    auto dc = (m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    auto dt = (m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    auto pc = (m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    auto pt = (m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    buttonpressed = true;
    sendbuttoncommand = true;
  }
  else if (gpioRead(22) == 0) // twist up
  {
    auto dc = (m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    auto dt = (m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    auto pc = (m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    auto pt = (m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    buttonpressed = true;
    sendbuttoncommand = true;
  }
  else if (gpioRead(23) == 0) // twist down
  {
    auto dc = (m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    auto dt = (m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    auto pc = (m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    auto pt = (m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    buttonpressed = true;
    sendbuttoncommand = true;is
  }
  else if (buttonpressed == true)
  {
    ::cout << "Clearing buttonpressed" << std::endl;
    buttonpressed = false;
    auto dc = (controls::NeutralOut{});
    auto dt = (controls::NeutralOut{});
    auto pc = (controls::NeutralOut{});
    auto pt = (controls::NeutralOut{});
    sendbuttoncommand = true;
  }

  if (sendbuttoncommand == true)
  {
    status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    dcstatus = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    dtstatus = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    pcstatus = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    ptstatus = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    status = driverCabLeader.SetControl(dc);
    if (!status.IsOK())
    {
      std::cout << "Could not command device. Error: " << status.GetName() << std::endl;
      return 1;
    }
    status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    status = driverTailLeader.SetControl(dt);
    if (!status.IsOK())
    {
      std::cout << "Could not command device. Error: " << status.GetName() << std::endl;
      return 1;
    }
    status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    status = passengerCabFollower.SetControl(pc);
    if (!status.IsOK())
    {
      std::cout << "Could not command device. Error: " << status.GetName() << std::endl;
      return 1;
    }
    status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    status = passengerTailFollower.SetControl(pt);
    if (!status.IsOK())
    {
      std::cout << "Could not command device. Error: " << status.GetName() << std::endl;
      return 1;
    }

    sendbuttoncommand = false;
  }

  towerPositionsStream.seekp(0);
  towerPositionsStream << std::fixed << std::setprecision(10) << (dcstatus = driverCabLeader.GetPosition()).GetValueAsDouble() << " " << (dtstatus = driverTailLeader.GetPosition()).GetValueAsDouble() << " " << (pcstatus = passengerCabFollower.GetPosition()).GetValueAsDouble() << " " << (ptstatus = passengerTailFollower.GetPosition()).GetValueAsDouble();
  towerPositionsStream.flush();
  if (towerPositionsStream.fail())
  {
    std::cerr << "Error writing to file: " << filePath << std::endl;
    // Optionally, you can close the file here
    towerPositionsStream.close();
    return 1;
  }

  if (!dcstatus.IsOK() || !dtstatus.IsOK()|| !pcstatus.IsOK() || !ptstatus.IsOK())
  {

    std::cout << "Everything is not all good. Shutting down" << std::endl;
    return 1;
  }

  return 0;

  /*
    callCount++;
    if (callCount == 50)
    {

      // Gear teeth
      int teeth1 = 32;
      int teeth2 = 39;

      // Maximum number of rotations to consider
      int maxRotations = 38;
      // Get the current absolute position from the CANCoder
      double encoder5 = -(cancoder5.GetAbsolutePosition().GetValueAsDouble() - 0.744629);

      // Get the current absolute position from the CANCoder
      double encoder6 = -(cancoder6.GetAbsolutePosition().GetValueAsDouble() - 0.327148);

      // Calculate the closest position for the 32-tooth gear
      double crtPosition = -9.0 * calculateClosestPosition(encoder5, encoder6, teeth1, teeth2, maxRotations);
      double position = driverTailLeader.GetPosition().GetValueAsDouble();

      // Print the encoder position
      std::cout << "Cancoder5:" << encoder5 << "Cancoder6:" << encoder6 << "Encoder position: " << position << " CRT position" << crtPosition << std::endl;

      callCount = 0;

    }
    */
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
  passengerCabFollower.SetControl(controls::NeutralOut{});
  passengerTailFollower.SetControl(controls::NeutralOut{});
}

/* ------ main function ------ */

int main()
{

  gpioInitialise();
  gpioSetMode(22, PI_INPUT);
  gpioSetMode(23, PI_INPUT);
  gpioSetPullUpDown(22, PI_PUD_UP);
  gpioSetPullUpDown(23, PI_PUD_UP);

  gpioSetMode(17, PI_INPUT);
  gpioSetMode(27, PI_INPUT);
  gpioSetPullUpDown(17, PI_PUD_UP);
  gpioSetPullUpDown(27, PI_PUD_UP);

  gpioSetMode(24, PI_INPUT);
  gpioSetPullUpDown(24, PI_PUD_UP);

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

  listener.support(methods::GET, [&](http_request request)
                   {
		
 // Extract query parameters
        auto query_params = uri::split_query(request.request_uri().query());

        auto found_cab = query_params.find(U("cab"));
        auto found_tail = query_params.find(U("tail"));
        int cab, tail;

        // Check if number1 and number2 are present
        if (!(found_cab == end(query_params)) && !(found_tail == end(query_params))) {

      
        // Convert query parameters to integers
        //int cab, tail;
        try
        {
          cab = std::stoi(query_params[U("cab")]);
          tail = std::stoi(query_params[U("tail")]);
          // Check if values are within the valid range
          if (cab >= 0 && cab <= 100 && tail >= 0 && tail <= 100)
          {
            std::cout << "Received tail" << tail << " cab" << cab << std::endl;
            uplift.SetPositionFrom0To100(cab, tail);
          }
        

        } catch (const std::invalid_argument&) {
            std::cout << "Just returning the current positions";
        }

        }


        cab = uplift.GetCab() ;
        tail = uplift.GetTail() ;

      //  cab = 15;
      // tail = 20;
  

      

        // Process the numbers (you can add your logic here)
       
            std::cout << "Sending tail" << tail << " cab" << cab << std::endl;

        // Create a JSON response
        json::value response;
        response[U("cab")] = json::value::number(cab);
        response[U("tail")] = json::value::number(tail);

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
