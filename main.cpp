

#define _CRT_SECURE_NO_DEPRECATE

#include <iostream>
#include <fstream>
#include <iomanip>
#include <string>
#include <sstream>
#include <thread>
#include <chrono>
#include <ctime>
#include <mutex>
#include <atomic>
#include <functional>
#include <map>
#include <vector>
#include <cstdint>

// Bluetooth (BLE GATT server via BlueZ / sdbus-c++), replacing the old
// cpprestsdk HTTP listener that served Home Assistant.
#include <sdbus-c++/sdbus-c++.h>

using namespace std;

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
  int UpLiftInit() override;
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
    if (!(driverCabLeader.SetControl(m_mmReq.WithPosition(cabSetting * 1_tr).WithSlot(0))).IsOK())
    {
      std::cout << "Could not set drivecab position: " << std::endl;
    }
    if (!(driverTailLeader.SetControl(m_mmReq.WithPosition(tailSetting * 1_tr).WithSlot(0))).IsOK())
    {
      std::cout << "Could not set drivetail position: " << std::endl;
    }
    if (!(passengerCabFollower.SetControl(m_mmReq.WithPosition(cabSetting * 1_tr).WithSlot(0))).IsOK())
    {
      std::cout << "Could not set passenger cab position: " << std::endl;
    }
    if (!(passengerTailFollower.SetControl(m_mmReq.WithPosition(tailSetting * 1_tr).WithSlot(0))).IsOK())
    {
      std::cout << "Could not set passenger tail position: " << std::endl;
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

  // ---- Zone-level helpers for the BLE command protocol (RAISE/LOWER/STOP/SET) ----
  // RAISE/LOWER drive one zone to its extreme while leaving the other zone's
  // current commanded position untouched, reusing the existing MotionMagic
  // position control (same mechanism the "SET" command uses).
  void RaiseCab() { SetPositionFrom0To100(100, GetTail()); }
  void LowerCab() { SetPositionFrom0To100(0, GetTail()); }
  void RaiseTail() { SetPositionFrom0To100(GetCab(), 100); }
  void LowerTail() { SetPositionFrom0To100(GetCab(), 0); }

  // STOP cuts power to that zone's motors immediately (NeutralOut), same as
  // what happens when a physical up/down button is released mid-travel.
  void StopCab()
  {
    driverCabLeader.SetControl(controls::NeutralOut{});
    passengerCabFollower.SetControl(controls::NeutralOut{});
  }
  void StopTail()
  {
    driverTailLeader.SetControl(controls::NeutralOut{});
    passengerTailFollower.SetControl(controls::NeutralOut{});
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
int UpLift::UpLiftInit()
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
      std::cerr << "Error reading from file tower Positions File " << std::endl;
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
  return 0;
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
  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  ctre::phoenix6::controls::MotionMagicVoltage dc(0_tr);
  ctre::phoenix6::controls::MotionMagicVoltage dt(0_tr);
  ctre::phoenix6::controls::MotionMagicVoltage pc(0_tr);
  ctre::phoenix6::controls::MotionMagicVoltage pt(0_tr);

  if (gpioRead(24) == 0) // shutdown
  {
    system("shutdown now");
  }
  if (gpioRead(17) == 0) // all up
  {

    dc = (m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    dt = (m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    pc = (m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    pt = (m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    buttonpressed = true;
    sendbuttoncommand = true;
  }
  else if (gpioRead(27) == 0) // all down
  {
    dc = (m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    dt = (m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    pc = (m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    pt = (m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    buttonpressed = true;
    sendbuttoncommand = true;
  }
  else if (gpioRead(22) == 0) // twist up
  {
    dc = (m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    dt = (m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    pc = (m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    pt = (m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    buttonpressed = true;
    sendbuttoncommand = true;
  }
  else if (gpioRead(23) == 0) // twist down
  {
    dc = (m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    dt = (m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    pc = (m_mmReq.WithPosition(maxlift * 1_tr).WithSlot(0));
    pt = (m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
    buttonpressed = true;
    sendbuttoncommand = true;
  }
  else if (buttonpressed == true)
  {
    ::cout << "Clearing buttonpressed" << std::endl;
    buttonpressed = false;

    status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    status = driverCabLeader.SetControl(controls::NeutralOut{});
    if (!status.IsOK())
    {
      std::cout << "Could not command device. Error: " << status.GetName() << std::endl;
      return 1;
    }
    status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    status = driverTailLeader.SetControl(controls::NeutralOut{});
    if (!status.IsOK())
    {
      std::cout << "Could not command device. Error: " << status.GetName() << std::endl;
      return 1;
    }
    status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    status = passengerCabFollower.SetControl(controls::NeutralOut{});
    if (!status.IsOK())
    {
      std::cout << "Could not command device. Error: " << status.GetName() << std::endl;
      return 1;
    }
    status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
    status = passengerTailFollower.SetControl(controls::NeutralOut{});
    if (!status.IsOK())
    {
      std::cout << "Could not command device. Error: " << status.GetName() << std::endl;
      return 1;
    }
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
    std::cerr << "Error writing to file: " << std::endl;
    // Optionally, you can close the file here
    towerPositionsStream.close();
    return 1;
  }

  if (!dcstatus.IsOK() || !dtstatus.IsOK() || !pcstatus.IsOK() || !ptstatus.IsOK())
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

/* ============================================================================
 * Bluetooth (BLE GATT server) — replaces the old cpprestsdk HTTP listener.
 *
 * Talks to the Base44/Capacitor phone app over BLE using a custom service
 * with one write characteristic (commands in) and one notify characteristic
 * (position feedback out), matching the app's "Bluetooth Configuration"
 * panel and the existing cab/tail naming used throughout this file.
 *
 * Protocol:
 *   Write (TX) commands:
 *     RAISE:cab / LOWER:cab / STOP:cab
 *     RAISE:tail / LOWER:tail / STOP:tail
 *     SET:cab:NN / SET:tail:NN     (NN = 0-100)
 *   Notify (RX) feedback:
 *     "cab:45,tail:30"
 *
 * IMPORTANT: BlueZ's GattManager1.RegisterApplication requires the app's
 * root path (APP_PATH) to implement org.freedesktop.DBus.ObjectManager and
 * return every service/characteristic via GetManagedObjects — this is
 * implemented below (GattApplication::GetManagedObjects). If BlueZ still
 * rejects registration on your bluez version, compare against BlueZ's own
 * "example-gatt-server" (Python, in the BlueZ source tree under test/),
 * which is the canonical reference for this wiring.
 * ============================================================================ */

static const std::string BLE_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const std::string BLE_TX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"; // Write  (app -> Pi)
static const std::string BLE_RX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"; // Notify (Pi -> app)

static const char *BLE_ADAPTER_PATH = "/org/bluez/hci0";
static const char *BLE_APP_PATH = "/com/tbosuplift/gatt";
static const char *BLE_SERVICE_PATH = "/com/tbosuplift/gatt/service0";
static const char *BLE_TX_CHAR_PATH = "/com/tbosuplift/gatt/service0/char0";
static const char *BLE_RX_CHAR_PATH = "/com/tbosuplift/gatt/service0/char1";

// Parses "RAISE:cab", "LOWER:tail", "STOP:cab", "SET:tail:45", etc.
static void handleBleCommand(UpLift &uplift, const std::string &cmd)
{
  std::istringstream ss(cmd);
  std::string action, zone, valueStr;
  std::getline(ss, action, ':');
  std::getline(ss, zone, ':');
  std::getline(ss, valueStr, ':');

  if (zone != "cab" && zone != "tail")
  {
    std::cerr << "[BLE] Unknown zone: " << zone << std::endl;
    return;
  }

  if (action == "RAISE")
  {
    std::cout << "[BLE] RAISE " << zone << std::endl;
    (zone == "cab") ? uplift.RaiseCab() : uplift.RaiseTail();
  }
  else if (action == "LOWER")
  {
    std::cout << "[BLE] LOWER " << zone << std::endl;
    (zone == "cab") ? uplift.LowerCab() : uplift.LowerTail();
  }
  else if (action == "STOP")
  {
    std::cout << "[BLE] STOP " << zone << std::endl;
    (zone == "cab") ? uplift.StopCab() : uplift.StopTail();
  }
  else if (action == "SET")
  {
    try
    {
      int target = std::stoi(valueStr);
      target = std::max(0, std::min(100, target));
      std::cout << "[BLE] SET " << zone << " -> " << target << std::endl;
      if (zone == "cab")
        uplift.SetPositionFrom0To100(target, uplift.GetTail());
      else
        uplift.SetPositionFrom0To100(uplift.GetCab(), target);
    }
    catch (...)
    {
      std::cerr << "[BLE] Bad SET value: " << valueStr << std::endl;
    }
  }
  else
  {
    std::cerr << "[BLE] Unknown command: " << cmd << std::endl;
  }
}

static std::string buildPositionFeedback(UpLift &uplift)
{
  std::ostringstream out;
  out << "cab:" << uplift.GetCab() << ",tail:" << uplift.GetTail();
  return out.str();
}

class BleCharacteristic
{
public:
  BleCharacteristic(sdbus::IConnection &connection,
                     std::string path,
                     std::string uuid,
                     std::string servicePath,
                     std::vector<std::string> flags)
      : path_(std::move(path)), uuid_(std::move(uuid)),
        servicePath_(std::move(servicePath)), flags_(std::move(flags))
  {
    object_ = sdbus::createObject(connection, sdbus::ObjectPath{path_});

    object_->registerMethod("ReadValue")
        .onInterface("org.bluez.GattCharacteristic1")
        .implementedAs([this](std::map<std::string, sdbus::Variant>)
                        { return onReadValue(); });

    object_->registerMethod("WriteValue")
        .onInterface("org.bluez.GattCharacteristic1")
        .implementedAs([this](std::vector<uint8_t> value, std::map<std::string, sdbus::Variant>)
                        { onWriteValue(value); });

    object_->registerMethod("StartNotify")
        .onInterface("org.bluez.GattCharacteristic1")
        .implementedAs([this]()
                        { notifying_ = true; });

    object_->registerMethod("StopNotify")
        .onInterface("org.bluez.GattCharacteristic1")
        .implementedAs([this]()
                        { notifying_ = false; });

    object_->registerProperty("UUID")
        .onInterface("org.bluez.GattCharacteristic1")
        .withGetter([this]()
                     { return uuid_; });

    object_->registerProperty("Service")
        .onInterface("org.bluez.GattCharacteristic1")
        .withGetter([this]()
                     { return sdbus::ObjectPath{servicePath_}; });

    object_->registerProperty("Flags")
        .onInterface("org.bluez.GattCharacteristic1")
        .withGetter([this]()
                     { return flags_; });

    object_->registerProperty("Notifying")
        .onInterface("org.bluez.GattCharacteristic1")
        .withGetter([this]()
                     { return notifying_; });

    object_->finishRegistration();
  }

  std::function<void(const std::vector<uint8_t> &)> onWrite;
  std::function<std::vector<uint8_t>()> onRead;

  const std::string &uuid() const { return uuid_; }
  const std::vector<std::string> &flags() const { return flags_; }

  void notify(const std::string &text)
  {
    if (!notifying_)
      return;
    lastValue_.assign(text.begin(), text.end());
    std::map<std::string, sdbus::Variant> changed{{"Value", sdbus::Variant(lastValue_)}};
    object_->emitSignal("PropertiesChanged")
        .onInterface("org.freedesktop.DBus.Properties")
        .withArguments(std::string("org.bluez.GattCharacteristic1"), changed, std::vector<std::string>{});
  }

private:
  std::vector<uint8_t> onReadValue()
  {
    if (onRead)
      return onRead();
    return lastValue_;
  }
  void onWriteValue(const std::vector<uint8_t> &value)
  {
    lastValue_ = value;
    if (onWrite)
      onWrite(value);
  }

  std::string path_, uuid_, servicePath_;
  std::vector<std::string> flags_;
  std::unique_ptr<sdbus::IObject> object_;
  std::vector<uint8_t> lastValue_;
  bool notifying_ = false;
};

// Exposes org.freedesktop.DBus.ObjectManager at BLE_APP_PATH so BlueZ's
// GattManager1.RegisterApplication can discover the service + characteristics.
class GattApplication
{
public:
  explicit GattApplication(sdbus::IConnection &connection)
  {
    object_ = sdbus::createObject(connection, sdbus::ObjectPath{BLE_APP_PATH});
    object_->registerMethod("GetManagedObjects")
        .onInterface("org.freedesktop.DBus.ObjectManager")
        .implementedAs([this]()
                        { return getManagedObjects(); });
    object_->finishRegistration();
  }

  void addService(const std::string &path, const std::string &uuid, bool primary)
  {
    std::map<std::string, sdbus::Variant> props{
        {"UUID", sdbus::Variant(uuid)},
        {"Primary", sdbus::Variant(primary)}};
    objects_[path]["org.bluez.GattService1"] = props;
  }

  void addCharacteristic(const std::string &path, const std::string &uuid,
                          const std::string &servicePath, const std::vector<std::string> &flags)
  {
    std::map<std::string, sdbus::Variant> props{
        {"UUID", sdbus::Variant(uuid)},
        {"Service", sdbus::Variant(sdbus::ObjectPath{servicePath})},
        {"Flags", sdbus::Variant(flags)}};
    objects_[path]["org.bluez.GattCharacteristic1"] = props;
  }

private:
  using InterfaceMap = std::map<std::string, std::map<std::string, sdbus::Variant>>;

  std::map<std::string, InterfaceMap> getManagedObjects()
  {
    std::map<sdbus::ObjectPath, InterfaceMap> result;
    for (auto &[path, ifaces] : objects_)
      result[sdbus::ObjectPath{path}] = ifaces;
    // sdbus-c++ marshals std::map<ObjectPath, InterfaceMap> directly as
    // a{oa{sa{sv}}}; return type kept as InterfaceMap-keyed-by-string above
    // for readability, converted at the call site if your sdbus-c++ version
    // needs an explicit sdbus::ObjectPath key type.
    std::map<std::string, InterfaceMap> out;
    for (auto &[path, ifaces] : objects_)
      out[path] = ifaces;
    return out;
  }

  std::unique_ptr<sdbus::IObject> object_;
  std::map<std::string, InterfaceMap> objects_;
};

static void runBleGattServer(UpLift &uplift)
{
  auto connection = sdbus::createSystemBusConnection();
  connection->requestName("com.tbosuplift.gatt");

  GattApplication app(connection.operator*());

  auto serviceObj = sdbus::createObject(*connection, sdbus::ObjectPath{BLE_SERVICE_PATH});
  serviceObj->registerProperty("UUID").onInterface("org.bluez.GattService1")
      .withGetter([]()
                   { return BLE_SERVICE_UUID; });
  serviceObj->registerProperty("Primary").onInterface("org.bluez.GattService1")
      .withGetter([]()
                   { return true; });
  serviceObj->finishRegistration();
  app.addService(BLE_SERVICE_PATH, BLE_SERVICE_UUID, true);

  BleCharacteristic txChar(*connection, BLE_TX_CHAR_PATH, BLE_TX_CHAR_UUID, BLE_SERVICE_PATH,
                            {"write", "write-without-response"});
  txChar.onWrite = [&uplift](const std::vector<uint8_t> &value)
  {
    std::string cmd(value.begin(), value.end());
    handleBleCommand(uplift, cmd);
  };
  app.addCharacteristic(BLE_TX_CHAR_PATH, BLE_TX_CHAR_UUID, BLE_SERVICE_PATH, txChar.flags());

  BleCharacteristic rxChar(*connection, BLE_RX_CHAR_PATH, BLE_RX_CHAR_UUID, BLE_SERVICE_PATH,
                            {"notify", "read"});
  rxChar.onRead = [&uplift]
  {
    std::string s = buildPositionFeedback(uplift);
    return std::vector<uint8_t>(s.begin(), s.end());
  };
  app.addCharacteristic(BLE_RX_CHAR_PATH, BLE_RX_CHAR_UUID, BLE_SERVICE_PATH, rxChar.flags());

  auto gattMgrProxy = sdbus::createProxy(*connection, sdbus::ServiceName{"org.bluez"},
                                          sdbus::ObjectPath{BLE_ADAPTER_PATH});
  std::map<std::string, sdbus::Variant> registerOptions;
  gattMgrProxy->callMethod("RegisterApplication")
      .onInterface("org.bluez.GattManager1")
      .withArguments(sdbus::ObjectPath{BLE_APP_PATH}, registerOptions);

  std::cout << "[BLE] GATT server running. Service UUID: " << BLE_SERVICE_UUID << std::endl;

  // Poll actual motor position periodically and push it to the phone app
  // whenever it changes — this catches movement from the physical GPIO
  // buttons too, not just BLE-issued commands.
  std::thread notifyThread([&]
                            {
    std::string lastSent;
    while (true)
    {
      std::string current = buildPositionFeedback(uplift);
      if (current != lastSent)
      {
        rxChar.notify(current);
        lastSent = current;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    } });
  notifyThread.detach();

  connection->enterEventLoop();
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
  // uplift.SetLoopTime(20_ms); // optionally change loop time for periodic calls

  // Bluetooth (BLE GATT server) replaces the old HTTP listener as the
  // interface for the phone app. Runs on its own thread since uplift.Run()
  // below blocks the main thread with the motor control loop.
  std::cout << "Starting BLE GATT server..." << std::endl;
  std::thread bleThread(runBleGattServer, std::ref(uplift));
  bleThread.detach();

  std::cout << "Waiting for incoming Bluetooth connections..." << std::endl;
  uplift.Run();

  return 0;
}
