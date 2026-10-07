#define _CRT_SECURE_NO_DEPRECATE

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <pigpio.h>

// Bluetooth (BLE GATT server via BlueZ / sdbus-c++ 1.x), replacing the old
// cpprestsdk HTTP listener that served Home Assistant.
#include <sdbus-c++/sdbus-c++.h>

#include "ctre/phoenix6/CANcoder.hpp"
// #include "ctre/phoenix6/configs/CANcoderConfiguration.hpp"
// #include "ctre/phoenix6/signals/AbsoluteSensorRangeValue.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "UpLiftBase.hpp"
// #include "Joystick.hpp"

using namespace std;
using namespace ctre::phoenix6;

/**
 * This is the main uplift class
 *
 * THREADING MODEL
 *   - The main thread runs the motor control loop (EnabledPeriodic). It is the
 *     ONLY thread that talks to the Phoenix devices.
 *   - The BLE thread(s) never touch Phoenix. They only write the atomic target
 *     values and read the atomic cached positions below.
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

  // "Drive into the limit switch" targets. The physical buttons and the BLE
  // RAISE/LOWER commands command these on purpose so the motors run until the
  // hardware limit switch stops them (the limit switch also re-zeroes the
  // encoder position to the autoset value).
  double maxliftCab = 2000;  // was 877 and could be up to 897
  double maxliftTail = 2000;
  int callCount = 0;

  // Real travel (rotations) that corresponds to 0-100% in the phone app.
  // The forward limit switches autoset the position to ~1017 turns, so that
  // is the true top of travel. Adjust these if your real travel differs.
  static constexpr double percentFullScaleCab = 1017.0;
  static constexpr double percentFullScaleTail = 1017.0;

  // Current commanded target for each zone, in raw encoder rotations.
  // ApplySyncedPositionControl() continuously drives the motors toward these
  // every loop tick. Atomic because the BLE thread writes them while the main
  // thread reads them.
  std::atomic<double> cabTargetRotations{0.0};
  std::atomic<double> tailTargetRotations{0.0};

  // Latest measured leader positions (rotations), published by the main loop
  // so the BLE threads can read them without touching the Phoenix API.
  std::atomic<double> cabPos{0.0};
  std::atomic<double> tailPos{0.0};

  static double clampPercent(double p) { return std::max(0.0, std::min(100.0, p)); }

  // True once all four motors have been configured and their saved positions
  // restored. Until then (e.g. CAN bus not connected) nothing is commanded and
  // the position file is left untouched; EnabledPeriodic() retries every
  // INIT_RETRY_SECONDS. Main thread only.
  bool devicesReady = false;
  std::chrono::steady_clock::time_point nextInitAttempt{};
  static constexpr int INIT_RETRY_SECONDS = 5;
  bool TryInitializeDevices();

public:
  /* main uplift interface */
  int UpLiftInit() override;
  void UpLiftPeriodic() override;

  bool IsEnabled() override;
  void EnabledInit() override;
  int EnabledPeriodic() override;

  void DisabledInit() override;
  void DisabledPeriodic() override;

  // ---- Per-zone setters (thread-safe: only touch atomics) ----
  void SetCabPercent(double percent)
  {
    cabTargetRotations = clampPercent(percent) * percentFullScaleCab / 100.0;
    std::cout << "Processing cab target " << cabTargetRotations.load() << std::endl;
  }
  void SetTailPercent(double percent)
  {
    tailTargetRotations = clampPercent(percent) * percentFullScaleTail / 100.0;
    std::cout << "Processing tail target " << tailTargetRotations.load() << std::endl;
  }
  void SetPositionFrom0To100(double cab, double tail)
  {
    SetCabPercent(cab);
    SetTailPercent(tail);
  }

  // Drives each zone's leader (driver side) toward the commanded target,
  // then drives that zone's follower (passenger side) to the leader's
  // *actual, just-measured* position -- not the same nominal target the
  // leader is chasing. Because the follower is always aimed at where the
  // leader really is right now rather than where it's ultimately headed,
  // the two sides stay level with each other throughout the entire move,
  // even when friction differences make one side lag the other.
  //
  // Called every loop tick (not just when a new command arrives) so the
  // sync holds during travel, not only once the leader reaches its target.
  // MAIN THREAD ONLY.
  bool ApplySyncedPositionControl()
  {
    bool ok = true;

    if (!(driverCabLeader.SetControl(m_mmReq.WithPosition(cabTargetRotations.load() * 1_tr).WithSlot(0))).IsOK())
    {
      std::cout << "Could not set driver cab position: " << std::endl;
      ok = false;
    }
    if (!(driverTailLeader.SetControl(m_mmReq.WithPosition(tailTargetRotations.load() * 1_tr).WithSlot(0))).IsOK())
    {
      std::cout << "Could not set driver tail position: " << std::endl;
      ok = false;
    }

    // Read the leaders' actual position fresh, after commanding them above,
    // so the followers are chasing the freshest possible measurement.
    units::angle::turn_t cabLeaderPos = driverCabLeader.GetPosition().GetValue();
    units::angle::turn_t tailLeaderPos = driverTailLeader.GetPosition().GetValue();

    if (!(passengerCabFollower.SetControl(m_mmReq.WithPosition(cabLeaderPos).WithSlot(0))).IsOK())
    {
      std::cout << "Could not set passenger cab position: " << std::endl;
      ok = false;
    }
    if (!(passengerTailFollower.SetControl(m_mmReq.WithPosition(tailLeaderPos).WithSlot(0))).IsOK())
    {
      std::cout << "Could not set passenger tail position: " << std::endl;
      ok = false;
    }

    return ok;
  }

  // Position as 0-100 percent of real travel, from the cached measurement.
  // Safe to call from any thread.
  int GetCab()
  {
    int pct = (int)std::lround(cabPos.load() * 100.0 / percentFullScaleCab);
    return std::max(0, std::min(100, pct));
  }
  int GetTail()
  {
    int pct = (int)std::lround(tailPos.load() * 100.0 / percentFullScaleTail);
    return std::max(0, std::min(100, pct));
  }

  // ---- Zone-level helpers for the BLE command protocol (RAISE/LOWER/STOP/SET) ----
  // Each one only changes its own zone's target; the other zone is untouched.
  // RAISE/LOWER drive into the limit switches, same as the physical buttons.
  void RaiseCab() { cabTargetRotations = maxliftCab; }
  void LowerCab() { cabTargetRotations = 0.0; }
  void RaiseTail() { tailTargetRotations = maxliftTail; }
  void LowerTail() { tailTargetRotations = 0.0; }

  // STOP holds the zone where it is right now. Because
  // ApplySyncedPositionControl() re-commands position every tick, "stop" means
  // "set the target to the current position" -- the motors actively hold there
  // rather than being cut to neutral.
  void StopCab() { cabTargetRotations = cabPos.load(); }
  void StopTail() { tailTargetRotations = tailPos.load(); }
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
 * Configures the four motors and restores saved positions.
 * Returns false (instead of aborting the program) if anything fails, e.g. the
 * CAN bus is not connected. Safe to call repeatedly.
 */
bool UpLift::TryInitializeDevices()
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
  mm.MotionMagicCruiseVelocity = 85_tps;
  mm.MotionMagicAcceleration = 120_tr_per_s_sq;
  mm.MotionMagicJerk = 0_tr_per_s_cu;

  configs::Slot0Configs &slot0 = cfg.Slot0;
  slot0.kP = 4.5;
  slot0.kI = 0;
  slot0.kD = 0.0078125;
  slot0.kV = 0.009375;
  slot0.kS = 0.02; // Approximately 0.25V to get the mechanism moving

  configs::FeedbackConfigs &fdb = cfg.Feedback;
  fdb.SensorToMechanismRatio = 1.0;

  cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

  cfg.HardwareLimitSwitch.ForwardLimitEnable = true;
  cfg.HardwareLimitSwitch.ForwardLimitAutosetPositionEnable = true;
  cfg.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 1017_tr;

  cfg.HardwareLimitSwitch.ReverseLimitEnable = true;
  cfg.HardwareLimitSwitch.ReverseLimitAutosetPositionEnable = true;
  cfg.HardwareLimitSwitch.ReverseLimitAutosetPositionValue = 0_tr;

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
    return false;
  }

  cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
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
    return false;
  }

  cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;

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
    return false;
  }

  cfg.HardwareLimitSwitch.ForwardLimitAutosetPositionValue = 1054_tr;
  cfg.MotorOutput.Inverted = signals::InvertedValue::Clockwise_Positive;
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
    return false;
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

      return false;
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
    return false;
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
    return false;
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
    return false;
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
    return false;
  }

  // Only truncate the file once everything above has succeeded, so the saved
  // positions survive any number of failed attempts.
  if (!towerPositionsStream.is_open())
  {
    towerPositionsStream.open(towerPositionsFile, std::ios::trunc);
  }
  if (!towerPositionsStream.is_open())
  {
    std::cerr << "Unable to open towerPositions file for writing." << std::endl;
    return false;
  }

  // ApplySyncedPositionControl() runs every EnabledPeriodic tick starting
  // right after this, so seed the targets with where we actually are --
  // otherwise the first tick would command everything toward 0.
  cabTargetRotations = driverCabFromFile;
  tailTargetRotations = driverTailFromFile;
  cabPos = driverCabFromFile;
  tailPos = driverTailFromFile;

  return true;
}

/**
 * Runs once at code initialization. Never fails the program: if the motors
 * can't be reached, Bluetooth and the rest of the program keep running and
 * EnabledPeriodic() keeps retrying.
 */
int UpLift::UpLiftInit()
{
  devicesReady = TryInitializeDevices();
  if (!devicesReady)
  {
    std::cout << "[CAN] Motors not reachable (bus disconnected?). Retrying every "
              << INIT_RETRY_SECONDS << " s; Bluetooth stays up." << std::endl;
    nextInitAttempt = std::chrono::steady_clock::now() + std::chrono::seconds(INIT_RETRY_SECONDS);
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
  gpioInitialise(); // as in the original code; harmless if already initialised

  if (gpioRead(24) == 0) // shutdown
  {
    system("shutdown now");
  }

  // Motors not initialized yet (CAN bus disconnected or devices not
  // responding): don't command anything and don't write the position file
  // (it would overwrite the saved positions with zeros). Just retry on a timer.
  if (!devicesReady)
  {
    if (std::chrono::steady_clock::now() >= nextInitAttempt)
    {
      std::cout << "[CAN] Retrying motor initialization..." << std::endl;
      devicesReady = TryInitializeDevices();
      if (devicesReady)
        std::cout << "[CAN] Motors initialized." << std::endl;
      else
        nextInitAttempt = std::chrono::steady_clock::now() + std::chrono::seconds(INIT_RETRY_SECONDS);
    }
    return 0;
  }

  if (gpioRead(17) == 0) // all up
  {
    cabTargetRotations = maxliftCab;
    tailTargetRotations = maxliftTail;
    buttonpressed = true;
  }
  else if (gpioRead(27) == 0) // all down
  {
    cabTargetRotations = 0.0;
    tailTargetRotations = 0.0;
    buttonpressed = true;
  }
  else if (gpioRead(22) == 0) // twist up
  {
    cabTargetRotations = 0.0;
    tailTargetRotations = maxliftTail;
    buttonpressed = true;
  }
  else if (gpioRead(23) == 0) // twist down
  {
    cabTargetRotations = maxliftCab;
    tailTargetRotations = 0.0;
    buttonpressed = true;
  }
  else if (buttonpressed == true)
  {
    // Button just released: hold right here instead of continuing on
    // toward the old extreme target, and cut power immediately the same
    // way the original code did.
    ::cout << "Clearing buttonpressed" << std::endl;
    buttonpressed = false;

    cabTargetRotations = driverCabLeader.GetPosition().GetValueAsDouble();
    tailTargetRotations = driverTailLeader.GetPosition().GetValueAsDouble();

    driverCabLeader.SetControl(controls::NeutralOut{});
    driverTailLeader.SetControl(controls::NeutralOut{});
    passengerCabFollower.SetControl(controls::NeutralOut{});
    passengerTailFollower.SetControl(controls::NeutralOut{});
  }

  // Every tick -- while a button/BLE move is in progress AND while holding
  // still -- re-drive the leaders toward their target and the followers
  // toward the leaders' actual current position. This is what keeps
  // driver/passenger level with each other through the whole move rather
  // than only once the (slower) side finally catches up.
  bool controlOk = ApplySyncedPositionControl();

  ctre::phoenix::StatusCode dcstatus = driverCabLeader.GetPosition().GetStatus();
  ctre::phoenix::StatusCode dtstatus = driverTailLeader.GetPosition().GetStatus();
  ctre::phoenix::StatusCode pcstatus = passengerCabFollower.GetPosition().GetStatus();
  ctre::phoenix::StatusCode ptstatus = passengerTailFollower.GetPosition().GetStatus();

  const double dcPos = driverCabLeader.GetPosition().GetValueAsDouble();
  const double dtPos = driverTailLeader.GetPosition().GetValueAsDouble();
  const double pcPos = passengerCabFollower.GetPosition().GetValueAsDouble();
  const double ptPos = passengerTailFollower.GetPosition().GetValueAsDouble();

  // Publish the latest measurements for the BLE threads (they never touch
  // the Phoenix API themselves).
  cabPos = dcPos;
  tailPos = dtPos;

  towerPositionsStream.seekp(0);
  towerPositionsStream << std::fixed << std::setprecision(10)
                       << dcPos << " "
                       << dtPos << " "
                       << pcPos << " "
                       << ptPos;
  towerPositionsStream.flush();
  if (towerPositionsStream.fail())
  {
    std::cerr << "Error writing to file: " << std::endl;
    // Optionally, you can close the file here
    towerPositionsStream.close();
    return 1;
  }

  if (!controlOk || !dcstatus.IsOK() || !dtstatus.IsOK() || !pcstatus.IsOK() || !ptstatus.IsOK())
  {

    // std::cout << "Everything is not all good. Shutting down" << pcstatus.GetName() << dcstatus.GetName() << dtstatus.GetName() << ptstatus.GetName() << std::endl;
    // return 1;
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
 * Bluetooth (BLE GATT server) -- replaces the old cpprestsdk HTTP listener.
 *
 * Talks to the Base44/Capacitor phone app over BLE using a custom service
 * with one write characteristic (commands in) and one notify characteristic
 * (position feedback out).
 *
 * Protocol:
 *   Write (TX) commands:
 *     RAISE:cab / LOWER:cab / STOP:cab
 *     RAISE:tail / LOWER:tail / STOP:tail
 *     SET:cab:NN / SET:tail:NN     (NN = 0-100)
 *   Notify (RX) feedback:
 *     "cab:45,tail:30"
 *
 * Structure:
 *   - GattApplication: object at BLE_APP_PATH with an ObjectManager. sdbus-c++
 *     builds the GetManagedObjects reply itself from the service/characteristic
 *     objects registered underneath that path.
 *   - BleAdvertisement: LEAdvertisement1 object so the phone can find the Pi.
 *   - runBleGattServer(): registers everything with BlueZ, starts the sdbus
 *     event loop on its own thread FIRST (BlueZ calls back into us during
 *     RegisterApplication), then loops pushing position updates.
 *   - bleThreadMain(): catches any Bluetooth failure so it can never take
 *     down the motor control loop.
 * ============================================================================ */

static const std::string BLE_SERVICE_UUID = "6e400001-b5a3-f393-e0a9-e50e24dcca9e";
static const std::string BLE_TX_CHAR_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"; // Write  (app -> Pi)
static const std::string BLE_RX_CHAR_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"; // Notify (Pi -> app)

// Keep this short: a 128-bit service UUID already uses 18 of the 31 bytes in
// a legacy advertising packet.
static const std::string BLE_LOCAL_NAME = "UpLift";

static const char *BLE_ADAPTER_PATH = "/org/bluez/hci0";
static const char *BLE_APP_PATH = "/com/tbosuplift/gatt";
static const char *BLE_SERVICE_PATH = "/com/tbosuplift/gatt/service0";
static const char *BLE_TX_CHAR_PATH = "/com/tbosuplift/gatt/service0/char0";
static const char *BLE_RX_CHAR_PATH = "/com/tbosuplift/gatt/service0/char1";
static const char *BLE_ADV_PATH = "/com/tbosuplift/adv0"; // deliberately NOT under BLE_APP_PATH

static std::string trimWhitespace(const std::string &s)
{
  const char *ws = " \t\r\n";
  size_t b = s.find_first_not_of(ws);
  if (b == std::string::npos)
    return "";
  size_t e = s.find_last_not_of(ws);
  return s.substr(b, e - b + 1);
}

// Parses "RAISE:cab", "LOWER:tail", "STOP:cab", "SET:tail:45", etc.
// Runs on the sdbus event-loop thread; only touches atomics via UpLift setters.
static void handleBleCommand(UpLift &uplift, const std::string &rawCmd)
{
  const std::string cmd = trimWhitespace(rawCmd);
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
    if (zone == "cab")
      uplift.RaiseCab();
    else
      uplift.RaiseTail();
  }
  else if (action == "LOWER")
  {
    std::cout << "[BLE] LOWER " << zone << std::endl;
    if (zone == "cab")
      uplift.LowerCab();
    else
      uplift.LowerTail();
  }
  else if (action == "STOP")
  {
    std::cout << "[BLE] STOP " << zone << std::endl;
    if (zone == "cab")
      uplift.StopCab();
    else
      uplift.StopTail();
  }
  else if (action == "SET")
  {
    try
    {
      int target = std::stoi(valueStr);
      target = std::max(0, std::min(100, target));
      std::cout << "[BLE] SET " << zone << " -> " << target << std::endl;
      if (zone == "cab")
        uplift.SetCabPercent(target);
      else
        uplift.SetTailPercent(target);
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
                    { return notifying_.load(); });

    // "Value" is a registered property so PropertiesChanged for it is a real,
    // well-formed signal (this is what BlueZ listens to for notifications).
    object_->registerProperty("Value")
        .onInterface("org.bluez.GattCharacteristic1")
        .withGetter([this]()
                    {
                      std::lock_guard<std::mutex> lock(valueMutex_);
                      return lastValue_;
                    });

    object_->finishRegistration();
  }

  std::function<void(const std::vector<uint8_t> &)> onWrite;
  std::function<std::vector<uint8_t>()> onRead;

  const std::string &uuid() const { return uuid_; }
  const std::vector<std::string> &flags() const { return flags_; }

  // Returns true only if the value was actually pushed to a subscriber, so the
  // caller doesn't mark an update as "sent" when nobody was listening.
  bool notify(const std::string &text)
  {
    if (!notifying_)
      return false;
    {
      std::lock_guard<std::mutex> lock(valueMutex_);
      lastValue_.assign(text.begin(), text.end());
    }
    object_->emitPropertiesChangedSignal("org.bluez.GattCharacteristic1", {"Value"});
    return true;
  }

private:
  std::vector<uint8_t> onReadValue()
  {
    if (onRead)
      return onRead();
    std::lock_guard<std::mutex> lock(valueMutex_);
    return lastValue_;
  }
  void onWriteValue(const std::vector<uint8_t> &value)
  {
    {
      std::lock_guard<std::mutex> lock(valueMutex_);
      lastValue_ = value;
    }
    if (onWrite)
    {
      try
      {
        onWrite(value);
      }
      catch (const std::exception &e)
      {
        std::cerr << "[BLE] Error handling write: " << e.what() << std::endl;
      }
    }
  }

  std::string path_, uuid_, servicePath_;
  std::vector<std::string> flags_;
  std::unique_ptr<sdbus::IObject> object_;
  std::vector<uint8_t> lastValue_;
  std::mutex valueMutex_;
  std::atomic<bool> notifying_{false};
};

// Root object of the GATT application. addObjectManager() makes sdbus-c++
// answer org.freedesktop.DBus.ObjectManager.GetManagedObjects (and emit
// InterfacesAdded/Removed) for every object registered below this path, which
// is exactly what BlueZ's GattManager1.RegisterApplication needs.
class GattApplication
{
public:
  explicit GattApplication(sdbus::IConnection &connection)
  {
    object_ = sdbus::createObject(connection, sdbus::ObjectPath{BLE_APP_PATH});
    object_->addObjectManager();
  }

private:
  std::unique_ptr<sdbus::IObject> object_;
};

// org.bluez.LEAdvertisement1 -- makes the Pi visible to the phone's scan.
class BleAdvertisement
{
public:
  BleAdvertisement(sdbus::IConnection &connection,
                   const std::string &path,
                   std::string localName,
                   std::vector<std::string> serviceUuids)
      : localName_(std::move(localName)), serviceUuids_(std::move(serviceUuids))
  {
    object_ = sdbus::createObject(connection, sdbus::ObjectPath{path});

    object_->registerMethod("Release")
        .onInterface("org.bluez.LEAdvertisement1")
        .implementedAs([]()
                       { std::cerr << "[BLE] Advertisement released by BlueZ" << std::endl; });

    object_->registerProperty("Type")
        .onInterface("org.bluez.LEAdvertisement1")
        .withGetter([]()
                    { return std::string("peripheral"); });

    object_->registerProperty("ServiceUUIDs")
        .onInterface("org.bluez.LEAdvertisement1")
        .withGetter([this]()
                    { return serviceUuids_; });

    object_->registerProperty("LocalName")
        .onInterface("org.bluez.LEAdvertisement1")
        .withGetter([this]()
                    { return localName_; });

    object_->finishRegistration();
  }

private:
  std::string localName_;
  std::vector<std::string> serviceUuids_;
  std::unique_ptr<sdbus::IObject> object_;
};

static void runBleGattServer(UpLift &uplift)
{
  // NOTE: no requestName() -- a well-known bus name isn't needed (BlueZ finds
  // us by object path), and the system bus denies it without a policy file.
  auto connection = sdbus::createSystemBusConnection();

  GattApplication app(*connection);

  auto serviceObj = sdbus::createObject(*connection, sdbus::ObjectPath{BLE_SERVICE_PATH});
  serviceObj->registerProperty("UUID").onInterface("org.bluez.GattService1")
      .withGetter([]()
                  { return BLE_SERVICE_UUID; });
  serviceObj->registerProperty("Primary").onInterface("org.bluez.GattService1")
      .withGetter([]()
                  { return true; });
  serviceObj->finishRegistration();

  BleCharacteristic txChar(*connection, BLE_TX_CHAR_PATH, BLE_TX_CHAR_UUID, BLE_SERVICE_PATH,
                           {"write", "write-without-response"});
  txChar.onWrite = [&uplift](const std::vector<uint8_t> &value)
  {
    std::string cmd(value.begin(), value.end());
    handleBleCommand(uplift, cmd);
  };

  BleCharacteristic rxChar(*connection, BLE_RX_CHAR_PATH, BLE_RX_CHAR_UUID, BLE_SERVICE_PATH,
                           {"notify", "read"});
  rxChar.onRead = [&uplift]
  {
    std::string s = buildPositionFeedback(uplift);
    return std::vector<uint8_t>(s.begin(), s.end());
  };

  BleAdvertisement advert(*connection, BLE_ADV_PATH, BLE_LOCAL_NAME, {BLE_SERVICE_UUID});

  // The event loop MUST be running before RegisterApplication: BlueZ calls
  // back into our objects (GetManagedObjects etc.) while handling the call,
  // and a call made before the loop is running would stall until it times out.
  connection->enterEventLoopAsync();

  auto adapterProxy = sdbus::createProxy(*connection, "org.bluez",
                                         sdbus::ObjectPath{BLE_ADAPTER_PATH});
  std::map<std::string, sdbus::Variant> registerOptions;

  adapterProxy->callMethod("RegisterApplication")
      .onInterface("org.bluez.GattManager1")
      .withArguments(sdbus::ObjectPath{BLE_APP_PATH}, registerOptions);
  std::cout << "[BLE] GATT application registered. Service UUID: " << BLE_SERVICE_UUID << std::endl;

  // Advertising failing shouldn't take the GATT server down (you can still
  // connect from a phone that already knows the address).
  try
  {
    adapterProxy->callMethod("RegisterAdvertisement")
        .onInterface("org.bluez.LEAdvertisingManager1")
        .withArguments(sdbus::ObjectPath{BLE_ADV_PATH}, registerOptions);
    std::cout << "[BLE] Advertising as \"" << BLE_LOCAL_NAME << "\"" << std::endl;
  }
  catch (const sdbus::Error &e)
  {
    std::cerr << "[BLE] Could not start advertising: " << e.getName() << ": " << e.getMessage() << std::endl;
  }

  // Push position updates to the phone whenever they change. This runs on the
  // BLE thread itself (no detached thread holding references to locals).
  // Movement from the physical GPIO buttons is picked up too, since the
  // positions come from the main loop's cached measurements.
  std::string lastSent;
  while (true)
  {
    const std::string current = buildPositionFeedback(uplift);
    // Only remember it as "sent" if a subscriber actually received it, so a
    // phone that subscribes later still gets the current state.
    if (current != lastSent && rxChar.notify(current))
      lastSent = current;
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
}

// Wrapper so a Bluetooth failure (no adapter, BlueZ down, registration
// rejected, ...) is logged instead of calling std::terminate and killing the
// motor control program.
static void bleThreadMain(UpLift &uplift)
{
  try
  {
    runBleGattServer(uplift);
  }
  catch (const sdbus::Error &e)
  {
    std::cerr << "[BLE] Bluetooth disabled: " << e.getName() << ": " << e.getMessage() << std::endl;
  }
  catch (const std::exception &e)
  {
    std::cerr << "[BLE] Bluetooth disabled: " << e.what() << std::endl;
  }
}

/* ------ main function ------ */

int main()
{

  if (gpioInitialise() < 0)
    std::cerr << "[GPIO] gpioInitialise() failed in main()" << std::endl;
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
  std::thread bleThread(bleThreadMain, std::ref(uplift));
  bleThread.detach();

  std::cout << "Waiting for incoming Bluetooth connections..." << std::endl;
  uplift.Run();

  return 0;
}