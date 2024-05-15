

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

void respond(const http_request& request, const status_code& status, const json::value& response) {
	json::value resp;
	resp[U("status")] = json::value::number(status);
	resp[U("response")] = response;

	// Pack in the current time for debugging purposes.
	time_t now = time(0);
	//utility::stringstream_t ss;
	//ss << put_time(localtime(&now), L"%Y-%m-%dT%H:%S:%MZ");
	resp[U("server_time")] = json::value::string("hello");

	request.reply(status, resp);
}

#include <iostream>		// Include all needed libraries here
#include <pigpio.h>
#include <cmath>
#include <limits>

using namespace std;		// No need to keep using “std”

#include "ctre/phoenix6/TalonFX.hpp"
#include "UpLiftBase.hpp"
//#include "Joystick.hpp"

#include <ctre/phoenix6/CANcoder.hpp>

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
    hardware::TalonFX passengerCabFollower{1, CANBUS_NAME};
    hardware::TalonFX driverTailLeader{2, CANBUS_NAME};
    hardware::TalonFX passengerTailFollower{3, CANBUS_NAME};
    hardware::CANcoder cancoder1{5, CANBUS_NAME};
    hardware::CANcoder cancoder2{6, CANBUS_NAME};

    /* control requests */
   // controls::DutyCycleOut cabOut{0};
   // controls::DutyCycleOut tailOut{0};
   ctre::phoenix6::controls::MotionMagicVoltage m_mmReq{0_tr};


    /* joystick */
 //   Joystick joy{0};
  //  float motorspeed;
    bool buttonpressed = false;
    double maxlift = -877;
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
double calculateClosestPosition(double encoder1, double encoder2, int teeth1, int teeth2, int maxRotations) {
    double minDifference = std::numeric_limits<double>::max();
    double closestPosition = 0.0;

    // Iterate through all possible rotations of the 32-tooth gear
    for (int i = 0; i <= maxRotations; ++i) {
        // Calculate the position of the 32-tooth gear
        double position1 = i + encoder1;

        // Calculate the corresponding position of the 39-tooth gear
        double position2 = position1 * teeth1 / teeth2;
        double fractionalPart = position2 - std::floor(position2);

        // Calculate the difference between the calculated position and the encoder reading
        double difference = std::abs(fractionalPart - encoder2);

        // If the difference is smaller than the minimum difference found so far, update the closest position
        if (difference < minDifference) {
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

  configs::CANcoderConfiguration toApply{};

  /* User can change the configs if they want, or leave it empty for factory-default */

  cancoder1.GetConfigurator().Apply(toApply);
  cancoder2.GetConfigurator().Apply(toApply);

  /* Speed up signals to an appropriate rate */
  cancoder1.GetPosition().SetUpdateFrequency(100_Hz);
  cancoder2.GetPosition().SetUpdateFrequency(100_Hz);


  configs::TalonFXConfiguration cfg{};

  /* Configure current limits */
  configs::MotionMagicConfigs &mm = cfg.MotionMagic;
 // MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);
  mm.MotionMagicCruiseVelocity = 90; // 5 rotations per second cruise
  mm.MotionMagicAcceleration = 200; // Set to 250 to match what we were using on elevator
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

  cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
  ctre::phoenix::StatusCode status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for(int i = 0; i < 5; ++i) {
    status = driverCabLeader.GetConfigurator().Apply(cfg);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }

  cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
  status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for(int i = 0; i < 5; ++i) {
    status = passengerCabFollower.GetConfigurator().Apply(cfg);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }

  cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
  status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for(int i = 0; i < 5; ++i) {
    status = driverTailLeader.GetConfigurator().Apply(cfg);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }

  cfg.MotorOutput.Inverted = signals::InvertedValue::CounterClockwise_Positive;
  status = ctre::phoenix::StatusCode::StatusCodeNotInitialized;
  for(int i = 0; i < 5; ++i) {
    status = passengerTailFollower.GetConfigurator().Apply(cfg);
    if (status.IsOK()) break;
  }
  if (!status.IsOK()) {
    std::cout << "Could not configure device. Error: " << status.GetName() << std::endl;
  }


    /* set follower motors to follow leaders; do NOT oppose the leaders' inverts */
//   passengerCabFollower.SetControl(controls::Follower{driverCabLeader.GetDeviceID(), false});
 //  passengerTailFollower.SetControl(controls::Follower{driverTailLeader.GetDeviceID(), false});
   

    gpioInitialise();
}
    


/**
 * Runs periodically during program execution.
 */
void UpLift::UpLiftPeriodic()
{
    /* periodically check that the joystick is still good */
  //  joy.Periodic();
}

/**
 * Returns whether upLift should be enabled.
 */
bool UpLift::IsEnabled()
{
    /* enable while joystick is an Xbox controller (6 axes),
     * and we are holding the right bumper */
   // if (joy.GetNumAxes() < 6) return false;
   // return joy.GetButton(5); // SDL_CONTROLLER_BUTTON_RIGHTSHOULDER
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

    if (gpioRead(22) == 0)  // all up
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
        driverTailLeader.SetControl(m_mmReq.WithPosition(0.0* 1_tr).WithSlot(0));
        passengerCabFollower.SetControl(m_mmReq.WithPosition(0.0 * 1_tr).WithSlot(0));
        passengerTailFollower.SetControl(m_mmReq.WithPosition(0.0* 1_tr).WithSlot(0));
        buttonpressed = true;
    }
    else if (gpioRead(5) == 0)  // twist up
    {
        driverCabLeader.SetControl(m_mmReq.WithPosition(0.0* 1_tr).WithSlot(0));
        driverTailLeader.SetControl(m_mmReq.WithPosition(maxlift* 1_tr).WithSlot(0));
        passengerCabFollower.SetControl(m_mmReq.WithPosition(0.0* 1_tr).WithSlot(0));
        passengerTailFollower.SetControl(m_mmReq.WithPosition(maxlift* 1_tr).WithSlot(0));
        buttonpressed = true;
    }
    else if (gpioRead(6) == 0) // twist down
    {
        driverCabLeader.SetControl(m_mmReq.WithPosition(maxlift* 1_tr).WithSlot(0));
        driverTailLeader.SetControl(m_mmReq.WithPosition(0.0* 1_tr).WithSlot(0));
        passengerCabFollower.SetControl(m_mmReq.WithPosition(maxlift* 1_tr).WithSlot(0));
        passengerTailFollower.SetControl(m_mmReq.WithPosition(0.0* 1_tr).WithSlot(0));
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
    double encoder2 = 0.92079; // 39-tooth gear

    // Gear teeth
    int teeth1 = 32;
    int teeth2 = 39;

    // Maximum number of rotations to consider
    int maxRotations = 100;

    // Calculate the closest position for the 32-tooth gear
    double closestPosition = calculateClosestPosition(encoder1, encoder2, teeth1, teeth2, maxRotations);

    // Output the result
    std::cout << "The closest position for the 32-tooth gear is " << closestPosition * 9.0 << " turns." << std::endl;

    //return 0;




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
	http_listener listener(U("http://localhost:8080/json"));
	listener.open().wait();

	// Handle incoming requests.
	uclog << U("Setting up JSON listener.") << endl;
	listener.support(methods::GET, [] (http_request req) {
		auto http_get_vars = uri::split_query(req.request_uri().query());

		auto found_name = http_get_vars.find(U("request"));

		if (found_name == end(http_get_vars)) {
			auto err = U("Request received with get var \"request\" omitted from query.");
			uclog << err << endl;
			respond(req, status_codes::BadRequest, json::value::string(err));
			return;
		}

		auto request_name = found_name->second;
		uclog << U("Received request: ") << request_name << endl;
		respond(req, status_codes::OK, json::value::string(U("Request received for: ") + request_name));
	});

	// Wait while the listener does the heavy lifting.
	// TODO: Provide a way to safely terminate this loop.
	uclog << U("Waiting for incoming connection...") << endl;
uplift.Run();

	// Nothing left to do but commit suicide.
	uclog << U("Terminating JSON listener.") << endl;
	listener.close();
	return 0;
  
}
