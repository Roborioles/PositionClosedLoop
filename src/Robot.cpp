/**
 * Example demonstrating the Position closed-loop servo.
 * Tested with Logitech F350 USB Gamepad inserted into Driver Station]
 *
 * Be sure to select the correct feedback sensor using configSelectedFeedbackSensor() below.
 *
 * After deploying/debugging this to your RIO, first use the left Y-stick
 * to throttle the Talon manually.  This will confirm your hardware setup.
 * Be sure to confirm that when the Talon is driving forward (green) the
 * position sensor is moving in a positive direction.  If this is not the cause
 * flip the boolean input to the reverseSensor() call below.
 *
 * Once you've ensured your feedback device is in-phase with the motor,
 * use the button shortcuts to servo to target position.
 *
 * Tweak the PID gains accordingly.
 */
#include "WPILib.h"
#include "ctre/Phoenix.h"
#include "Constants.h"

class Robot: public IterativeRobot {
private:
	WPI_TalonSRX * _leftMotor = new WPI_TalonSRX(2);
	WPI_TalonSRX * _leftFollower = new WPI_TalonSRX(1);
	WPI_TalonSRX * _rightMotor = new WPI_TalonSRX(4);
	WPI_TalonSRX * _rightFollower = new WPI_TalonSRX(3);
	Joystick * _joy = new Joystick(0);
	std::string _sb;
	std::string _Rsb;
	int _loops = 0;
	bool _lastButton1 = false;
	/** save the target position to servo to */
	double targetPositionRotations;

	void RobotInit() {
		/* lets grab the 360 degree position of the MagEncoder's absolute position */
		int absolutePosition = _leftMotor->GetSelectedSensorPosition(0) & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
		/* use the low level API to set the quad encoder signal */
		_leftMotor->SetSelectedSensorPosition(absolutePosition, kPIDLoopIdx,
				kTimeoutMs);

		/* lets grab the 360 degree position of the MagEncoder's absolute position */
		int RabsolutePosition = _rightMotor->GetSelectedSensorPosition(0) & 0xFFF; /* mask out the bottom12 bits, we don't care about the wrap arounds */
		/* use the low level API to set the quad encoder signal */
		_rightMotor->SetSelectedSensorPosition(RabsolutePosition, kPIDLoopIdx,
				kTimeoutMs);

		/* choose the sensor and sensor direction */
		_leftMotor->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx,
				kTimeoutMs);
		_leftMotor->SetSensorPhase(false);
		_leftMotor->SetInverted(true);

		/* choose the sensor and sensor direction */
		_rightMotor->ConfigSelectedFeedbackSensor(
				FeedbackDevice::CTRE_MagEncoder_Relative, kPIDLoopIdx,
				kTimeoutMs);
		_rightMotor->SetSensorPhase(false);

		/* set the peak and nominal outputs, 12V means full */
		_leftMotor->ConfigNominalOutputForward(0, kTimeoutMs);
		_leftMotor->ConfigNominalOutputReverse(0, kTimeoutMs);
		_leftMotor->ConfigPeakOutputForward(0.5, kTimeoutMs);
		_leftMotor->ConfigPeakOutputReverse(-0.5, kTimeoutMs);

		/* set closed loop gains in slot0 */
		_leftMotor->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		_leftMotor->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		_leftMotor->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		_leftMotor->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

		/* set the peak and nominal outputs, 12V means full */
		_rightMotor->ConfigNominalOutputForward(0, kTimeoutMs);
		_rightMotor->ConfigNominalOutputReverse(0, kTimeoutMs);
		_rightMotor->ConfigPeakOutputForward(0.5, kTimeoutMs);
		_rightMotor->ConfigPeakOutputReverse(-0.5, kTimeoutMs);

		/* set closed loop gains in slot0 */
		_rightMotor->Config_kF(kPIDLoopIdx, 0.0, kTimeoutMs);
		_rightMotor->Config_kP(kPIDLoopIdx, 0.1, kTimeoutMs);
		_rightMotor->Config_kI(kPIDLoopIdx, 0.0, kTimeoutMs);
		_rightMotor->Config_kD(kPIDLoopIdx, 0.0, kTimeoutMs);

		/* set control mode to follower for second left motor */
		_leftFollower->Set(ControlMode::Follower,2);
		_leftFollower->SetInverted(true);

		/* set control mode to follower for second right motor */
		_rightFollower->Set(ControlMode::Follower,4);

	}

	/**
	 * This function is called periodically during operator control
	 */
	void TeleopPeriodic() {
		/* get gamepad axis */
		double leftYstick = _joy->GetY();
		double motorOutput = _leftMotor->GetMotorOutputPercent();
		double RmotorOutput = _rightMotor->GetMotorOutputPercent();
		bool button1 = _joy->GetRawButton(1);
		bool button2 = _joy->GetRawButton(2);
		/* prepare line to print */
		_sb.append("\tLout:");
		_sb.append(std::to_string(motorOutput));
		_sb.append("\tpos:");
		_sb.append(std::to_string(_leftMotor->GetSelectedSensorPosition(kPIDLoopIdx)));
		_Rsb.append("\tRout:");
		_Rsb.append(std::to_string(RmotorOutput));
		_Rsb.append("\tpos:");
		_Rsb.append(std::to_string(_rightMotor->GetSelectedSensorPosition(kPIDLoopIdx)));
		/* on button1 press enter closed-loop mode on target position */
		if (!_lastButton1 && button1) {
			/* Position mode - button just pressed */
			targetPositionRotations = leftYstick * 10.0 * 4096; /* 50 Rotations in either direction */
			_leftMotor->Set(ControlMode::Position, targetPositionRotations); /* 50 rotations in either direction */
			_rightMotor->Set(ControlMode::Position, targetPositionRotations); /* 50 rotations in either direction */
		}
		/* on button2 just straight drive */
		if (button2) {
			/* Percent voltage mode */
			_leftMotor->Set(ControlMode::PercentOutput, leftYstick);
			_rightMotor->Set(ControlMode::PercentOutput, leftYstick);
		}
		/* if Talon is in position closed-loop, print some more info */
		if (_leftMotor->GetControlMode() == ControlMode::Position) {
			/* append more signals to print when in speed mode. */
			_sb.append("\terrNative:");
			_sb.append(std::to_string(_leftMotor->GetClosedLoopError(kPIDLoopIdx)));
			_sb.append("\ttrg:");
			_sb.append(std::to_string(targetPositionRotations));
			_Rsb.append("\terrNative:");
			_Rsb.append(std::to_string(_rightMotor->GetClosedLoopError(kPIDLoopIdx)));
			_Rsb.append("\ttrg:");
			_Rsb.append(std::to_string(targetPositionRotations));
		}
		/* print every ten loops, printing too much too fast is generally bad for performance */
		if (++_loops >= 10) {
			_loops = 0;
			printf("%s\n", _sb.c_str());
			printf("%s\n", _Rsb.c_str());
		}
		_sb.clear();
		_Rsb.clear();
		/* save button state for on press detect */
		_lastButton1 = button1;
	}

};

START_ROBOT_CLASS(Robot)
