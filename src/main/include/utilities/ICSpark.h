#pragma once

#include <rev/SparkBase.h>
#include <rev/SparkSim.h>
#include <rev/config/SparkBaseConfigAccessor.h>
#include <frc/controller/PIDController.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <frc/Alert.h>
#include <frc/Timer.h>
#include <units/acceleration.h>
#include <units/length.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/angular_acceleration.h>
#include <units/time.h>
#include <units/current.h>
#include <units/velocity.h>
#include <units/temperature.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableBuilder.h>
#include "utilities/ICSparkEncoder.h"

/**
 * Wrapper around the Rev CANSparkBase class with some convenience features.
 * - Easier simulation support (see CalcSimVoltage() and IterateSim())
 * - Uses C++ units
 * - Encoder and pid functions are built in
 */
class ICSpark : public wpi::Sendable {
 public:
  enum class ControlType {
    kDutyCycle = (int)rev::spark::SparkLowLevel::ControlType::kDutyCycle,
    kVelocity = (int)rev::spark::SparkLowLevel::ControlType::kVelocity,
    kVoltage = (int)rev::spark::SparkLowLevel::ControlType::kVoltage,
    kPosition = (int)rev::spark::SparkLowLevel::ControlType::kPosition,
    kMaxMotion = (int)rev::spark::SparkLowLevel::ControlType::kMAXMotionPositionControl,
    kCurrent = (int)rev::spark::SparkLowLevel::ControlType::kCurrent,
    kMotionProfile = 10
  };

  using VoltsPerRpm = units::unit_t<
      units::compound_unit<units::volts, units::inverse<units::revolutions_per_minute>>>;
  using VoltsPerRpmPerS = units::unit_t<
      units::compound_unit<units::volts, units::inverse<units::revolutions_per_minute_per_second>>>;

  /**
   * Create a new object to control a SPARK motor controller.
   *
   * @param spark Reference to the spark to control
   * @param inbultEncoder The encoder built into the NEO
   */
  ICSpark(rev::spark::SparkBase* spark, rev::spark::SparkRelativeEncoder& inbuiltEncoder,
          rev::spark::SparkBaseConfigAccessor& configAccessor);

  /**
   * Sets position of motor
   *
   * @param position What to set the position to
   */
  void SetPosition(units::turn_t position);

  /**
   * Sets a closed loop position target (aka reference or goal) for the motor to drive to.
   *
   * @param target The target position drive to.
   *
   * @param arbFeedforward A voltage from -32.0V to 32.0V which is applied to the motor after the
   * result of the specified control mode. This value is added after the control mode, but before
   * any current limits or ramp rates
   *
   * @param slot The closed loop configuration slot to use for this control action
   */
  void SetPositionTarget(units::turn_t target, units::volt_t arbFeedForward = 0.0_V,
                         rev::spark::ClosedLoopSlot slot = rev::spark::ClosedLoopSlot::kSlot0);

  /**
   * Sets a closed loop position target (aka reference or goal) for the motor to drive to using the
   * Spark's MAXMotion control mode. This generates a profiled movement that accelerates and
   * decelerates in a controlled way. This can reduce ware on components, limit current draw and is
   * easier to tune. Also consider using SetMotionProfileTarget() to compute a profile on the
   * RoboRio and perform feedback control based on position.
   *
   * @param target The target position drive to.
   *
   * @param arbFeedforward A voltage from -32.0V to 32.0V which is applied to the motor after the
   * result of the specified control mode. This value is added after the control mode, but before
   * any current limits or ramp rates
   *
   * @param slot The closed loop configuration slot to use for this control action
   */
  void SetMaxMotionTarget(units::turn_t target, units::volt_t arbFeedForward = 0.0_V,
                          rev::spark::ClosedLoopSlot slot = rev::spark::ClosedLoopSlot::kSlot0);

  /**
   * !! Must periodically call UpdateMotionProfile() !!
   * Sets a closed loop position target (aka reference or goal) for the motor to drive to using a
   * motion profile computed onboard the RoboRio and followed using the Spark's onboard PID position
   * control. This generates a profiled movement that accelerates and decelerates in a controlled
   * way. This allows model-based feedforward for faster response time, can reduce ware on
   * components, limit current draw and is easier to tune.
   *
   * Relies on periodic calls to UpdateMotionProfile() to update the feedforward model and targets.
   *
   * @param target The target position drive to.
   *
   * @param arbFeedforward A voltage from -32.0V to 32.0V which is applied to the motor after the
   * result of the specified control mode. This value is added after the control mode, but before
   * any current limits or ramp rates
   *
   * @param slot The closed loop configuration slot to use for this control action
   */
  void SetMotionProfileTarget(units::turn_t target, units::volt_t arbFeedForward = 0.0_V,
                              rev::spark::ClosedLoopSlot slot = rev::spark::ClosedLoopSlot::kSlot0);

  /**
   * Sets the closed loop target (aka reference or goal) for the motor to drive to.
   *
   * @param target The target position drive to.
   *
   * @param arbFeedforward A voltage from -32.0V to 32.0V which is applied to the motor after the
   * result of the specified control mode. This value is added after the control mode, but before
   * any current limits or ramp rates
   *
   * @param slot The closed loop configuration slot to use for this control action
   */
  void SetVelocityTarget(units::revolutions_per_minute_t target,
                         units::volt_t arbFeedForward = 0.0_V,
                         rev::spark::ClosedLoopSlot slot = rev::spark::ClosedLoopSlot::kSlot0);

  /**
   * Update motion profile targets and feedforward calculations. This is required to be called
   * periodically when using MaxMotion or Motion Profile.
   *
   * @param loopTime The frequency at which this is being called. 20ms is the default loop time for
   * WPILib periodic functions.
   */
  void UpdateMotionProfile(units::second_t loopTime = 20_ms);

  /**
   * Calculate how many volts to send to the motor from the confgured feedforward model.
   *
   * @param pos The position target
   * @param vel The velocity target
   * @param accel The acceleration target
   */
  units::volt_t CalculateFeedforward(
      units::turn_t pos, units::revolutions_per_minute_t vel,
      units::revolutions_per_minute_per_second_t accel = 0_tr_per_s_sq);

  /**
   * Gets the current closed loop position target if there is one. Zero otherwise.
   */
  units::turn_t GetPositionTarget() { return _positionTarget; };

  /**
   * Gets the current closed loop velocity target if there is one. Zero otherwise
   */
  units::revolutions_per_minute_t GetVelocityTarget() { return _velocityTarget; };

  /**
   * Get the closed loop position error (current position - target position) if there is one.
   * Zero otherwise.
   */
  units::turn_t GetPosError() { return GetPosition() - _positionTarget; }

  /**
   * Get the closed loop velocity error (current velocity - target velocity) if there is one.
   * Zero otherwise.
   */
  units::revolutions_per_minute_t GetVelError() { return GetVelocity() - _velocityTarget; }

  /**
   * Calculates how much voltage the spark would be giving to the attached motor given its
   * current control type and PID configuration. Use this in conjunction with one of WPILib's
   * physics simulation classes.
   * (https://docs.wpilib.org/en/stable/docs/software/wpilib-tools/robot-simulation/physics-sim.html)
   */
  units::volt_t CalcSimVoltage();

  /**
   * Run internal calculations and set internal state.
   * This method belongs in Simulation Periodic. Use a WPILib physics simulation class or equivalent
   * to calculate the velocity from the applied output and pass it in to this method, which will
   * update the simulated state of the motor.
   *
   * Simulating a Spark this way will use the configurations and controls of the original
   * CANSparkMax or CANSparkFlex device to simulate all supported control modes, arb feedforward
   * input, voltage compensation, limit switches, soft limits, and current limiting, with
   * algorithms translated directly from the Spark firmware.
   *
   * This method will update the CANSparkSim's position and velocity, accessible with getPosition()
   * and getVelocity(). These values are automatically used as the selected feedback sensor for
   * calculations like closed-loop control and soft limits, and are reflected in the selected
   * sensor's value. Other sensors each have their own Sim class, which can be used to inject their
   * positions based on these calculations, to match how they are configured physically. For
   * example, to represent an Absolute Encoder on a 1:5 ratio from the mechanism,
   * SparkAbsoluteEncoderSim.iterate(double, double) is called each simulationPeriodic loop with a
   * velocity divided by 5. The selected sensor's position and velocity will automatically be
   * updated to match the CANSparkSim's when this method is called.
   *
   * @param velocity - The externally calculated velocity in units after conversion. The internal
   * simulation state will 'lag' slightly behind this input due to the SPARK Device internal
   * filtering.
   * @param position - The externally calculated position in units after conversion. If position is
   * not provided, it will be calculated from the velocity over time (this may fall out of sync with
   * a WPI physics simulation).
   */
  void IterateSim(units::revolutions_per_minute_t velocity,
                  std::optional<units::turn_t> position = std::nullopt);

  /**
   * Gets the current closed loop control type.
   */
  ControlType GetControlType() { return _controlType; };

  /**
   * Get the velocity of the motor.
   */
  units::revolutions_per_minute_t GetVelocity();

  /**
   * Get the position of the motor.
   */
  units::turn_t GetPosition();

  /**
   * Get the duty cycle (-1 to 1) of the motor.
   */
  double GetDutyCycle() const;

  /**
   * Get the voltage applied to the motor.
   */
  units::volt_t GetMotorVoltage();

  /**
   * Get the current draw of the motor stator.
   */
  units::ampere_t GetStatorCurrent();

  /**
   * Get the temperature of the motor.
   */
  units::celsius_t GetTemperature();

  /**
   * Common interface to stop the motor until Set is called again or closed loop control is started.
   */
  void StopMotor() { SetDutyCycle(0); };

  /**
   * Sets the duty cycle of a speed controller.
   *
   * @param speed The duty cycle to set. Value should be between -1.0 and 1.0.
   */
  void SetDutyCycle(double speed);

  /**
   * Sets the voltage of a speed controller.
   *
   * @param output The voltage to set.
   */
  void SetVoltage(units::volt_t output);

  /**
   * Check whether the motor is on its position target, within a given tolerance.
   *
   * @param tolerance The tolerance to be considered on target
   */
  bool OnPosTarget(units::turn_t tolerance) { return units::math::abs(GetPosError()) < tolerance; }

  /**
   * Check whether the motor is on its velocity target, within a given tolerance.
   *
   * @param tolerance The tolerance to be considered on target
   */
  bool OnVelTarget(units::revolutions_per_minute_t tolerance) {
    return units::math::abs(GetVelError()) < tolerance;
  }

  /**
   * Set the configuration for the SPARK.
   * When configuring the conversion factors, the ICSpark assumes you are converting position into
   * rotations and velocity into RPM.
   *
   * If @c resetMode is ResetMode::kResetSafeParameters, this method will reset safe writable
   * parameters to their default values before setting the given configuration. The following
   * parameters will not be reset by this action: CAN ID, Motor Type, Idle Mode, PWM Input Deadband,
   * and Duty Cycle Offset.
   *
   * If @c persistMode is PersistMode::kPersistParameters, this method will save all parameters
   * to the SPARK's non-volatile memory after setting the given configuration. This will allow
   * parameters to persist across power cycles.
   *
   * @param config The desired SPARK configuration
   * @param resetMode Whether to reset safe parameters before setting the configuration
   * @param persistMode Whether to persist the parameters after setting the configuration
   * @param async Whether to run the configuration asynchronously (without waiting for a response)
   * @return REVLibError::kOk if successful, async will always return kOk
   */
  rev::REVLibError Configure(rev::spark::SparkBaseConfig& config,
                             rev::ResetMode resetMode,
                             rev::PersistMode persistMode, bool async = false);

  /**
   * Convenience method for calling
   * Configure(config, ResetMode::kNoResetSafeParameters, PersistMode::kPersistParameters)
   */
  rev::REVLibError AdjustConfig(rev::spark::SparkBaseConfig& config);

  /**
   * Convenience method for calling
   * Configure(config, ResetMode::kNoResetSafeParameters, PersistMode::kNoPersistParameters)
   */
  rev::REVLibError AdjustConfigNoPersist(rev::spark::SparkBaseConfig& config);

  /**
   * Convenience method for calling
   * Configure(config, ResetMode::kResetSafeParameters, PersistMode::kPersistParameters)
   */
  rev::REVLibError OverwriteConfig(rev::spark::SparkBaseConfig& config);

  // Sendable setup, called automatically when this is passed into smartDashbaord::PutData()
  void InitSendable(wpi::SendableBuilder& builder) override;

  // Check for temperature, voltage and current alerts. This should be called periodically
  void CheckAlerts();

 protected:
  // Use a relative (alternarte for Max, external for Flex) encoder as the logging device.
  template <std::derived_from<rev::RelativeEncoder> RelEncoder>
  void UseRelativeEncoder(RelEncoder& encoder) {
    _encoder.UseRelative(encoder);
  }

 private:
  /**
   * Refresh internal config cache to match the SPARK's current configuration.
   */
  void RefreshConfigCache();

  /**
   * Configure the maximum velocity constraint for motion profiles. This includes the on-controller
   * MAX Motion mode and the on-rio motion profiles.
   * This uses the Set Parameter API and should be used infrequently.
   *
   * @param maxVelocity The maxmimum velocity for the motion profile.
   */
  void TuneMotionMaxVel(units::revolutions_per_minute_t maxVelocity);

  /**
   * Configure the maximum acceleration constraint for motion profiles. This includes the
   * on-controller MAX Motion mode and the on-rio motion profiles.
   * This uses the Set Parameter API and should be used infrequently.
   *
   * @param maxAcceleration The maxmimum acceleration for the motion profile.
   */
  void TuneMotionMaxAccel(units::revolutions_per_minute_per_second_t maxAcceleration);

  /**
   * Set the Proportional gain for PID feedback control. This uses the Set Parameter API and should
   * be used infrequently.
   *
   * @param P The proportional gain value, must be positive
   */
  void TuneFeedbackProportional(double P);

  /**
   * Set the Integral gain for PID feedback control. This uses the Set Parameter API and should
   * be used infrequently.
   *
   * @param I The Integral gain value, must be positive
   */
  void TuneFeedbackIntegral(double I);

  /**
   * Set the Derivative gain for PID feedback control. This uses the Set Parameter API and should
   * be used infrequently.
   *
   * @param D The Derivative gain value, must be positive
   */
  void TuneFeedbackDerivative(double D);

  /**
   * Set the Static Friction gain constant of the feed forward model.
   *
   * @param S Constant voltage to overcome static friction in the system.
   */
  void TuneFeedforwardStaticFriction(units::volt_t S);

  /**
   * Set the Linear Gravity gain constant of the feed forward model. linearG is a constant voltage
   * that is always applied, regardless of direction of travel. Useful on elevators.
   *
   * @param linearG Voltage to overcome linear gravity in the system.
   */
  void TuneFeedforwardLinearGravity(units::volt_t linearG);

  /**
   * Set the Rotational Gravity gain constant of the feed forward model. Gravity compensation
   * voltage is calculated as rotationalG * cos(angle). Meaning, rotationalG should be the voltage
   * to compensate gravity when the mechanism is at 0 degrees. Useful on arms.
   *
   * @param rotationalG Voltage to overcome rotational gravity in the system.
   */
  void TuneFeedforwardRotationalGravity(units::volt_t rotationalG);

  /**
   * Set the Velocity gain constant of the feed forward model.
   *
   * @param V Voltage to travel at a desired velocity.
   */
  void TuneFeedforwardVelocity(VoltsPerRpm V);

  /**
   * Set the Acceleration gain constant of the feed forward model.
   *
   * @param A Voltage to travel at a desired acceleration.
   */
  void TuneFeedforwardAcceleration(VoltsPerRpmPerS A);

  /**
   * This sets the ratio that is used to calculate the absolute position of
   * your arm mechanism for use with RotationalGravity. This is applied after the
   * conversion factor and should convert from those units to absolute
   * rotations of your mechanism. Ensure your selected encoder is zeroed such
   * that 0 = horizontal.
   *
   * @param kCosRatio The kCosRatio in Volts
   */
  void TuneFeedforwardCosineRatio(double ratio);

  rev::spark::SparkBase* _spark;
  rev::spark::SparkBaseConfigAccessor _configAccessor;
  struct ConfigCache {
    double feedbackP = 0.0;
    double feedbackI = 0.0;
    double feedbackD = 0.0;
    units::volt_t feedforwardStaticFriction = 0_V;
    units::volt_t feedforwardLinearGravity = 0_V;
    units::volt_t feedforwardRotationalGravity = 0_V;
    VoltsPerRpm feedforwardVelocity = 0_V / 1_rpm;
    VoltsPerRpmPerS feedforwardAcceleration = 0_V / 1_rev_per_m_per_s;
    double feedforwardCosineRatio = 0.0;
    units::revolutions_per_minute_t motionMaxVelocity = 0_rpm;
    units::revolutions_per_minute_per_second_t motionMaxAcceleration = 0_rev_per_m_per_s;
  } _configCache;

  // Feedback control objects
  rev::spark::SparkClosedLoopController _sparkPidController{_spark->GetClosedLoopController()};
  ICSparkEncoder _encoder;
  units::volt_t _arbFeedForward = 0.0_V;

  // Control References (Targets)
  using MPState = frc::TrapezoidProfile<units::turns>::State;
  units::turn_t _positionTarget{0};
  units::revolutions_per_minute_t _velocityTarget{0};
  frc::TrapezoidProfile<units::turns> _motionProfile{{0_rpm, 0_rev_per_m_per_s}};
  MPState CalcNextMotionTarget(MPState current, units::turn_t goalPosition,
                               units::second_t lookahead = 20_ms);
  MPState _latestMotionTarget;

  // Control Type management
  ControlType _controlType = ControlType::kDutyCycle;
  rev::spark::SparkLowLevel::ControlType GetREVControlType();
  bool InMotionMode();

  // Alerts - Text added later
  /* Please Note: Enabling []All Exceptions will cause VScode to breakpoint on a
   * exception that purposefully thrown and caught by WPIlib. The exception is 
   * thrown when adding an alert for the first time. */
  frc::Alert _configErrorAlert{"", frc::Alert::AlertType::kError};
  frc::Alert _temperatureAlert{"", frc::Alert::AlertType::kWarning};
  frc::Alert _currentAlert{"", frc::Alert::AlertType::kWarning};
  frc::Timer _currentAlertTimer;

  // Simulation objects
  frc::DCMotor _vortexModel = frc::DCMotor::NeoVortex();
  rev::spark::SparkSim _simSpark;
};
