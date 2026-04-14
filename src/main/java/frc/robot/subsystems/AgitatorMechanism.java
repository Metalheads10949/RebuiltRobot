// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import yams.mechanisms.SmartMechanism;
import yams.mechanisms.config.FlyWheelConfig;
import yams.mechanisms.velocity.FlyWheel;
import yams.motorcontrollers.SmartMotorController;
import yams.motorcontrollers.SmartMotorControllerConfig;
import yams.motorcontrollers.SmartMotorControllerConfig.ControlMode;
import yams.motorcontrollers.SmartMotorControllerConfig.MotorMode;
import yams.motorcontrollers.SmartMotorControllerConfig.TelemetryVerbosity;
import yams.motorcontrollers.local.SparkWrapper;

public class AgitatorMechanism extends SubsystemBase {

  private double kSlowVelocity = 60;
  private double kFastVelocity = 300;

  private double kP = 1;
  private double kI = 0;
  private double kD = 0;

  private SmartMotorControllerConfig smcConfig = new SmartMotorControllerConfig(this)
  .withControlMode(ControlMode.CLOSED_LOOP)
  // Feedback Constants (PID Constants)
  .withClosedLoopController(kP, kI, kD)
  .withSimClosedLoopController(kP, kI, kD)
  // Feedforward Constants
  .withFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  .withSimFeedforward(new SimpleMotorFeedforward(0, 0, 0))
  // Telemetry name and verbosity level
  .withTelemetry("AgitatorMotor", TelemetryVerbosity.HIGH)
  .withGearing(24/16)
  // Motor properties to prevent over currenting.
  .withMotorInverted(true)
  .withIdleMode(MotorMode.COAST)
  .withStatorCurrentLimit(Amps.of(8.5));

  private SparkMax sparkMax = new SparkMax(20, MotorType.kBrushed);

  private SmartMotorController sparkSmartMotorController = new SparkWrapper(sparkMax, DCMotor.getCIM(1), smcConfig) { 
  };

   private final FlyWheelConfig agitatorConfig = new FlyWheelConfig(sparkSmartMotorController)
  // Diameter of the flywheel.
  .withDiameter(Inches.of(2))
  // Mass of the flywheel.
  .withMass(Pounds.of(1))
  // Maximum speed of the shooter.
  .withUpperSoftLimit(RPM.of(1000))
  // Telemetry name and verbosity for the arm.
  .withTelemetry("AgitatorMech", TelemetryVerbosity.HIGH);

  // Shooter Mechanism
  private FlyWheel agitator = new FlyWheel(agitatorConfig);

   /**
   * Gets the current velocity of the shooter.
   *
   * @return Shooter velocity.
   */
  public AngularVelocity getVelocity() {return agitator.getSpeed();}

  /**
   * Set the shooter velocity.
   *
   * @param speed Speed to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */
  public Command setVelocity(AngularVelocity speed) {return agitator.setSpeed(speed);}

  /**
   * Set the dutycycle of the shooter.
   *
   * @param dutyCycle DutyCycle to set.
   * @return {@link edu.wpi.first.wpilibj2.command.RunCommand}
   */

  public Command setSlowVelocity() {return setVelocity((RPM.of(kSlowVelocity)));}
  public Command setFastVelocity() {return setVelocity((RPM.of(kFastVelocity)));}

  public Command setDutyCycle(double dutyCycle) {return agitator.set(dutyCycle);}

  public Command setVoltage(double volts) {return agitator.setVoltage(Volts.of(volts));}
  
  /** Creates a new ExampleSubsystem. */
  public AgitatorMechanism() {}

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    agitator.updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
    agitator.simIterate();
  }
}
