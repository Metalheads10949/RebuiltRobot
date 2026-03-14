// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.DoubleToIntFunction;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    private int FieldCentricAngle = 0 /*TODO: fix new SwerveRequest.FieldCentricFacingAngle.ForwardPerspectiveValue()*/ ;

    //arctan((robot.x - hub.x)/(robot.y - hub.y))
    private int HubCentricAngle = 0;
    private int RotationTarget = FieldCentricAngle;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final IntakeMechanism m_intakeSubsystem = new IntakeMechanism();
    private final IndexerMechanism m_indexerSubsystem = new IndexerMechanism();
    private final LauncherMechanism m_launcherSubsystem = new LauncherMechanism();

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        configureBindings();

        m_intakeSubsystem.setDefaultCommand(m_intakeSubsystem.setDutyCycle(0));
        m_indexerSubsystem.setDefaultCommand(m_indexerSubsystem.setDutyCycle(0));
        m_launcherSubsystem.setDefaultCommand(m_launcherSubsystem.setDutyCycle(0));

    }

    private void configureBindings() {
        
        //m_driverController.a().whileTrue(m_intakeSubsystem.setSlowVelocity());
        //m_driverController.b().whileTrue(m_intakeSubsystem.setFastVelocity());
        

        m_driverController.x().whileTrue(m_intakeSubsystem.setDutyCycle(0.3));
        m_driverController.y().whileTrue(m_intakeSubsystem.setDutyCycle(-0.3));

        //m_driverController.leftBumper().whileTrue(m_indexerSubsystem.setDutyCycle(0.5));
        //m_driverController.rightBumper().whileTrue(m_indexerSubsystem.setDutyCycle(0.8));

        m_driverController.leftBumper().whileTrue(m_indexerSubsystem.setVoltage(0.7*12));
        //m_driverController.rightBumper().whileTrue(m_launcherSubsystem.setDutyCycle(0.4));

        m_operatorController.leftBumper().whileTrue(m_indexerSubsystem.setVelocity(RPM.of(3000)));
        m_operatorController.rightBumper().whileTrue(m_indexerSubsystem.setVelocity(RPM.of(3500)));

        m_operatorController.a().whileTrue(m_launcherSubsystem.setVelocity(RPM.of(60)));
        m_operatorController.b().whileTrue(m_launcherSubsystem.setVelocity(RPM.of(300)));
        
        m_operatorController.x().whileTrue(m_launcherSubsystem.setDutyCycle(1));
        m_driverController.rightBumper().whileTrue(m_launcherSubsystem.setVoltage(0.4*12));

        //field centric drive command
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(m_driverController.getLeftY() * MaxSpeed) // Drive forward with positive Y (forward)
                    .withVelocityY(m_driverController.getLeftX() * MaxSpeed) // Drive left with positive X (left)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                    //.withForwardPerspective(SwerveRequest.ForwardPerspectiveValue(RotationTarget))
            )
        );
        

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));
        m_driverController.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-m_driverController.getLeftY(), -m_driverController.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        m_driverController.back().and(m_driverController.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        m_driverController.back().and(m_driverController.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        m_driverController.start().and(m_driverController.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        m_driverController.start().and(m_driverController.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        //m_driverController.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}
