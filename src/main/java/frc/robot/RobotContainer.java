package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerMechanism;
import frc.robot.subsystems.IntakeMechanism;
import frc.robot.subsystems.LauncherMechanism;
import frc.robot.subsystems.QuestNavSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    /* QuestNav Subsystem */
    private final QuestNavSubsystem m_questNav = new QuestNavSubsystem();

    public enum DriveMode {
        NORMAL,
        BUMP,
        HUB_LOCK
    }

    private DriveMode m_currentMode = DriveMode.NORMAL;

    private final Translation2d HUB_LOCATION = new Translation2d(2, 0);

    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    
    private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final CommandXboxController m_driverController = new CommandXboxController(0);
    private final CommandXboxController m_operatorController = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public Translation2d robotPose;

    private final IntakeMechanism m_intakeSubsystem = new IntakeMechanism();
    private final IndexerMechanism m_indexerSubsystem = new IndexerMechanism();
    private final LauncherMechanism m_launcherSubsystem = new LauncherMechanism(); 

    public double currentLauncherVoltage = 1;

    private void logVoltageChange() {
    String message = "Current Launcher Voltage: " + currentLauncherVoltage;
    m_launcherSubsystem.UpdateCurrentVoltage(currentLauncherVoltage);
    
    System.out.println(message);
    }

    public RobotContainer() {
        DriverStation.silenceJoystickConnectionWarning(true);
        robotPose = drivetrain.getState().Pose.getTranslation();
        configureBindings();

        drivetrain.registerTelemetry(telem -> {
            logger.telemeterize(telem);
            
            if (m_questNav.isTracking()) {
                drivetrain.updateVisionFromQuest(
                    m_questNav.getRobotPose2d(), 
                    m_questNav.getAppTimestamp().orElse(edu.wpi.first.wpilibj.Timer.getFPGATimestamp())
                );
            }
        });

        m_intakeSubsystem.setDefaultCommand(m_intakeSubsystem.setDutyCycle(0));
        m_indexerSubsystem.setDefaultCommand(m_indexerSubsystem.setDutyCycle(0));
        m_launcherSubsystem.setDefaultCommand(m_launcherSubsystem.setDutyCycle(0));
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() -> {
                switch (m_currentMode) {
                    case BUMP:
                        return driveFacingAngle
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withTargetDirection(Rotation2d.fromDegrees(180));

                    case HUB_LOCK:
                        Rotation2d angleToHub = HUB_LOCATION.minus(robotPose).getAngle();
                        return driveFacingAngle
                            .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                            .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                            .withTargetDirection(angleToHub);

                    case NORMAL:
                    default:
                        return drive
                            //for blue alliance X and Y should be negative
                            //for red alliance X and Y should be positive
                            //rotation is always negative
                            .withVelocityX(m_driverController.getLeftY() * MaxSpeed)
                            .withVelocityY(m_driverController.getLeftX() * MaxSpeed)
                            .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate);
                }
            })
        );

        //m_driverController.povLeft().onTrue(Commands.runOnce(() -> m_currentMode = DriveMode.NORMAL));
        //m_driverController.povUp().onTrue(Commands.runOnce(() -> m_currentMode = DriveMode.BUMP));
        //m_driverController.povRight().onTrue(Commands.runOnce(() -> m_currentMode = DriveMode.HUB_LOCK));

        m_operatorController.leftBumper().whileTrue(m_indexerSubsystem.setVoltage(8));
        m_operatorController.rightBumper().whileTrue(m_launcherSubsystem.setVoltage(7.5));
        m_operatorController.povUp().whileTrue(m_intakeSubsystem.setVoltage(-10));
        m_operatorController.povDown().whileTrue(m_intakeSubsystem.setVoltage(6));
        
        //m_driverController.rightBumper().whileTrue(m_launcherSubsystem.smartLaunch(robotPose.getDistance(HUB_LOCATION)));
        //m_driverController.a().whileTrue(drivetrain.applyRequest(() -> brake));

        /*
        m_driverController.x().onTrue(Commands.runOnce(() -> currentLauncherVoltage -= 0.5));
        m_driverController.b().onTrue(Commands.runOnce(() -> currentLauncherVoltage += 0.5));
        m_driverController.a().onTrue(Commands.runOnce(() -> currentLauncherVoltage -= 0.01));
        m_driverController.y().onTrue(Commands.runOnce(() -> currentLauncherVoltage += 0.01));
        */
            /* 
        m_driverController.y().onTrue(
        Commands.runOnce(() -> {
            currentLauncherVoltage += 0.01;
            logVoltageChange();
            })
        );
        m_driverController.a().onTrue(
            Commands.runOnce(() -> {
                currentLauncherVoltage -= 0.01;
                logVoltageChange();
            })
        );
        m_driverController.b().onTrue(
            Commands.runOnce(() -> {
                currentLauncherVoltage += 0.5;
                logVoltageChange();
            })
        );
        m_driverController.x().onTrue(
            Commands.runOnce(() -> {
                currentLauncherVoltage -= 0.5;
                logVoltageChange();
            })
        ); */
        
        //m_driverController.y().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }
}