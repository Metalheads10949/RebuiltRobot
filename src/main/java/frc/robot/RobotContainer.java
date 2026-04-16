package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.AgitatorMechanism;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerMechanism;
import frc.robot.subsystems.IntakeMechanism;
import frc.robot.subsystems.LauncherMechanism;
import frc.robot.subsystems.QuestNavSubsystem;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    //this was 0.75 but imma try increasing to 1
    private double MaxAngularRate = RotationsPerSecond.of(1).in(RadiansPerSecond);

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

    //private final IntakeMechanism m_intakeSubsystem = new IntakeMechanism();
    private final IndexerMechanism m_indexerSubsystem = new IndexerMechanism();
    private final LauncherMechanism m_launcherSubsystem = new LauncherMechanism(); 
    private final AgitatorMechanism m_agitatorSubsystem = new AgitatorMechanism();

    //private final SendableChooser<Command> autoChooser;

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

        //autoChooser = AutoBuilder.buildAutoChooser(); 
        //SmartDashboard.putData("Auto Chooser", autoChooser);

        drivetrain.registerTelemetry(telem -> {
            logger.telemeterize(telem);
            
            if (m_questNav.isTracking()) {
                drivetrain.updateVisionFromQuest(
                    m_questNav.getRobotPose2d(), 
                    m_questNav.getAppTimestamp().orElse(edu.wpi.first.wpilibj.Timer.getFPGATimestamp())
                );
            }
        });

        //m_intakeSubsystem.setDefaultCommand(m_intakeSubsystem.setDutyCycle(0));
        m_indexerSubsystem.setDefaultCommand(m_indexerSubsystem.setDutyCycle(0));
        m_launcherSubsystem.setDefaultCommand(m_launcherSubsystem.setDutyCycle(0));
        m_agitatorSubsystem.setDefaultCommand(m_agitatorSubsystem.setDutyCycle(0));
    }

    public Command getAutonomousCommand() {
        return Commands.runOnce(drivetrain::seedFieldCentric);
    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> 
            drive.withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                 .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                 .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)
            )
        );
            /*
        //Launch
        m_operatorController.rightBumper().whileTrue(m_launcherSubsystem.setVoltage(7.5));
        //Index
        m_operatorController.leftBumper().whileTrue(m_indexerSubsystem.setVoltage(8));
        m_operatorController.leftBumper().whileTrue(m_agitatorSubsystem.setVoltage(12));
        //Intake
        m_operatorController.povDown().whileTrue(m_intakeSubsystem.setVoltage(6));
        //m_operatorController.a().whileTrue(m_agitatorSubsystem.setVoltage(-12));
        //Outtake
        m_operatorController.povUp().whileTrue(m_intakeSubsystem.setVoltage(-10));
            */
    }
}