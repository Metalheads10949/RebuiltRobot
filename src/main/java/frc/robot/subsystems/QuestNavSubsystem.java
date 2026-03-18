// Copyright QuestNav 2025
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {
  private final QuestNav questNav;

  /** * Transform from robot center (floor) to QuestNav camera position.
   * Example: Camera is 0.5 meters up, 0.1 meters forward, and tilted up 20 degrees.
   */
  private static final Transform3d ROBOT_TO_QUEST = new Transform3d(
      0.1, 0.0, 0.5, new Rotation3d(0.0, Math.toRadians(-20), 0.0));

  public QuestNavSubsystem() {
    questNav = new QuestNav();
  }

  @Override
  public void periodic() {
    questNav.commandPeriodic();
  }

  /** Gets the latest 3D robot pose. */
  public Pose3d getRobotPose3d() {
    try {
      PoseFrame[] poseFrames = questNav.getAllUnreadPoseFrames();
      if (poseFrames.length > 0) {
        Pose3d questPose = poseFrames[poseFrames.length - 1].questPose3d(); 
        if (questPose != null) {
          return questPose.transformBy(ROBOT_TO_QUEST.inverse());
        }
      }
    } catch (Exception e) {
      System.err.println("Error getting QuestNav Pose3d: " + e.getMessage());
    }
    return new Pose3d();
  }

  /** Helper to get 2D pose for standard WPILib odometry/Swerve. */
  public Pose2d getRobotPose2d() {
    return getRobotPose3d().toPose2d();
  }

  /** Sets the robot's pose in QuestNav using 3D coordinates. */
  public void setRobotPose(Pose3d robotPose) {
    Pose3d questPose = robotPose.transformBy(ROBOT_TO_QUEST);
    questNav.setPose(questPose);
  }

  public boolean isActive() {
    return questNav.isConnected();
  }

  public boolean isTracking() {
    return questNav.isTracking();
  }

  public PoseFrame[] getAllUnreadPoseFrames() {
    return questNav.getAllUnreadPoseFrames();
  }

  public double getLatency() {
    return questNav.getLatency();
  }

  public java.util.OptionalInt getBatteryPercent() {
    return questNav.getBatteryPercent();
  }

  public java.util.OptionalInt getFrameCount() {
    return questNav.getFrameCount();
  }

  public java.util.OptionalInt getTrackingLostCounter() {
    return questNav.getTrackingLostCounter();
  }

  public java.util.OptionalDouble getAppTimestamp() {
    return questNav.getAppTimestamp();
  }

  public double getCurrentYawDegrees() {
    return getRobotPose2d().getRotation().getDegrees();
  }

  public double getCurrentYawRadians() {
    return getRobotPose2d().getRotation().getRadians();
  }

  public Rotation2d getCurrentRotation() {
    return getRobotPose2d().getRotation();
  }

  public Translation2d getCurrentPosition() {
    return getRobotPose2d().getTranslation();
  }
}