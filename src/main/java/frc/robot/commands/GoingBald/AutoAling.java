package frc.robot.commands.GoingBald;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class AutoAling extends Command {
  public GeneralAlign align;
  public RobotBase<?> base;
  public Pose2d goalPose2d;
  public boolean rightPole;
  public Pose2d finalPose2d;
  public AutoAling(GeneralAlign align, RobotBase<?> base, boolean rightPole) {
    this.align = align;
    this.base = base;
    this.rightPole = rightPole;
  }

  @Override
  public void initialize() 
  {
    align.reset();
  }

  @Override
  public void execute() 
  {
    align.setTagId();
    // align.shuffleboard();
    LimeLight camera_left = base.getVision().getLimelight("limelight-left");
    LimeLight camera_right = base.getVision().getLimelight("limelight-right");

    if(camera_left.hasValidTarget() && (int)camera_left.getAprilTagID() == align.getTagId())
      {      
        goalPose2d = new Pose2d(Units.inchesToMeters(40), rightPole ? Units.inchesToMeters(0) : Units.inchesToMeters(0),new Rotation2d()).rotateAround(new Translation2d(0, 0), ReefPole.getPoleFromID(camera_left.getAprilTagID(), camera_left).getRotation());
        finalPose2d = new Pose2d(Units.inchesToMeters(15.5), rightPole ? Units.inchesToMeters(6.2) : Units.inchesToMeters(-11.5),new Rotation2d()).rotateAround(new Translation2d(0, 0), ReefPole.getPoleFromID(camera_left.getAprilTagID(), camera_left).getRotation());  
      }

    if(camera_right.hasValidTarget() && (int)camera_right.getAprilTagID() == align.getTagId())
      {    
        goalPose2d = new Pose2d(Units.inchesToMeters(40), rightPole ? Units.inchesToMeters(0) : Units.inchesToMeters(0),new Rotation2d()).rotateAround(new Translation2d(0, 0), ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera_right).getRotation());
        finalPose2d = new Pose2d(Units.inchesToMeters(15.5), rightPole ? Units.inchesToMeters(6.2) : Units.inchesToMeters(-11.5),new Rotation2d()).rotateAround(new Translation2d(0, 0), ReefPole.getPoleFromID(camera_right.getAprilTagID(), camera_right).getRotation());     
      }

    align.setGeneralPosition(goalPose2d);
    align.setScoringPos(finalPose2d);
    align.updateBotpose();
    base.getRobotSpeeds().setSpeeds("feedback", align.calculateSpeeds()); 
  }

  @Override
  public void end(boolean interrupted) 
  {
    base.getRobotSpeeds().setSpeeds("feedback", new ChassisSpeeds()); 

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
