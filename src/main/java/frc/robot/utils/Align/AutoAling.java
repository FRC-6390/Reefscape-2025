package frc.robot.utils.Align;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAling extends Command {
  public AlignHelper align;
  public RobotBase<?> base;
  public AutoAling(AlignHelper align) {
    this.align = align;
    this.base = align.base;
  }

  public Pose2d calculateClosestPoint(double radius, Pose2d currentPose)
  {
    double x = (radius * currentPose.getX()) / (Math.sqrt((currentPose.getX() * currentPose.getX()) + (currentPose.getY() * currentPose.getY())));
    double y = (radius * currentPose.getY()) / (Math.sqrt((currentPose.getX() * currentPose.getX()) + (currentPose.getY() * currentPose.getY())));
    
    return new Pose2d(x, y, new Rotation2d());
  }

  @Override
  public void initialize() 
  {
    align.init();
  }

  @Override
  public void execute() 
  {    
    align.setId();
    align.setRelativePose();
    align.setGoal(calculateClosestPoint(1, base.getLocalization().getRelativePose()));
    if(align.calculateSpeeds() != null)
    {
    base.getRobotSpeeds().setSpeeds("feedback", align.calculateSpeeds()); 
    }
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
