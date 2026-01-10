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

  @Override
  public void initialize() 
  {
    align.init();
  }

  @Override
  public void execute() 
  {

    LimeLight camera_left = base.getVision().getLimelight("limelight-left");
    LimeLight camera_right = base.getVision().getLimelight("limelight-right");
    

    align.setId();
    align.setRelativePose();
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
