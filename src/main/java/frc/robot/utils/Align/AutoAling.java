package frc.robot.utils.Align;

import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class AutoAling extends Command {
  public AlignHelper align;
  public Robot base;
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
    align.setRelativePose();
    if(align.calculateSpeeds() != null)
    {
    base.getRobotSpeeds().setSpeeds("feedback", align.calculateSpeeds()); 
    }
    align.shuffleboard();
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
