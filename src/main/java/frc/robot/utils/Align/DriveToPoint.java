package frc.robot.utils.Align;

import ca.frc6390.athena.core.RobotCore;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Align.RelevantPosition.AlignMode;

public class DriveToPoint extends Command {
  public RelevantPosition align;
  public RobotCore<?> base;
  public PIDController rController;
  public PIDController xController;
  public PIDController yController;

  public DriveToPoint(RelevantPosition align, PIDController rController, PIDController xController, PIDController yController) {
    this.align = align;
    this.xController = xController;
    this.yController = yController;
    this.rController = rController;
    this.base = align.base;
  }

  @Override
  public void initialize() 
  {
  }

  @Override
  public void execute() 
  {    
    align.setRelativePose();
    if(align.calculateSpeeds(rController, xController, yController, AlignMode.PARALLEL) != null)
    {
    base.getRobotSpeeds().setSpeeds("feedback", align.calculateSpeeds(rController, xController, yController, AlignMode.PARALLEL)); 
    }
  }

  @Override
  public void end(boolean interrupted) 
  {
    base.getRobotSpeeds().setSpeeds("feedback", new ChassisSpeeds()); 
    
  }

  @Override
  public boolean isFinished() {
    return xController.atSetpoint() && yController.atSetpoint();
  }
}
