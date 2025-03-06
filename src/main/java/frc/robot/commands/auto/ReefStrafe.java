// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import au.grapplerobotics.LaserCan;
import ca.frc6390.athena.core.RobotBase;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.ReefScoringPos;
import frc.robot.utils.ReefScoringPos.ReefPole;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefStrafe extends Command {
  /** Creates a new ReefStrafe. */
  public LaserCan laserLeft;
  public LaserCan laserRight;
  public LaserCan curLas;
  public RobotBase<?> base;
  public double speed;
  public boolean isDone = false;

  public ReefStrafe(LaserCan laserLeft, LaserCan laserRight, RobotBase<?> base, double speed) 
  {
   this.laserLeft = laserLeft;
   this.laserRight = laserRight; 
   this.base = base;
   this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
   isDone = false; 
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(ReefScoringPos.getLimelightFacing(base).config.table() == "limelight-left")
    {
     curLas = laserLeft; 
    }
    else
    {
      curLas = laserRight;
    }

    if(curLas.getMeasurement() != null)
    {
      if(curLas.getMeasurement().distance_mm > 800)
      {
        base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(speed,0,0);
      }
      else
      {
        base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(0,0,0);
        base.getDrivetrain().getRobotSpeeds().stopFeedbackSpeeds();
        isDone = true;
      }
    }
}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(0,0,0);
    base.getDrivetrain().getRobotSpeeds().stopFeedbackSpeeds();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(DriverStation.isAutonomous())
    {
    return isDone;
    }
    else
    {
    return false;
    }
  }
}
