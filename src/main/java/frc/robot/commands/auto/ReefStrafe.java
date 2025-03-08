// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import au.grapplerobotics.LaserCan;
import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.ReefScoringPos;
import frc.robot.utils.ReefScoringPos.ReefPole;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ReefStrafe extends Command {
  /** Creates a new ReefStrafe. */
  public RobotBase<?> base;
  public LimeLight limeLight;
  public double speed;
  public boolean isDone = false;
  public  EnhancedXboxController  controller;
  public ReefStrafe(LaserCan laserLeft, LaserCan laserRight, RobotBase<?> base, double speed, EnhancedXboxController controller ) 
  {
   this.base = base;
   this.speed = speed;  this.controller = controller;
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

    limeLight = ReefScoringPos.getLimelightFacing(base);
    if(limeLight.getTargetHorizontalOffset() > 4)
    {
        base.getDrivetrain().getRobotSpeeds().setDriverSpeeds(speed,controller.getLeftY(),0);
    }
    else if(limeLight.getTargetHorizontalOffset() < 4)
    {
        base.getDrivetrain().getRobotSpeeds().setDriverSpeeds(-speed,controller.getLeftY(),0);
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
