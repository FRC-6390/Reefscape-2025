// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.utils.ReefScoringPos.ReefPole;

public class DriveToPoint extends Command {
  public RobotBase<?> robotBase;
  public LimeLight limeLight;
  public boolean isDone;
  

  public DriveToPoint(RobotBase<?> robotBase) {
    this.robotBase = robotBase;
  }

  @Override
  public void initialize() 
  {
    isDone = false;
    limeLight = robotBase.getCameraFacing(ReefPole.getCenterReef());
  }

  @Override
  public void execute() 
  {
    if(limeLight.hasValidTarget())
    {
      isDone = true;

    }
  }

  @Override
  public void end(boolean interrupted) 
  {
    if(ReefPole.getPoleFromID(limeLight.getAprilTagID(),limeLight) != null){
    CommandScheduler.getInstance().schedule(new SequentialCommandGroup(AutoBuilder.followPath(ReefPole.getPoleFromID(limeLight.getAprilTagID(), limeLight).getPath())));
    
    }

  }

  @Override
  public boolean isFinished() {
    return isDone;
  }
}
