// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.TargetCorner;

import ca.frc6390.athena.controllers.DelayedOutput;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotSpeeds.SpeedAxis;
import ca.frc6390.athena.core.RobotSpeeds.SpeedSource;
import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoPickup extends Command {
  /** Creates a new AutoPickup. */
  public PhotonCamera camera;
  public RobotBase<?> base;
  public DelayedOutput end;
  public boolean hasSet = false;
  public PIDController rController = new PIDController(0.9, 0, 0);
  public PIDController xController = new PIDController(0.1, 0, 0);
  public AutoPickup(RobotBase<?> base) 
  {
    this.base = base; 
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    camera = new PhotonCamera("CoralCam");
    end = new DelayedOutput(() -> endCommand(), 0.75);
    hasSet = false;
  }

  public boolean endCommand()
  {
    boolean targetVisible = false;
    double targetYaw = 0;
    double targetPitch = 0;
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) 
    {
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) 
        {
            var target = result.getTargets().get(0);
            targetYaw = target.getYaw();
            targetPitch = target.getPitch();
            targetVisible = true;  
        }
        else{
          targetVisible = false;
        }
    }

    if(targetYaw < 3 && targetPitch < 3 && targetVisible)
    {
      return true;
    }

    if(hasSet && !targetVisible)
    {
      return true;
    }

    return false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    boolean targetVisible = false;
    double targetYaw = 0;
    double targetPitch = 0;
    var results = camera.getAllUnreadResults();
    if (!results.isEmpty()) 
    {
        hasSet = true;
        var result = results.get(results.size() - 1);
        if (result.hasTargets()) 
        {
            
            var target = result.getTargets().get(0);
            List<TargetCorner> corners = target.getMinAreaRectCorners();
            double len1 = Math.sqrt(Math.pow((corners.get(0).x - corners.get(1).x),2) + (Math.pow(corners.get(0).y - corners.get(1).y,2)));
            double len2 = Math.sqrt(Math.pow((corners.get(0).x - corners.get(2).x),2) + (Math.pow(corners.get(0).y - corners.get(2).y,2)));
            double len3 = Math.sqrt(Math.pow((corners.get(0).x - corners.get(3).x),2) + (Math.pow(corners.get(0).y - corners.get(3).y,2)));
            double[] lens = {len1, len2, len3};
            
            double max = Arrays.stream(lens).max().getAsDouble();
            lens = Arrays.stream(lens).filter((val) -> !(val == max)).toArray();

            double biggest = lens[0] > lens[1] ? lens[0] : lens[1];
            double smallest = lens[0] < lens[1] ? lens[0] : lens[1];
            SmartDashboard.putNumber("Ratio", smallest / biggest);            
            SmartDashboard.putNumber("Len1", len1);
            SmartDashboard.putNumber("Len2", len2);
            SmartDashboard.putNumber("CORNER 0 X",  corners.get(0).x);
            SmartDashboard.putNumber("CORNER 1 X",  corners.get(1).x);
            SmartDashboard.putNumber("CORNER 2 X",  corners.get(2).x);
            SmartDashboard.putNumber("CORNER 3 X",  corners.get(3).x);
            SmartDashboard.putNumber("CORNER 0 Y",  corners.get(0).y);
            SmartDashboard.putNumber("CORNER 1 Y",  corners.get(1).y);
            SmartDashboard.putNumber("CORNER 2 Y",  corners.get(2).y);
            SmartDashboard.putNumber("CORNER 3 Y",  corners.get(3).y);



            targetYaw = target.getYaw();
            targetPitch = target.getPitch();
            targetVisible = true;  
        }
        else{
          targetVisible = false;
        }
    }

    if(targetVisible)
    {
    base.getDrivetrain().getRobotSpeeds().enableSpeeds(SpeedSource.AUTO, false);
    base.getDrivetrain().getRobotSpeeds().stopAutoSpeeds();

    double rSpeed = rController.calculate(targetYaw, 0);
    double xSpeed = -xController.calculate(targetPitch, 0);

    // base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(xSpeed, 0, rSpeed);
    }
    else
    {
      base.getDrivetrain().getRobotSpeeds().enableSpeeds(SpeedSource.AUTO, true);
    }
    // else
    // {
    //   base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(0.5, 0, 0);

    // }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(0,0,0);
  }


  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;//end.getAsBoolean();
  }
}
