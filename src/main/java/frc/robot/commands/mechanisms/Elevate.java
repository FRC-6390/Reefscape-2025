// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanisms;

import au.grapplerobotics.LaserCan;
import ca.frc6390.athena.controllers.DelayedOutput;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.utils.ReefScoringPos.ReefPole;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Elevate extends Command {

  public ElevatorState state;
  public Superstructure superstructure;
  public LimeLight limeLight;
  public LaserCan las;
  public RobotBase<?> base;
  public double xOffset;
  public boolean hasSeen;
  public double area;
  public Translation2d translation = new Translation2d();
  public DelayedOutput output;
  public double dist;
  
  /** Creates a new Elevate. */
  public Elevate(ElevatorState state,LaserCan las, Superstructure superstructure, RobotBase<?> base) {
    this.base = base;
    this.state = state;
    this.superstructure = superstructure;
    this.las =las;
   
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    limeLight = base.getCameraFacing(ReefPole.A.getTranslation());
    
    output =  new DelayedOutput(this::closeEnough, 0.85
    );
    
    // output = new DelayedOutput(this::closeEnough, 0.25);
    hasSeen = false;
    xOffset = 0;
    dist = 99999;
    area = 0;
  }

  public boolean closeEnough()
  {
    dist = base.getLocalization().getFieldPose().getTranslation().getDistance(translation);
  
    if(dist < 0.3)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    SmartDashboard.putNumber("Tag Area", area);
    SmartDashboard.putNumber("Las", las.getMeasurement().distance_mm);
    SmartDashboard.putNumber("Offset", xOffset);
    SmartDashboard.putBoolean("haSeen", hasSeen);
    // SmartDashboard.putBoolean("AHH", output.getAsBoolean());

    if(limeLight.hasValidTarget())
    {
      xOffset = limeLight.getTargetHorizontalOffset();
      area = limeLight.getTargetArea();
      if(ReefPole.getPoleFromID((int)limeLight.getAprilTagID()) != null)
      {
      translation = ReefPole.getPoleFromID((int)limeLight.getAprilTagID()).getTranslation();
      }
      if(!hasSeen)
      { 
        hasSeen = true;
      }
    }

    if(las.getMeasurement() != null)
    {
    if(hasSeen && las.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && las.getMeasurement().distance_mm < 800)
    {
      superstructure.elevatorStateManager(state);
      hasSeen = false;      
    }
    else if(las.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && las.getMeasurement().distance_mm > 800)
    {
      superstructure.elevatorStateManager(ElevatorState.Feeder);
    }
    }

    if(output.getAsBoolean())
    {
      superstructure.ejectPiece(1);
    }
    else
    {
      superstructure.ejectPiece(0);
    }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
