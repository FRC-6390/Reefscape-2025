// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanisms;

import au.grapplerobotics.LaserCan;
import ca.frc6390.athena.controllers.DelayedOutput;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.superstructure.Elevator.ElevatorState;
import frc.robot.subsystems.superstructure.EndEffector.EndEffectorState;
import frc.robot.utils.ReefScoringPos.ReefPole;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Elevate extends Command {

  public ElevatorState state;
  public Superstructure superstructure;
  public LimeLight limeLight;
  public LaserCan lasLeft;
  public LaserCan lasRight;
  public LaserCan currentLas;
  public RobotBase<?> base;
  public boolean hasSeen;
  public Translation2d translation = new Translation2d();
  
  public double dist;
  public double distToTag;
  
  /** Creates a new Elevate. */
  public Elevate(ElevatorState state,LaserCan lasLeft, LaserCan lasRight, Superstructure superstructure, RobotBase<?> base) {
    this.base = base;
    this.state = state;
    this.superstructure = superstructure;
    this.lasLeft = lasLeft;
    this.lasRight = lasRight;
   
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    limeLight = base.getCameraFacing(ReefPole.A.getTranslation());
    
   
    
    // output = new DelayedOutput(this::closeEnough, 0.25);
    hasSeen = false;
    dist = 99999;
    distToTag = 9999;
  }

  

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    SmartDashboard.putNumber("Las Left", lasLeft.getMeasurement().distance_mm);
    SmartDashboard.putNumber("Las Right", lasRight.getMeasurement().distance_mm);
    SmartDashboard.putString("Camera Facing", limeLight.config.table());
    SmartDashboard.putNumber("Laser Facing", currentLas.getMeasurement().distance_mm);
    SmartDashboard.putNumber("DistToTag", distToTag);

    //END EFFECTOR AUTOMATION
    if(limeLight.config.table() == "limelight-left")
    {
      currentLas = lasLeft;
      if(state.equals(ElevatorState.L4))
      {
        superstructure.endEffectorStateManager(EndEffectorState.LeftL4);
      }
      else
      {
        superstructure.endEffectorStateManager(EndEffectorState.Left);
      }
    }
    if(limeLight.config.table() == "limelight-right")
    {
      currentLas = lasRight;
      if(state.equals(ElevatorState.L4))
      {
        superstructure.endEffectorStateManager(EndEffectorState.RightL4);
      }
      else
      {
        superstructure.endEffectorStateManager(EndEffectorState.Right);
      }
    }

    //DATA GATHERING
    if(limeLight.hasValidTarget())
    {
      if(ReefPole.getPoleFromID((int)limeLight.getAprilTagID()) != null)
      {
      translation = ReefPole.getPoleFromID((int)limeLight.getAprilTagID()).getTranslation();
      distToTag = Math.abs(limeLight.getPoseEstimate(PoseEstimateType.BOT_POSE_TARGET_SPACE).getPose().getY());
      }
      else
      {
        distToTag = 9999;
      }
    }

    //ELEVATOR AUTOMATION
    if(currentLas.getMeasurement() != null)
    {
    if(distToTag < 0.35 && currentLas.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && currentLas.getMeasurement().distance_mm < 800)
    {
      superstructure.elevatorStateManager(state);
      hasSeen = false;      
    }
    else if(distToTag > 0.35 || currentLas.getMeasurement().status != LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT || currentLas.getMeasurement().distance_mm > 800)
    {
      superstructure.elevatorStateManager(ElevatorState.Feeder);
    }
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
