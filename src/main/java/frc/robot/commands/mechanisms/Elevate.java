// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.mechanisms;

import au.grapplerobotics.LaserCan;
import ca.frc6390.athena.commands.RunnableTrigger;
import ca.frc6390.athena.controllers.DelayedOutput;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.superstructure.Elevator;
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
  public DelayedOutput output;
  public RobotBase<?> base;
  public Translation2d translation = new Translation2d();
  public Elevator elevator;
  
  public double distToTag;

  public RunnableTrigger limelightLeft;
  public RunnableTrigger elevate;
  public RunnableTrigger limelightRight;
  public RunnableTrigger eject;
  public Elevate(ElevatorState state, LaserCan lasLeft, LaserCan lasRight, Superstructure superstructure, RobotBase<?> base, Elevator elevator) {
    this.base = base;
    this.superstructure = superstructure;
    this.state = state;
    this.lasLeft = lasLeft;
    this.lasRight = lasRight;
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    output = new DelayedOutput(superstructure::closeEnough, 0.25);
    limeLight = null;
    distToTag = 9999;
    
    limelightLeft = new RunnableTrigger(() -> isLimelight("limelight-left"));
    limelightRight = new RunnableTrigger(() -> isLimelight("limelight-right"));
    eject = new RunnableTrigger(() -> output.getAsBoolean() && DriverStation.isAutonomous());
    elevate = new RunnableTrigger(() -> distToTag < 0.35 && currentLas.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && currentLas.getMeasurement().distance_mm < 800);
    
    limelightLeft.and(() -> elevator.stateMachine.atState(ElevatorState.L4)).onTrue(superstructure.setEndEffector(EndEffectorState.LeftL4));
    limelightLeft.onTrue(() -> currentLas = lasLeft);
    limelightLeft.and(() -> !elevator.stateMachine.atState(ElevatorState.L4)).onTrue(superstructure.setEndEffector(EndEffectorState.Left));

    limelightRight.and(() -> elevator.stateMachine.atState(ElevatorState.L4)).onTrue(superstructure.setEndEffector(EndEffectorState.RightL4));
    limelightRight.and(() -> !elevator.stateMachine.atState(ElevatorState.L4)).onTrue(superstructure.setEndEffector(EndEffectorState.Right));
    limelightRight.onTrue(() -> currentLas = lasRight);
    

    limelightRight.onTrue(superstructure.setEndEffector(EndEffectorState.Right));
    eject.onTrue(() -> superstructure.ejectPiece(1)).onFalse(() -> superstructure.ejectPiece(0));
    elevate.onTrue(superstructure.setElevator(state)).onFalse(Commands.sequence(superstructure.setElevator(ElevatorState.Home), superstructure.setEndEffector(EndEffectorState.Home)));

  }


  
  public boolean isLimelight(String table)
  {
    if(limeLight == null)
    {
      return false;
    }
    else{
      return limeLight.config.table() == table;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    limeLight = base.getCameraFacing(ReefPole.getCenterReef());

    //DATA GATHERING
    if(limeLight.hasValidTarget())
    {
      if(ReefPole.getPoleFromID(limeLight.getAprilTagID(), limeLight) != null)
      {
      translation = ReefPole.getPoleFromID(limeLight.getAprilTagID(), limeLight).getTranslation();
      distToTag = Math.abs(limeLight.getPoseEstimate(PoseEstimateType.BOT_POSE_TARGET_SPACE).getPose().getY());
      }
      else
      {
        distToTag = 9999;
      }
    }

    SmartDashboard.putNumber("Las Left", lasLeft.getMeasurement().distance_mm);
    SmartDashboard.putNumber("Las Right", lasRight.getMeasurement().distance_mm);
    SmartDashboard.putString("Camera Facing", limeLight.config.table());
    // SmartDashboard.putNumber("Laser Facing", currentLas.getMeasurement().distance_mm);
    SmartDashboard.putNumber("DistToTag", distToTag);
    SmartDashboard.putNumber("DistToTag", distToTag);
    SmartDashboard.putString("LL name", limeLight.config.table());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public void setState(ElevatorState stateA) {
      state = stateA;
  }
}
