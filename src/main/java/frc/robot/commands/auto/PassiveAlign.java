// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import au.grapplerobotics.LaserCan;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.filters.FilteredValue;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.ReefScoringPos;
import frc.robot.utils.ReefScoringPos.ReefPole;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassiveAlign extends Command {

  public LimeLight limeLight;
 
  public RobotBase<?> base;
  public PIDController controller = new PIDController(0.0275, 0, 0);
  
  public FilteredValue rotationFiltered;
  
  /** Creates a new PassiveAlign. */
  public PassiveAlign(RobotBase<?> base) {
    this.base = base;
 

    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    limeLight = ReefScoringPos.getLimelightFacing(base);
    rotationFiltered =  new FilteredValue(() -> limeLight.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4])
    .addMovingAverage(10)
    .addMedianFilter(15);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    ReefPole pole = ReefPole.getPoleFromID(limeLight.getAprilTagID(), limeLight);
    if(pole != null)
    {
    if(limeLight.hasValidTarget())
    {
      double r = -controller.calculate(rotationFiltered.get(), 0);
      base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(0,0,r);
      SmartDashboard.putNumber("Rotational Vel", r);
      
    }
    else{
      base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(0,0,0);
    }
  }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    base.getDrivetrain().getRobotSpeeds().stopFeedbackSpeeds();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
