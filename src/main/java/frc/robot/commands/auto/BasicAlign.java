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
public class BasicAlign extends Command {

  public LimeLight limeLight;
 
  public RobotBase<?> base;
  public PIDController controller = new PIDController(0.03, 0, 0);
  
  
  /** Creates a new PassiveAlign. */
  public BasicAlign(RobotBase<?> base, String llTable) {
    this.base = base;
    limeLight = this.base.getVision().getLimelight(llTable);

    // Use addRequirements() here to declare subsystem dependencies.
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
   
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
      double r = controller.calculate(limeLight.getTargetHorizontalOffset(), 0);
      base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(0,r,0);
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
