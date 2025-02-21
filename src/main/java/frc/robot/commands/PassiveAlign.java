// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.filters.FilterList;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.AutoAlignHelper;
import frc.robot.utils.ReefScoringPos.ReefPole;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassiveAlign extends Command {

  public LimeLight limeLight;
  public RobotLocalization localization;
  public AutoAlignHelper helper;
  public RobotBase<?> base;
  public PIDController controller = new PIDController(0.025, 0, 0);
  public FilterList a = new FilterList().addMedianFilter(10);
  /** Creates a new PassiveAlign. */
  public PassiveAlign(RobotBase<?> base) {
    this.base = base;
    this.localization = base.getLocalization();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    limeLight = base.getCameraFacing(ReefPole.A.getTranslation());
    helper = new AutoAlignHelper(limeLight, localization, base.getDrivetrain());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(limeLight.hasValidTarget())
    {
      double r =  controller.calculate(-a.calculate(limeLight.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4]));   
      base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(0,0,r);
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
