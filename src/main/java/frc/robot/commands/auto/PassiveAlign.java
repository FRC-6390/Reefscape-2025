// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import au.grapplerobotics.LaserCan;
import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.filters.FilterList;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.AutoAlignHelper;
import frc.robot.utils.ReefScoringPos.ReefPole;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class PassiveAlign extends Command {

  public LimeLight limeLight;
  public RobotLocalization localization;
  public AutoAlignHelper helper;
  public LaserCan las;
  public EnhancedXboxController drcontroller;
  public RobotBase<?> base;
  public PIDController controller = new PIDController(0.025, 0, 0);
  public PIDController rController = new PIDController(0.1, 0, 0);
 public ProfiledPIDController xController = new ProfiledPIDController(0.01, 0, 0.0, new Constraints(2, 1.5));
  public FilterList a = new FilterList().addMedianFilter(1);
  LinearFilter filter = LinearFilter.movingAverage(5);
  public FilterList b = new FilterList().addMovingAverage(10);
  
  /** Creates a new PassiveAlign. */
  public PassiveAlign(RobotBase<?> base, LaserCan las, EnhancedXboxController drcontroller) {
    this.base = base;
    this.localization = base.getLocalization();
    this.las = las;
    this.drcontroller= drcontroller;
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
      double r =  controller.calculate(-filter.calculate(a.calculate(limeLight.getPoseEstimate(PoseEstimateType.TARGET_POSE_ROBOT_SPACE).getRaw()[4])), 0);   
      double l = b.calculate(-limeLight.getTargetHorizontalOffset()); //limeLight.getPoseEstimate(PoseEstimateType.BOT_POSE_TARGET_SPACE).getPose().getY()
      double x = xController.calculate(l, 0);
      SmartDashboard.putNumber("AHH", l);
      base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(0,x,0);
      // if(limeLight.getTargetHorizontalOffset() < 50 && las.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT)
      // {
      //   if(las.getMeasurement().distance_mm <  2000)
      //   {
      //   base.getDrivetrain().getRobotSpeeds().setDriverSpeeds(0,x,r);
      //   }
      // }
    }
    else{
      base.getDrivetrain().getRobotSpeeds().setFeedbackSpeeds(0,0,0);
      
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
