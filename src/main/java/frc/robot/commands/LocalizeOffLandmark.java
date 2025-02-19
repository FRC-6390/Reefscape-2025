// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import au.grapplerobotics.LaserCan;
import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.ReefScoringPos.ReefPole;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LocalizeOffLandmark extends Command {
  /** Creates a new LocalizeOffLandmark. */
  public LimeLight limeLight;
  public RobotBase<?> base;
  public LaserCan las;
  public RobotLocalization localization;
  public boolean isDone;

  public LocalizeOffLandmark(RobotBase<?> base, LaserCan las) {
    this.las = las;
    this.base = base;
    this.localization = base.getLocalization();
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    isDone = false;
    limeLight = base.getCameraFacing(ReefPole.A.getTranslation());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    if(las.getMeasurement() != null)
    {
    if(limeLight.hasValidTarget() && limeLight.getTargetHorizontalOffset() < 15 && las.getMeasurement().status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT && las.getMeasurement().distance_mm <= 400)
    {
      localization.resetFieldPose(new Pose2d(ReefPole.getPoleFromID((int)limeLight.getAprilTagID()).getTranslation(), localization.getFieldPose().getRotation()));
      isDone = true;
    }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
