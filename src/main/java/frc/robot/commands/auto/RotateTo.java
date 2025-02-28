// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.core.RobotLocalization;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateTo extends Command {
  
  public RobotBase<?> base;
  public Rotation2d angle;
  public boolean isDone = false;
  /** Creates a new RotateTo. */
  public RotateTo(RobotBase<?> base, Rotation2d angle) {
    this.base = base;
    this.angle = angle;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    SmartDashboard.putNumber("Rotation",  base.getLocalization().getFieldPose().getRotation().getDegrees());
    SmartDashboard.putNumber("ANGLE DES",  angle.getDegrees());
    SmartDashboard.putBoolean("IsDone", isDone);
    if
    (
      base.getLocalization().getFieldPose().getRotation().getDegrees() < angle.plus(Rotation2d.fromDegrees(5)).getDegrees()
      &&
      base.getLocalization().getFieldPose().getRotation().getDegrees() > angle.minus(Rotation2d.fromDegrees(5)).getDegrees()
    )
    {
      isDone = true;
    }
    else
    {
      base.getDrivetrain().getRobotSpeeds().setAutoSpeeds(0, 0, 1);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    base.getDrivetrain().getRobotSpeeds().setAutoSpeeds(0, 0, 0);
    base.getDrivetrain().getRobotSpeeds().stopAutoSpeeds();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
