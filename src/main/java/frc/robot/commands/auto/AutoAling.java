// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import ca.frc6390.athena.core.RobotBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.GeneralAlign;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoAling extends Command {
  /** Creates a new AutoAling. */
  public GeneralAlign align;
  public RobotBase<?> base;
  public AutoAling(GeneralAlign align, RobotBase<?> base) {
    this.align = align;
    this.base = base;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
    align.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    align.setTagId();
    align.getRobotPositionRelativeToTag(1, 1, true);
    base.getRobotSpeeds().setSpeeds("feedback", align.calculateSpeeds(new Pose2d(3, 0, new Rotation2d()), new Pose2d(4,1, new Rotation2d()))); 
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
