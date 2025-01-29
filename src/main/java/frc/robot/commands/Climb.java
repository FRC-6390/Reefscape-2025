// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import ca.frc6390.athena.controllers.DebouncedController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Climb extends Command {
  /** Creates a new Climb. */
  public Climber climber;
  public DebouncedController controller;
  
  public Climb(Climber climber, DebouncedController controller) {
    this.climber = climber;
    this.controller = controller;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() 
  {
    climber.setSpeed(controller.getLeftY() / 2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) 
  {
    climber.setSpeed(0);
    climber.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
