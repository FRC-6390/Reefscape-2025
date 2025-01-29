// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.Climber;
import frc.robot.subsystems.superstructure.Elevator;
import frc.robot.subsystems.superstructure.Climber.STATE;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
  // Elevator elevator;
  Climber climber;

  public Superstructure(Climber climber) 
  {
    this.climber = climber;
  }

  public void setClimberState(STATE state){

  }


  @Override
  public void periodic() {
    climber.update();
    // elevator.update();
    // This method will be called once per scheduler run
  }
}
