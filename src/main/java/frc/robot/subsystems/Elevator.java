// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class Elevator extends SubsystemBase {
  /** Creates a new Climber. */

  public double setpoint;
  public PIDController controller;
  public CANcoder encoder;
  public TalonFX leftMotor;
  public TalonFX rightMotor;
  public boolean hasSetHome;
  public DigitalInput lowerlimitSwitch;
  public DigitalInput upperlimitSwitch;
  public ShuffleboardTab tab;
  

  public Elevator() 
  {
    encoder = new CANcoder(Constants.Elevator.ENCODER);
    leftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR);
    rightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR);
    lowerlimitSwitch = new DigitalInput(Constants.Elevator.LOWER_LIMIT_SWITCH);
    upperlimitSwitch = new DigitalInput(Constants.Elevator.UPPER_LIMIT_SWITCH);
    controller = new PIDController(0.1, 0, 0);
    tab = Shuffleboard.getTab("Elevator");
  }

  //POSITION IN CM
  public double getPositionInCM()
  {
    return encoder.getPosition().refresh().getValueAsDouble() / Constants.Elevator.ROTATION_TO_CM;
  }

  //SETS THE SETPOINT OF THE PID CONTROLLER, WHICH ALWAYS RUNS
  //POSITION SHOULD BE IN CM
  public void setPosition(Constants.Elevator.Position position)
  {
    setpoint = position.getPosition();
  }

  //MOVES ELEVATOR UP OR DOWN
  public void moveElevator(double speed)
  {
    leftMotor.set(speed);
    rightMotor.set(-speed);
  }
  
  public void homeEncoder()
  {
    encoder.setPosition(0);
  }

  public void homeElevator()
  {
    hasSetHome = false;
  }

  public void shuffleboard()
  {
    tab.add("Upper Limit",upperlimitSwitch.get());
    tab.add("Lower Limit", lowerlimitSwitch.get());
    tab.add("Encoder Rotations", encoder.getPosition().refresh().getValueAsDouble());
    tab.add("Elevator Height CM", getPositionInCM());
    tab.add("Setpoint", setpoint);
    tab.add("PID Output", controller.calculate(getPositionInCM(), setpoint));
  }

  public void update()
  {
    if(!hasSetHome)
    {
      moveElevator(-0.5);
      if(!lowerlimitSwitch.get())
      {
        homeEncoder();
        hasSetHome = true;
      }
    }
    else
    {
      double speed = controller.calculate(getPositionInCM(), setpoint);
      if(!lowerlimitSwitch.get() && speed < 0)
      {
        speed = 0;
      }
      else if(!upperlimitSwitch.get() && speed > 0)
      {
        speed = 0;
      }
      moveElevator(speed);
    }
  }
  
  @Override
  public void periodic() 
  {
    update();
    shuffleboard();
  }
}
