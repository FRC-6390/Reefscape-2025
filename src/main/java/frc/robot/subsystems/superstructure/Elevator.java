// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
  /** Creates a new Climber. */

  public PIDController controller;
  public CANcoder encoder;
  public TalonFX leftMotor;
  public TalonFX rightMotor;
  public boolean hasSetHome;
  public GenericLimitSwitch lowerlimitSwitch;
  public ShuffleboardTab tab;

  public StateMachine<State> stateMachine;

  public enum State {
    //ELEVATOR HEIGHT FROM FLOOR IN INCHES
    StartConfiguration(12),
    Home(0),
    L1(0),
    L2(10),
    L3(20),
    L4(30),
    Feeder(15);

    double pos;
    private State(double pos){
        this.pos = pos;
    }

    public double get() {
        return pos;
    }
}

  public Elevator() 
  {
    encoder = new CANcoder(Constants.Elevator.ENCODER);
    leftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR);
    rightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR);
    
    lowerlimitSwitch = new GenericLimitSwitch(Constants.Elevator.LIMIT_SWITCH);
    lowerlimitSwitch.onPress(() ->  encoder.setPosition(0));

    controller = Constants.Elevator.CONTORLLER;
    controller.setTolerance(1);
    
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);

    tab = Shuffleboard.getTab("Elevator");
    stateMachine = new StateMachine<State>(State.Home, controller::atSetpoint);
  }

  //POSITION IN INCHES
  public double getHeight()
  {
    return (encoder.getPosition(true).getValueAsDouble() / Constants.Elevator.GEAR_RATIO) * Math.PI *  Constants.Elevator.GEAR_DIAMETER_INCHES;
  }

  public double getHeightFromFloor(){
    return getHeight() + Constants.Elevator.OFFSET_FROM_FLOOR;
  }

  public StateMachine<State> getStateMachine() {
    return stateMachine;
  }

  //MOVES ELEVATOR UP OR DOWN
  private void setMotors(double speed)
  {
    if (lowerlimitSwitch.isPressed() && speed < 0){
      speed = 0;
    }
    leftMotor.set(speed);
    rightMotor.set(-speed);
  }
  
  public void shuffleboard()
  {
    tab.add("Lower Limit", lowerlimitSwitch.isPressed());
    tab.add("Elevator Height", getHeight());
    tab.add("Elevator Height From Floor Inches", getHeightFromFloor());
    tab.add("Setpoint", stateMachine.getGoalState());
    tab.add("PID Output", controller.calculate(getHeightFromFloor(), stateMachine.getGoalState().get()));
  }

  public void update()
  {
    shuffleboard();
    switch (stateMachine.getGoalState()) {
      case Home:
        setMotors(-0.5);
        break;
      case Feeder, L1, L2, L3, L4, StartConfiguration:
        double speed = controller.calculate(getHeightFromFloor(), stateMachine.getGoalState().get());
        setMotors(speed);
    }
  }

  @Override
  public void periodic() {
      update();
  }
}
