// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import frc.robot.Constants;

public class Elevator extends SubsystemBase{
  /** Creates a new Climber. */
  public CANcoder encoder;
  public TalonFX leftMotor;
  public TalonFX rightMotor;
  public boolean hasSetHome;
  public GenericLimitSwitch lowerlimitSwitch;
  public ShuffleboardTab tab;

  public ProfiledPIDController controller;
  public ElevatorFeedforward feedforward;

  public StateMachine<ElevatorState> stateMachine;
  public StatusSignal<Angle> getPosition;
  public StatusSignal<AngularVelocity> getVelocity;
  public SysIdRoutine routine;
  public double gear_ratio;
  
  public enum ElevatorState implements SetpointProvider {
    //ELEVATOR HEIGHT FROM FLOOR IN INCHES
    Home(Constants.Elevator.OFFSET_FROM_FLOOR),
    L1(Constants.Elevator.OFFSET_FROM_FLOOR),
    L2(32),
    L3(48),
    L4(72);


    double pos;
    private ElevatorState(double pos){
        this.pos = pos;
    }

    @Override
    public double getSetpoint() {
      return pos;
    }
}

double current = 40;
double nudge = 0;

  public Elevator() 
  {
   encoder = new CANcoder(Constants.Elevator.ENCODER, Constants.Elevator.CANBUS);
   leftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR, Constants.Elevator.CANBUS);
   rightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR, Constants.Elevator.CANBUS);
   
    if (encoder != null) {
      getPosition = encoder.getPosition();
      getVelocity = encoder.getVelocity();
      gear_ratio = Constants.Elevator.ENCODER_GEAR_RATIO;
    }else{
      getPosition = leftMotor.getRotorPosition();
      getVelocity = leftMotor.getRotorVelocity();
      gear_ratio = Constants.Elevator.MOTOR_GEAR_RATIO;
    }

    lowerlimitSwitch = new GenericLimitSwitch(Constants.Elevator.LIMIT_SWITCH);
    lowerlimitSwitch.onTrue(new InstantCommand(() -> {encoder.setPosition(0); stop();}));
    
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);

    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(current);
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    leftMotor.getConfigurator().apply(currentLimitsConfigs);
    rightMotor.getConfigurator().apply(currentLimitsConfigs);

    //CONTROL SYSTEM SETUP
    controller = Constants.Elevator.CONTORLLER;
    controller.setIntegratorRange(-1.5, 1.5);
    controller.setTolerance(0.2);
    controller.reset(getHeightFromFloor());
    feedforward = Constants.Elevator.FEEDFORWARD;
    stateMachine = new StateMachine<ElevatorState>(ElevatorState.Home, controller::atSetpoint);

  }

  public double getCurrentLimit(){
    return current;
  }
  
  public void setCurrentLimit(double current)
  {
    this.current = current;

    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(current);
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    leftMotor.getConfigurator().apply(currentLimitsConfigs);
    rightMotor.getConfigurator().apply(currentLimitsConfigs);
  }


  //POSITION IN INCHES
  public double getHeight()
  {
    return -(getPosition.getValueAsDouble() / gear_ratio) * Math.PI *  Constants.Elevator.GEAR_DIAMETER_INCHES;
  }

  public double getVel()
  {
    return -(getVelocity.getValueAsDouble() / gear_ratio) * Math.PI *  Constants.Elevator.GEAR_DIAMETER_INCHES;
  }

  public double getHeightFromFloor(){
    return getHeight() + Constants.Elevator.OFFSET_FROM_FLOOR;
  }

  public void voltageDrive(Voltage voltage)
  {
    leftMotor.setVoltage(-voltage.magnitude());
    rightMotor.setVoltage(-voltage.magnitude());
  }

  public StateMachine<ElevatorState> getStateMachine() {
    return stateMachine;
  }

  //MOVES ELEVATOR UP OR DOWN
  public void setMotors(double speed)
  {
    if (lowerlimitSwitch.getAsBoolean() && speed < 0){
      speed = 0;
    }
    //negative is up, this makes negative down
   
    speed = -speed;
    leftMotor.set(speed);
    rightMotor.set(speed);
  }
  
  public void stop() {
    setMotors(0);
  }

  public ShuffleboardTab shuffleboard(String tab) {
      return shuffleboard(Shuffleboard.getTab(tab));
  }

  public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
      tab.addBoolean("Lower Limit", lowerlimitSwitch::getAsBoolean);
      tab.addDouble("Elevator Height", this::getHeight).withPosition(1, 1);
      tab.addDouble("Elevator Height From Floor Inches", this::getHeightFromFloor).withPosition(2, 1);
      tab.addString("Setpoint", () -> stateMachine.getGoalState().name()).withPosition(3, 1);
      tab.addDouble("SetpoitnValue", () -> stateMachine.getGoalState().getSetpoint());
      tab.addString("Next State", () -> stateMachine.getNextState().name()).withPosition(4, 1);
      tab.addDouble("Fused Controller Output", () -> controller.calculate(getHeightFromFloor(),stateMachine.getGoalState().getSetpoint()) + feedforward.calculate(controller.getSetpoint().velocity) / 12);
      tab.addDouble("PID Output", () -> controller.calculate(getHeightFromFloor(), stateMachine.getGoalState().getSetpoint())).withPosition(5, 1);
      tab.addDouble("Feedforward Output", () -> feedforward.calculate(controller.getSetpoint().velocity));
      tab.addBoolean("State Changer", stateMachine.getChangeStateSupplier()).withPosition(6, 1);
      tab.addDouble("Profiled Pos Setpoint",() -> controller.getSetpoint().position);
      tab.addDouble("Profiled Vel Setpoint",() -> controller.getSetpoint().velocity);
      tab.addDouble("Motor Current Limit",this::getCurrentLimit);

      return tab;
  }

  public void nudge(double inches){
    this.nudge += inches;
  }

  public void resetNudge(){
    this.nudge = 0;
  }

  public void refresh(){
    getPosition.refresh();
    getVelocity.refresh();
    stateMachine.update();
  }

  public void update()
  {
    
    switch (stateMachine.getGoalState()) {
      case Home:
        setMotors(-0.1);
        resetNudge();
        break;
      case L1, L2, L3, L4:
        double speed = controller.calculate(getHeightFromFloor(),stateMachine.getGoalState().getSetpoint() + nudge) + feedforward.calculate(controller.getSetpoint().velocity) / 12;
        setMotors(speed);
    }
    
  }

  @Override
  public void periodic() {
    refresh();
    update();
  }
}