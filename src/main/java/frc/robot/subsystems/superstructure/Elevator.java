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
import frc.robot.Constants;

public class Elevator extends SubsystemBase{

  private final CANcoder encoder;
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  private final GenericLimitSwitch lowerlimitSwitch;

  public final ProfiledPIDController controller;
  private final ElevatorFeedforward feedforward;

  private final StateMachine<Double, ElevatorState> stateMachine;
  private final StatusSignal<Angle> getPosition;
  private final StatusSignal<AngularVelocity> getVelocity;
  private double nudge = 0;
  
  public enum ElevatorState implements SetpointProvider<Double> {
    //ELEVATOR HEIGHT FROM FLOOR IN INCHES
    Home(Constants.Elevator.OFFSET_FROM_FLOOR),
    L1(Constants.Elevator.OFFSET_FROM_FLOOR),
    AlgaeHigh(54.66734391140974),
    AlgaeLow(37.60104065578804),
    //31.5
    L2(32.78280700103924),
    //47.25
    L3(48),
    //72
    L4(77.23066732041963);


    double pos;
    private ElevatorState(double pos){
        this.pos = pos;
    }

    @Override
    public Double getSetpoint() {
      return pos;
    }
}

  public Elevator() 
  {
    encoder = new CANcoder(Constants.Elevator.ENCODER, Constants.CANIVORE_CANBUS);
    leftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR, Constants.CANIVORE_CANBUS);
    rightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR, Constants.CANIVORE_CANBUS);
    getPosition = encoder.getPosition();
    getVelocity = encoder.getVelocity();
    lowerlimitSwitch = new GenericLimitSwitch(Constants.Elevator.LIMIT_SWITCH);
    lowerlimitSwitch.onTrue(new InstantCommand(() -> {encoder.setPosition(0); stop();}));
    
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);

    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(43);
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
    leftMotor.getConfigurator().apply(currentLimitsConfigs);
    rightMotor.getConfigurator().apply(currentLimitsConfigs);

    //CONTROL SYSTEM SETUP
    controller = Constants.Elevator.CONTORLLER;
    controller.setIntegratorRange(-1.5, 1.5);
    controller.setTolerance(0.8);
    controller.reset(getHeightFromFloor());
    feedforward = Constants.Elevator.FEEDFORWARD;
    stateMachine = new StateMachine<Double, ElevatorState>(ElevatorState.Home, controller::atSetpoint);

  }

  public double getHeight()
  {
    return -(getPosition.getValueAsDouble() / Constants.Elevator.ENCODER_GEAR_RATIO) * Math.PI *  Constants.Elevator.GEAR_DIAMETER_INCHES;
  }

  public double getVelocity()
  {
    return -(getVelocity.getValueAsDouble() / Constants.Elevator.ENCODER_GEAR_RATIO) * Math.PI *  Constants.Elevator.GEAR_DIAMETER_INCHES;
  }

  public double getHeightFromFloor(){
    return getHeight() + Constants.Elevator.OFFSET_FROM_FLOOR - nudge;
  }

  public void voltageDrive(Voltage voltage)
  {
    leftMotor.setVoltage(-voltage.magnitude());
    rightMotor.setVoltage(-voltage.magnitude());
  }

  public StateMachine<Double, ElevatorState> getStateMachine() {
    return stateMachine;
  }

  public void reset(){
    controller.reset(getHeightFromFloor(), getVelocity());
  }

  //MOVES ELEVATOR UP OR DOWN
  public void setMotors(double speed)
  {
    if(encoder != null)
    {
    if (lowerlimitSwitch.getAsBoolean() && speed < 0){
      speed = 0;
    }
    //negative is up, this makes negative down
   
    speed = -speed;
    leftMotor.set(speed);
    rightMotor.set(speed);
  }
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
      tab.addDouble("Nudge",() -> nudge);

      return tab;
  }

  public void nudge(double inches){
    nudge += inches;
  }

  public void resetNudge(){
    nudge = 0;
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
      case L1, L2, L3, L4, AlgaeHigh, AlgaeLow:
        double speed = controller.calculate(getHeightFromFloor(),stateMachine.getGoalState().getSetpoint()) + feedforward.calculate(controller.getSetpoint().velocity) / 12;
        setMotors(speed);
    }
    
  }

  @Override
  public void periodic() {
    refresh();
    update();
  }
}