// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volt;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants;

public class Elevator extends SubsystemBase{
  /** Creates a new Climber. */

  public PIDController controller;
  public CANcoder encoder;
  public TalonFX leftMotor;
  public TalonFX rightMotor;
  public boolean hasSetHome;
  public GenericLimitSwitch lowerlimitSwitch;
  public ElevatorFeedforward feedforward;
  public ShuffleboardTab tab;

  public StateMachine<ElevatorState> stateMachine;
  public StatusSignal<Angle> getPosition;
  public StatusSignal<AngularVelocity> getVelocity;
  public SysIdRoutine routine;
  public double gear_ratio;
  public enum ElevatorState {
    //ELEVATOR HEIGHT FROM FLOOR IN INCHES
    StartConfiguration(0),
    Home(0),
    L1(6),
    L2(10),
    L3(20),
    L4(30),
    Feeder(14);


    double pos;
    private ElevatorState(double pos){
        this.pos = pos;
    }

    public double get() {
        return pos;
    }
}

public void logMotors(SysIdRoutineLog log)
{
System.out.println(log);
}
  public Elevator() 
  {
   encoder = new CANcoder(Constants.Elevator.ENCODER, Constants.Elevator.CANBUS);
    leftMotor = new TalonFX(Constants.Elevator.LEFT_MOTOR, Constants.Elevator.CANBUS);
    rightMotor = new TalonFX(Constants.Elevator.RIGHT_MOTOR, Constants.Elevator.CANBUS);
    feedforward = Constants.Elevator.FEEDFORWARD;
    routine = 
      new SysIdRoutine(
          // Empty config defaults to 1 volt/second ramp rate and 7 volt step voltage.
          new SysIdRoutine.Config(),
          new SysIdRoutine.Mechanism(
              // Tell SysId how to plumb the driving voltage to the motor(s).
              this::voltageDrive,
              // Tell SysId how to record a frame of data for each motor on the mechanism being
              // characterized.
              log -> {
                // Record a frame for the shooter motor.
                log.motor("left-motor")
                    .voltage(leftMotor.getMotorVoltage().getValue())
                    .linearVelocity(LinearVelocity.ofBaseUnits(getVel(), Units.InchesPerSecond))
                    .linearPosition(Distance.ofBaseUnits(getHeight(), Units.Inches));
              },
              this));
    if (encoder != null) {
      getPosition = encoder.getPosition();
      getVelocity = encoder.getVelocity();
      gear_ratio = Constants.Elevator.ENCODER_GEAR_RATIO;
    }else{
      getPosition = leftMotor.getRotorPosition();
      gear_ratio = Constants.Elevator.MOTOR_GEAR_RATIO;
    }

    lowerlimitSwitch = new GenericLimitSwitch(Constants.Elevator.LIMIT_SWITCH);
    lowerlimitSwitch.getTrigger().whileTrue(new InstantCommand(() -> {encoder.setPosition(0); stop();}));

    controller = Constants.Elevator.CONTORLLER;
    controller.setIntegratorRange(-1, 1);
    controller.setTolerance(0.2);
    
    leftMotor.setNeutralMode(NeutralModeValue.Brake);
    rightMotor.setNeutralMode(NeutralModeValue.Brake);

    stateMachine = new StateMachine<ElevatorState>(ElevatorState.Home, controller::atSetpoint);
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  return routine.dynamic(direction);
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
    //negative is up, this makes negative down
    
    if (lowerlimitSwitch.isPressed() && speed < 0){
      speed = 0;
    }
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
      tab.addBoolean("Lower Limit", () -> lowerlimitSwitch.isPressed());
      tab.addDouble("Elevator Height", this::getHeight).withPosition(1, 1);
      tab.addDouble("Elevator Height From Floor Inches", this::getHeightFromFloor).withPosition(2, 1);
      tab.addString("Setpoint", () -> stateMachine.getGoalState().name()).withPosition(3, 1);
      tab.addDouble("SetpoitnValue", () -> stateMachine.getGoalState().get());
      tab.addString("Next State", () -> stateMachine.getNextState().name()).withPosition(4, 1);
      tab.addDouble("PID Output", () -> controller.calculate(getHeightFromFloor(), stateMachine.getGoalState().get())).withPosition(5, 1);
      tab.addBoolean("State Changer", stateMachine.getChangeStateSupplier()).withPosition(6, 1);

      return tab;
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
        break;
      case Feeder, L1, L2, L3, L4, StartConfiguration:
        double speed = controller.calculate(getHeightFromFloor(), stateMachine.getGoalState().get()) + feedforward.calculate(1) / 12;
        setMotors(speed);
    }
    
  }

  @Override
  public void periodic() {
    refresh();
    update();
  }
}

