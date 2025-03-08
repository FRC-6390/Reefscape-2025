// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.config.PIDConstants;

import ca.frc6390.athena.devices.Encoder;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.superstructure.endeffector.Rotator.RotatorState;
import frc.robot.utils.ReefScoringPos;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  public TalonFX motorLeft;
  public TalonFX motorRight;  
  public double setpoint = 0;
  public CANcoder encoder;
  public PIDController controller;
  // public GenericLimitSwitch limitSwitch;
  public StatusSignal<Angle> getPosition;



  public Climber() 
  {
    motorLeft = new TalonFX(Constants.Climber.LEFT_MOTOR, Constants.Climber.CANBUS);
    motorRight = new TalonFX(Constants.Climber.RIGHT_MOTOR, Constants.Climber.CANBUS);
    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs().withStatorCurrentLimit(60);
    currentLimitsConfigs.StatorCurrentLimitEnable = true;
   

    motorLeft.getConfigurator().apply(currentLimitsConfigs);
    motorRight.getConfigurator().apply(currentLimitsConfigs);
    encoder = new CANcoder(Constants.Climber.ENCODER);
    encoder.setPosition(0);
    controller = new PIDController(0.1, 0, 0);
    getPosition = encoder.getPosition();

    // limitSwitch = new GenericLimitSwitch(Constants.Climber.LIMIT_SWITCH);

    // limitSwitch.onTrue(() ->encoder.setPosition(0));
  }

public void setMotors(double speed)
  {
    motorLeft.set(speed);
    motorRight.set(-speed);
  }
  public double getSetpoint()
  {
    return setpoint;
  }

  public void setClimber(double setPoint)
  {
    setpoint = setPoint;
  }


  public void stopMotors() {
    motorLeft.stopMotor();
    motorRight.stopMotor();
  }

  public double getPosition() {
    return (getPosition.getValueAsDouble() / 4) * 360;
   
  }
  public void setPosition(double angle) {
   encoder.setPosition(angle);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(getPosition());
  }

  public void update()
  {
    // double speed = controller.calculate(getAngle().getDegrees(),setpoint);
    // setMotors(speed);
  }

  public ShuffleboardTab shuffleboard(String tab) {
      return shuffleboard(Shuffleboard.getTab(tab));
  }

  public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
    tab.addNumber("PID Output", () -> controller.calculate(getAngle().getDegrees(), getSetpoint()));
    tab.addNumber("Angle", () -> getAngle().getDegrees()).withPosition(4,1);
    tab.addNumber("Rotations", this::getPosition).withPosition(4,1);
    return tab;
  }

  public void refresh(){
    getPosition.refresh();
  }

  @Override
  public void periodic() {
      refresh();
      update();
  }
}
