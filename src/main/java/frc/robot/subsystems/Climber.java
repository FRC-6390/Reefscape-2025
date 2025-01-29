// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  public TalonFX leftMotor;
  public TalonFX rightMotor;
  public CANcoder encoder;
  public double setpoint;
  public GenericLimitSwitch limitSwitch;
  public double speed;
  public ShuffleboardTab tab;
  /** Creates a new Climber. */
  public Climber() 
  {
    leftMotor = new TalonFX(Constants.Climber.LEFT_MOTOR);
    rightMotor = new TalonFX(Constants.Climber.RIGHT_MOTOR);
    limitSwitch = new GenericLimitSwitch(Constants.Climber.LIMIT_SWITCH);

    encoder = new CANcoder(Constants.Climber.ENCODER);

    limitSwitch.onPress(() -> encoder.setPosition(0));

    tab = Shuffleboard.getTab("Climber");
    leftMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(40).withStatorCurrentLimitEnable(true));
    rightMotor.getConfigurator().apply(new CurrentLimitsConfigs().withStatorCurrentLimit(40).withStatorCurrentLimitEnable(true));
  }

  public void moveClimber(double speed)
  {
    leftMotor.set(speed);
    rightMotor.set(-speed);
  }
  public void setSpeed(double sspeed)
  {
    speed = sspeed;
  }

  public void setPosition(double setpoint)
  {
    this.setpoint = setpoint;
  }
  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void update()
  {
    if(limitSwitch.isPressed() && speed < 0)
    {
      stopMotors();
      setSpeed(0);
    }else{
      moveClimber(speed);
      //double spd = controller.calculate(encoder.getPosition(), setpoint);
      //moveClimber(spd);
    }  
     
  }

  public void shuffleboard()
  {
    tab.add("Climber Speed", speed);
    tab.add("Limit Switch", limitSwitch.isPressed());
  }

  public double getPosition() {
    return encoder.getPosition(true).getValueAsDouble();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(getPosition() / Constants.Climber.ENCODER_GEAR_RATIO);
  }

  @Override
  public void periodic() {
    update();
    shuffleboard();
    // This method will be called once per scheduler run
  }
}
