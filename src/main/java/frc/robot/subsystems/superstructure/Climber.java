package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
public class Climber {
  public TalonFX leftMotor;
  public TalonFX rightMotor;
  public CANcoder encoder;
  
  public DigitalInput limitSwitch;
  public PIDController controller;
  public STATE state;
  public enum STATE
  {
    HOME(0),
    CLIMB(45);
    private double angle;
    private STATE(double angle)
    {
      this.angle = angle;
    }

    public double get()
    {
      return angle;
    }
  }

  /** Creates a new Climber. */
  public Climber() 
  {
    leftMotor = new TalonFX(Constants.Climber.LEFT_MOTOR, "can");
    rightMotor = new TalonFX(Constants.Climber.RIGHT_MOTOR, "can");
    limitSwitch = new DigitalInput(Constants.Climber.LIMIT_SWITCH);
    controller = new PIDController(0.015, 0, 0);
    controller.enableContinuousInput(0, 90);
    state = STATE.HOME;
    leftMotor.getConfigurator().apply(new TalonFXConfiguration());
    rightMotor.getConfigurator().apply(new TalonFXConfiguration());
     encoder = new CANcoder(Constants.Climber.ENCODER, "can");

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    encoder.getConfigurator().apply(config);
  }


  public void moveClimber(double speed)
  {
    leftMotor.set(speed);
    rightMotor.set(-speed);
  }

  public void setPosition(STATE state)
  {
    this.state = state;
  }

  public void stopMotors() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  public void update()
  {
    shuffleboard();
    System.out.println(getAngle().getDegrees());
    double speed = -controller.calculate(getAngle().getDegrees(), state.get());
    if(!limitSwitch.get() && speed > 0)
    {
      return;
    }
    else{
    moveClimber(speed); 
    }
  }

  public void shuffleboard()
  {
    SmartDashboard.putBoolean("Limit Switch", !limitSwitch.get());
    SmartDashboard.putNumber("Setpoint", state.get());
    SmartDashboard.putNumber("PID Output", controller.calculate(getAngle().getDegrees(), state.get()));
    SmartDashboard.putNumber("Angle", getAngle().getDegrees());
    SmartDashboard.putNumber("Rotations", getPosition());
  }

  public double getPosition() {
    return encoder.getAbsolutePosition(true).getValueAsDouble();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(getPosition() / Constants.Climber.ENCODER_GEAR_RATIO).minus(Rotation2d.fromDegrees(Constants.Climber.ENCODER_OFFSET));
  }
}
