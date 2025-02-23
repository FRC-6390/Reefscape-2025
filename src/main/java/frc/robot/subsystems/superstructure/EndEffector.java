package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

  //EJECTOR STUFF
  public TalonFX roller;
  public DigitalInput beamBreakLeft, beamBreakRight, beamBreakCenter;
  public double rollerSpeed = 0;

  //ALGAE STUFF
  public TalonFX algaeExtender;
  public boolean atSetpoint = false;
  public GenericLimitSwitch limitSwitchAlgae;
  public StateMachine<AlgaeExtensionState> algStateMachine;
  public StatusSignal<Angle> getPosition = new StatusSignal<>(null, null, null);
  
  //ROTATOR STUFF
  public TalonFX motor;
  public CANcoder encoder;
  public PIDController controller;
  public StateMachine<EndEffectorState> stateMachine;
  public StatusSignal<Angle> getAbsolutePosition = new StatusSignal<>(null, null, null);

  public enum EndEffectorState implements SetpointProvider
  {
      StartConfiguration(35),
      Home(0),
      Left(35),
      Right(-35),
      LeftL4(35),
      RightL4(-35);

      private double angle;
      private EndEffectorState(double angle)
      {
        this.angle = angle;
      }

      @Override
      public double getSetpoint() {
        return angle;
      }
  }

  public enum AlgaeExtensionState implements SetpointProvider
  {
    Extended(20),
    Home(0);

    private double angle;
    private AlgaeExtensionState(double angle)
    {
      this.angle = angle;
    }

    @Override
    public double getSetpoint()
    {
      return angle;
    }
  }

  /** Creates a new EndEffector. */
  public EndEffector() 
  {
    //ROTATOR STUFF
      motor = new TalonFX(Constants.EndEffector.MOTOR, Constants.EndEffector.CANBUS);
      encoder = new CANcoder(Constants.EndEffector.ENCODER, Constants.EndEffector.CANBUS);
      getAbsolutePosition = encoder.getAbsolutePosition();

      motor.getConfigurator().apply(new TalonFXConfiguration());
      motor.setNeutralMode(NeutralModeValue.Brake);

      CANcoderConfiguration config = new CANcoderConfiguration();
      config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;

      encoder.getConfigurator().apply(config);

      controller = Constants.EndEffector.CONTORLLER;
      controller.enableContinuousInput(0, 90);
      controller.setTolerance(1);

      stateMachine = new StateMachine<EndEffectorState>(EndEffectorState.Home, controller::atSetpoint);

    //EJECTOR STUFF
      roller = new TalonFX(Constants.EndEffector.ROLLER, Constants.EndEffector.CANBUS);
      beamBreakLeft = new DigitalInput(1);
      beamBreakCenter = new DigitalInput(0);
      beamBreakRight = new DigitalInput(2);

    //ALGAE EXTENSION STUFF
      limitSwitchAlgae = new GenericLimitSwitch(Constants.EndEffector.LIMIT_SWITCH);
      algaeExtender = new TalonFX(Constants.EndEffector.ALGAE_MOTOR);

      getPosition = algaeExtender.getRotorPosition();

      algaeExtender.getConfigurator().apply(new TalonFXConfiguration());
      algaeExtender.setNeutralMode(NeutralModeValue.Brake);

      algStateMachine = new StateMachine<AlgaeExtensionState>(AlgaeExtensionState.Home, this::algaeAtSetpoint);
      algaeExtender.setPosition(0);
  }


  public void setMotors(double speed)
  {
    motor.set(speed);
  }

  public void setExtension(double speed)
  {
    algaeExtender.set(speed);
  }

  public void setRollers(double speed)
  {
    rollerSpeed = speed;
  }

  public StateMachine<EndEffectorState> getStateMachine()
  {
    return stateMachine;
  }

  public StateMachine<AlgaeExtensionState> getAlgaeStateMachine()
  {
    return algStateMachine;
  }
  public void stopMotors() {
    motor.stopMotor();
  }

  public double getPosition() {
    return getAbsolutePosition.getValueAsDouble();
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(getPosition() / Constants.EndEffector.ENCODER_GEAR_RATIO).minus(Rotation2d.fromDegrees(Constants.EndEffector.ENCODER_OFFSET));
  }

  public boolean algaeAtSetpoint()
  {
    return atSetpoint;
  }

  public void setAlgaeEncoder(double rotations)
  {
    algaeExtender.setPosition(0);
  }


  public void update()
  {
    //ROTATOR STUFF
    switch (stateMachine.getGoalState()) {
      case Left, Right, RightL4, LeftL4, Home, StartConfiguration:
      double speed = -controller.calculate(getAngle().getDegrees(), stateMachine.getGoalState().getSetpoint());
      setMotors(speed);
    }

    //EXTENSION STUFF
    switch (algStateMachine.getGoalState()) 
    {
      case Home:
        if(Math.abs(getPosition.getValueAsDouble()) > 0.3)
        {
        setMotors(-0.3);  
        }
        else{
          atSetpoint = true;
        }   
        break; 
      case Extended:
        if(limitSwitchAlgae.getAsBoolean() == false)
        {
        setMotors(0.3);
        }
        else {
          atSetpoint = true;
        }
      default:
        break;
    }
    
    //ROLLER STUFF
    roller.set(rollerSpeed);
  }


  public ShuffleboardTab shuffleboard(String tab) {
      return shuffleboard(Shuffleboard.getTab(tab));
  }

  public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
    tab.addBoolean("Limit Switch", limitSwitchAlgae::getAsBoolean).withPosition(1,1);
    tab.addString("State", () -> stateMachine.getGoalState().name()).withPosition(2,1);
    tab.addNumber("PID Output", () -> controller.calculate(getAngle().getDegrees(), stateMachine.getGoalState().getSetpoint())).withPosition(3,1);
    tab.addNumber("Angle", () -> getAngle().getDegrees()).withPosition(4,1);
    tab.addNumber("Rotations", this::getPosition).withPosition(4,1);
    tab.addString("Next State", () -> stateMachine.getNextState().name()).withPosition(5, 1);

    return tab;
  }

  public void refresh(){
    getAbsolutePosition.refresh();
    stateMachine.update();
  }

  @Override
  public void periodic() {
      refresh();
      update();
  }
}
