package frc.robot.subsystems.superstructure;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.sensors.beambreak.IRBeamBreak;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class EndEffector extends SubsystemBase {

  //EJECTOR STUFF
  public TalonFX roller;
  IRBeamBreak beamBreakLeft;
    public IRBeamBreak beamBreakRight, beamBreakCenter;
  public CANdle candle;
  public StateMachine<EjectorState> ejectStateMachine;

  //ALGAE STUFF
  public TalonFX algaeExtender;
  public boolean atSetpoint = false;
  public GenericLimitSwitch limitSwitchAlgae;
  public PIDController algaController;
  public StateMachine<AlgaeExtensionState> algStateMachine;
  public StatusSignal<Angle> getExtenderPosition = new StatusSignal<>(null, null, null);
  
  //ROTATOR STUFF
  public TalonFX motor;
  public CANcoder encoder;
  public PIDController controller;
  public StateMachine<EndEffectorState> stateMachine;
  public StatusSignal<Angle> getAbsolutePosition = new StatusSignal<>(null, null, null);

  public enum EndEffectorState implements SetpointProvider
  {
      StartConfiguration(0),
      Home(0),
      Left(-25),
      Right(25),
      LeftL4(-35),
      RightL4(35);

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
    Extended(-2.3),
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

  public enum EjectorState implements SetpointProvider
  {
    Right(-1),
    Stopped(0),
    Left(1);

    private double speed;
    private EjectorState(double angle)
    {
      this.speed = angle;
    }

    @Override
    public double getSetpoint()
    {
      return speed;
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
      controller.setTolerance(1);

      stateMachine = new StateMachine<EndEffectorState>(EndEffectorState.Home, controller::atSetpoint);

    //EJECTOR STUFF
      roller = new TalonFX(Constants.EndEffector.ROLLER, Constants.EndEffector.CANBUS);
      beamBreakLeft = new IRBeamBreak(1);
      beamBreakCenter = new IRBeamBreak(0);
      beamBreakRight = new IRBeamBreak(2);
      candle = new CANdle(Constants.EndEffector.CANDLE_ID, Constants.EndEffector.CANBUS);
      CANdleConfiguration configuration = new CANdleConfiguration();
      configuration.stripType = LEDStripType.GRB;
      configuration.statusLedOffWhenActive = false;
      configuration.disableWhenLOS = false;
      configuration.brightnessScalar = 1;
      configuration.vBatOutputMode = VBatOutputMode.Modulated;
      configuration.v5Enabled = true;
      candle.configAllSettings(configuration, 100);
      ejectStateMachine = new StateMachine<EndEffector.EjectorState>(EjectorState.Stopped, () -> true);
      

    //ALGAE EXTENSION STUFF
      limitSwitchAlgae = new GenericLimitSwitch(Constants.EndEffector.LIMIT_SWITCH);
      algaeExtender = new TalonFX(Constants.EndEffector.ALGAE_MOTOR, Constants.EndEffector.CANBUS);

      getExtenderPosition = algaeExtender.getRotorPosition();

      algaeExtender.getConfigurator().apply(new TalonFXConfiguration());
      algaeExtender.setNeutralMode(NeutralModeValue.Brake);
      CurrentLimitsConfigs limit = new CurrentLimitsConfigs();
      limit = limit.withStatorCurrentLimit(30);
      algaeExtender.getConfigurator().apply(limit);
      algaController = new PIDController(0.5, 0, 0);
      algaController.setTolerance(0.3);
      algStateMachine = new StateMachine<AlgaeExtensionState>(AlgaeExtensionState.Home, this::algaeAtSetpoint);
      algaeExtender.setPosition(0);
      limitSwitchAlgae.whileTrue(() -> algaeExtender.setPosition(AlgaeExtensionState.Extended.getSetpoint()));
  }

  public void setMotors(double speed)
  {
    motor.set(speed);
  }

  public void setExtension(double speed)
  {

    algaeExtender.set(-speed);
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
    return -getAbsolutePosition.getValueAsDouble();
   
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRotations(getPosition() / Constants.EndEffector.ENCODER_GEAR_RATIO).minus(Rotation2d.fromRotations(Constants.EndEffector.ENCODER_OFFSET));
  }

  public boolean algaeAtSetpoint()
  {
    if(algStateMachine.getGoalState().equals(AlgaeExtensionState.Home)) 
    {
    return algaController.atSetpoint();
    }
    else
    {
      return algaController.atSetpoint() && limitSwitchAlgae.getAsBoolean();
    }
  }

  public void setAlgaeEncoder(double rotations)
  {
    algaeExtender.setPosition(0);
  }

  public void update()
  {
    if(limitSwitchAlgae.getAsBoolean())
    {
      algaeExtender.setPosition(EndEffector.AlgaeExtensionState.Extended.getSetpoint());
    }

    //ROTATOR STUFF
    switch (stateMachine.getGoalState()) {
      case Left, Right, RightL4, LeftL4, Home, StartConfiguration:
      double speed = controller.calculate(getAngle().getDegrees(), stateMachine.getGoalState().getSetpoint());
      setMotors(speed);
    }

    //EXTENSION STUFF
    switch (algStateMachine.getGoalState()) 
    {
      case Home:
        setExtension(-algaController.calculate(getExtenderPosition.getValueAsDouble(),AlgaeExtensionState.Home.getSetpoint()));
        break; 
      case Extended:
        double spd = Math.abs(algaController.calculate(getExtenderPosition.getValueAsDouble(),AlgaeExtensionState.Extended.getSetpoint()));
        SmartDashboard.putNumber("EEEEH", spd);
        setExtension(spd + Math.copySign(0.1, spd));
        
      default:
        break;
    }
    
    // //ROLLER STUFF
    //Eject logic
    switch (ejectStateMachine.getGoalState()) {
      case Left:
      case Stopped:
      case Right:
        roller.set(ejectStateMachine.getGoalState().getSetpoint());
        break;
      default:
        break;
    }

    if(!beamBreakCenter.getAsBoolean() && !beamBreakLeft.getAsBoolean() && !beamBreakRight.getAsBoolean() && !algStateMachine.atState(AlgaeExtensionState.Extended))
    {
        ejectStateMachine.setGoalState(EjectorState.Stopped);
    }


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
    tab.addNumber("Next State Angle", () -> stateMachine.getNextState().getSetpoint()).withPosition(5, 1);
    tab.addNumber("Algae", () -> getExtenderPosition.getValueAsDouble()).withPosition(5, 1);
    tab.addNumber("AlgaeOutput", () -> algaController.calculate(AlgaeExtensionState.Extended.getSetpoint())).withPosition(5, 1);
    tab.addString("Algae State", () -> algStateMachine.getNextState().name()).withPosition(5, 1);


    tab.addBoolean("BeamBreakLeft", () -> beamBreakLeft.getAsBoolean());
    tab.addBoolean("BeamBreakRight", () -> beamBreakRight.getAsBoolean());
    tab.addBoolean("BeamBreakCenter", () -> beamBreakCenter.getAsBoolean());
    
    return tab;
  }

  public void refresh(){
    getAbsolutePosition.refresh();
    getExtenderPosition.refresh();
    algStateMachine.update();
    stateMachine.update();
    ejectStateMachine.update();
  }

  @Override
  public void periodic() {
      refresh();
      update();
  }
}
