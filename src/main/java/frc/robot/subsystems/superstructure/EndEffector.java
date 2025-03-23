package frc.robot.subsystems.superstructure;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.sensors.beambreak.IRBeamBreak;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.endeffector.AlgaeExtender;
import frc.robot.subsystems.superstructure.endeffector.AlgaeExtender.AlgaeExtenderState;
import frc.robot.subsystems.superstructure.endeffector.Rollers;
import frc.robot.subsystems.superstructure.endeffector.Rotator;
import frc.robot.subsystems.superstructure.endeffector.Rotator.RotatorState;
import frc.robot.subsystems.superstructure.endeffector.Rollers.RollerState;

public class EndEffector extends SubsystemBase{

    private final Rollers rollers;
    private boolean rollersEnabled = true;
    private final Rotator rotator;
    private boolean rotatorEnabled = true;
    private final AlgaeExtender algaeExtender;
    private boolean algaeEnabled = true;

    private boolean autoEndScoring = true;

    // public Elevator elevator;

    private final StateMachine<Double, RollerState> rollerStateMachine;
    private final StateMachine<Double, RotatorState> rotatorStateMachine;
    private final StateMachine<Double, AlgaeExtenderState> algaeExtenderStateMachine;

    private final StateMachine<EndEffectorTuple, EndEffectorState> stateMachine;

    private final IRBeamBreak beamBreakLeft, beamBreakRight, beamBreakCenter;

    public record EndEffectorTuple(RollerState rollerState,  RotatorState rotatorState, AlgaeExtenderState algaeExtenderState) {}
    
    public enum EndEffectorState implements SetpointProvider<EndEffectorTuple>
    {
        AlgaeHigh(new EndEffectorTuple(RollerState.Algae, RotatorState.Algae, AlgaeExtenderState.Extended)),
        AlgaeRetract(new EndEffectorTuple(RollerState.Stopped, RotatorState.Home, AlgaeExtenderState.Home)),
        AlgaeLow(new EndEffectorTuple(RollerState.Algae, RotatorState.Algae, AlgaeExtenderState.Extended)),
        L4(new EndEffectorTuple(null, RotatorState.L4, AlgaeExtenderState.Home)),
        L3(new EndEffectorTuple(null, RotatorState.Home, AlgaeExtenderState.Home)),
        L2(new EndEffectorTuple(null, RotatorState.Home, AlgaeExtenderState.Home)),
        L1(new EndEffectorTuple(null, RotatorState.Home, AlgaeExtenderState.Home)),
        Score(new EndEffectorTuple(RollerState.Running, null, null)),
        Stop(new EndEffectorTuple(RollerState.Stopped, null, null)),
        Home(new EndEffectorTuple(RollerState.Stopped, RotatorState.Home, AlgaeExtenderState.Home));


        private EndEffectorTuple states;
        private EndEffectorState(EndEffectorTuple states)
        {
            this.states = states;
        }

        @Override
        public EndEffectorTuple getSetpoint()
        {
            return states;
        }
    }

    public EndEffector(RobotBase<?> base){
        this(new Rollers(base), new Rotator(base), new AlgaeExtender());
    }

    public EndEffector(Rollers rollers, Rotator rotator, AlgaeExtender algae){
        this.rollers = rollers;
        this.rotator = rotator;
        this.algaeExtender = algae;

        this.rollerStateMachine = rollers.getStateMachine();
        this.rotatorStateMachine = rotator.getStateMachine();
        this.algaeExtenderStateMachine = algae.getStateMachine();

        beamBreakLeft = new IRBeamBreak(1);
        beamBreakCenter = new IRBeamBreak(0);
        beamBreakRight = new IRBeamBreak(2);

        this.stateMachine = new StateMachine<EndEffectorTuple,EndEffectorState>(EndEffectorState.Home, () -> rollerStateMachine.atGoalState() && rotatorStateMachine.atGoalState() && algaeExtenderStateMachine.atGoalState());
    }

    public StateMachine<EndEffectorTuple, EndEffectorState> getStateMachine() {
        return stateMachine;
    }

    public boolean hasGamePiece(){
        return beamBreakLeft.getAsBoolean() || beamBreakCenter.getAsBoolean() || beamBreakRight.getAsBoolean();
    }

    public boolean hasNoPiece(){
        return !beamBreakLeft.getAsBoolean() && !beamBreakCenter.getAsBoolean() && !beamBreakRight.getAsBoolean();
    }

    public ShuffleboardTab shuffleboard(String tab) {
        return shuffleboard(Shuffleboard.getTab(tab));
    }

    public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
       
        tab.addBoolean("BeamBreakLeft", () -> beamBreakLeft.getAsBoolean());
        tab.addBoolean("BeamBreakRight", () -> beamBreakRight.getAsBoolean());
        tab.addBoolean("BeamBreakCenter", () -> beamBreakCenter.getAsBoolean());

        
        tab.addBoolean("rollersEnabled", () -> rollersEnabled);
        tab.addBoolean("algaeEnabled", () -> algaeEnabled);
        tab.addBoolean("rotatorEnabled", () -> rotatorEnabled);

        algaeExtender.shuffleboard(tab, "Algae Extender");
        rollers.shuffleboard(tab, "Rollers");
        rotator.shuffleboard(tab, "Rotator");

        return tab;
    } 

    public boolean isScoring(){
        return rollerStateMachine.atAnyState(RollerState.Running, RollerState.RunningInverted);
    }

    public EndEffector setAutoEndScoring(boolean autoEndScoring) {
        this.autoEndScoring = autoEndScoring;
        return this;
    }

    public EndEffector setAlgaeEnabled(boolean algaeEnabled) {
        this.algaeEnabled = algaeEnabled;
        return this;
    }

    public EndEffector setRollersEnabled(boolean rollersEnabled) {
        this.rollersEnabled = rollersEnabled;
        return this;
    }

    public EndEffector setRotatorEnabled(boolean rotatorEnabled) {
        this.rotatorEnabled = rotatorEnabled;
        return this;
    }

    public void update(){

        algaeExtenderStateMachine.update();
        rollerStateMachine.update();
        rotatorStateMachine.update();

        stateMachine.update();

        EndEffectorState state = stateMachine.getGoalState();
        EndEffectorTuple val = stateMachine.getGoalStateSetpoint();
        switch (state) {
            case L4: 
            case L3: 
            case L2: 
            case L1: 
            case AlgaeHigh: 
            case AlgaeRetract:
            case AlgaeLow: 
            case Home:
            case Score:
            case Stop:
                if (val.rollerState != null && rollersEnabled) rollerStateMachine.queueState(val.rollerState);
                if (val.rotatorState != null && rotatorEnabled) rotatorStateMachine.queueState(val.rotatorState);
                if (val.algaeExtenderState != null && algaeEnabled) algaeExtenderStateMachine.queueState(val.algaeExtenderState());
            default:
                break;
        }
    }

    public AlgaeExtender getAlgaeExtender() {
        return algaeExtender;
    }

    public Rollers getRollers() {
        return rollers;
    }

    public Rotator getRotator() {
        return rotator;
    }

    public boolean isAutoEndScoring() {
        return autoEndScoring;
    }


    @Override
    public void periodic() {
        update();
    }
}
