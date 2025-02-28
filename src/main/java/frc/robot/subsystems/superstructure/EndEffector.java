package frc.robot.subsystems.superstructure;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.sensors.beambreak.IRBeamBreak;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.superstructure.endeffector.AlgaeExtender;
import frc.robot.subsystems.superstructure.endeffector.AlgaeExtender.AlgaeExtenderState;
import frc.robot.subsystems.superstructure.endeffector.Rollers;
import frc.robot.subsystems.superstructure.endeffector.Rotator;
import frc.robot.subsystems.superstructure.endeffector.Rotator.RotatorState;
import frc.robot.subsystems.superstructure.endeffector.Rollers.RollerState;

public class EndEffector extends SubsystemBase{

    private final Rollers rollers;
    private final boolean rollersEnabled = true;
    private final Rotator rotator;
    private final boolean rotatorEnabled = true;
    private final AlgaeExtender algae;
    private final boolean algaeEnabled = true;

    private final StateMachine<Double, RollerState> rollerStateMachine;
    private final StateMachine<Double, RotatorState> rotatorStateMachine;
    private final StateMachine<Double, AlgaeExtenderState> algaeExtenderStateMachine;

    private final StateMachine<EndEffectorTuple, EndEffectorState> stateMachine;

    private final IRBeamBreak beamBreakLeft, beamBreakRight, beamBreakCenter;

    public record EndEffectorTuple(RollerState rollerState,  RotatorState rotatorState, AlgaeExtenderState algaeExtenderState) {}
    
    public enum EndEffectorState implements SetpointProvider<EndEffectorTuple>
    {
        AlgaeHigh(new EndEffectorTuple(RollerState.Algae, RotatorState.Home, AlgaeExtenderState.Extended)),
        AlgaeLow(new EndEffectorTuple(RollerState.Algae, RotatorState.Home, AlgaeExtenderState.Extended)),
        L4(new EndEffectorTuple(RollerState.Running, RotatorState.LeftL4, AlgaeExtenderState.Home)),
        L3(new EndEffectorTuple(RollerState.Running, RotatorState.Home, AlgaeExtenderState.Home)),
        L2(new EndEffectorTuple(RollerState.Running, RotatorState.Home, AlgaeExtenderState.Home)),
        L1(new EndEffectorTuple(RollerState.Running, RotatorState.Home, AlgaeExtenderState.Home)),
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
        this(new Rollers(base), new Rotator(), new AlgaeExtender());
    }

    public EndEffector(Rollers rollers, Rotator rotator, AlgaeExtender algae){
        this.rollers = rollers;
        this.rotator = rotator;
        this.algae = algae;

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

        algae.shuffleboard(tab, "Algae Extender");
        rollers.shuffleboard(tab, "Rollers");
        rotator.shuffleboard(tab, "Rotator");

        return tab;
    } 

    public void update(){

        if (algaeEnabled) algaeExtenderStateMachine.update();
        if (rollersEnabled) rollerStateMachine.update();
        if (rotatorEnabled) rotatorStateMachine.update();

        stateMachine.update();

        EndEffectorState state = stateMachine.getGoalState();
        switch (state) {
            case L4: 
            case L3: 
            case L2: 
            case L1: 
                rollerStateMachine.setGoalState(state.getSetpoint().rollerState);
                rotatorStateMachine.setGoalState(state.getSetpoint().rotatorState, () -> rollerStateMachine.atGoalState());
                algaeExtenderStateMachine.setGoalState(state.getSetpoint().algaeExtenderState);
            break;
            case AlgaeHigh: 
            case AlgaeLow: 
                rollerStateMachine.setGoalState(state.getSetpoint().rollerState);
                rotatorStateMachine.setGoalState(state.getSetpoint().rotatorState, () -> rollerStateMachine.atGoalState());
                algaeExtenderStateMachine.setGoalState(state.getSetpoint().algaeExtenderState, () -> rollerStateMachine.atGoalState());
            break;
            case Home:
                rollerStateMachine.setGoalState(state.getSetpoint().rollerState);
                rotatorStateMachine.setGoalState(state.getSetpoint().rotatorState, () -> rollerStateMachine.atGoalState());
                algaeExtenderStateMachine.setGoalState(state.getSetpoint().algaeExtenderState);
            default:
                break;
        }
    }

    @Override
    public void periodic() {
        update();
    }
}
