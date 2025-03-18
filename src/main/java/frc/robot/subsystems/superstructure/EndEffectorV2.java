package frc.robot.subsystems.superstructure;

import ca.frc6390.athena.core.RobotBase;
import ca.frc6390.athena.mechanisms.Mechanism.StatefulMechanism;
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
import frc.robot.Constants.EndEffector.PivotState;
import frc.robot.Constants.EndEffector.RollerState;


public class EndEffectorV2 extends SubsystemBase{

    private final StatefulMechanism<PivotState> pivot;
    private final StatefulMechanism<RollerState> rollers;

    private boolean autoEndScoring = true;

    // public Elevator elevator;

    // private final StateMachine<Double, RollerState> rollerStateMachine;
    // private final StateMachine<Double, RotatorState> rotatorStateMachine;
    // private final StateMachine<Double, AlgaeExtenderState> algaeExtenderStateMachine;

    private final StateMachine<EndEffectorTuple, EndEffectorState> stateMachine;

    private final IRBeamBreak beamBreakLeft, beamBreakRight, beamBreakCenter;

    public record EndEffectorTuple(RollerState rollerState,  PivotState rotatorState ) {}
    
    public enum EndEffectorState implements SetpointProvider<EndEffectorTuple>
    {
        // AlgaeHigh(new EndEffectorTuple(RollerState.Algae, RotatorState.Algae, AlgaeExtenderState.Extended)),
        // AlgaeRetract(new EndEffectorTuple(RollerState.Stopped, RotatorState.Home, AlgaeExtenderState.Home)),
        // AlgaeLow(new EndEffectorTuple(RollerState.Algae, RotatorState.Algae, AlgaeExtenderState.Extended)),
        L4(new EndEffectorTuple(RollerState.Stopped, PivotState.Scoring)),
        L3(new EndEffectorTuple(RollerState.Stopped,  PivotState.Scoring)),
        L2(new EndEffectorTuple(RollerState.Stopped, PivotState.Scoring)),
        L1(new EndEffectorTuple(RollerState.Stopped, PivotState.Scoring)),
        Score(new EndEffectorTuple(RollerState.Scoring, PivotState.Scoring)),
        Stop(new EndEffectorTuple(RollerState.Stopped, null)),
        Home(new EndEffectorTuple(RollerState.Stopped, PivotState.Home)),
        Intaking(new EndEffectorTuple(RollerState.Intaking, PivotState.Intaking));


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

   

    public EndEffectorV2( StatefulMechanism<PivotState> Pivot, StatefulMechanism<RollerState>  Rollers ){
    
        this.pivot = Pivot;
        this.rollers = Rollers;

        beamBreakLeft = new IRBeamBreak(1);
        beamBreakCenter = new IRBeamBreak(0);
        beamBreakRight = new IRBeamBreak(2);

        this.stateMachine = new StateMachine<EndEffectorTuple,EndEffectorState>(EndEffectorState.Home, () -> rollers.getStateMachine().atGoalState() && pivot.getStateMachine().atGoalState());
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

        
        // tab.addBoolean("rollersEnabled", () -> rollersEnabled);
        // tab.addBoolean("algaeEnabled", () -> algaeEnabled);
        // tab.addBoolean("rotatorEnabled", () -> rotatorEnabled);

        // algaeExtender.shuffleboard(tab, "Algae Extender");
        // rollers.shuffleboard(tab, "Rollers");
        // rotator.shuffleboard(tab, "Rotator");

        return tab;
    } 

    public boolean isScoring(){
        return rollers.getStateMachine().atAnyState(RollerState.Scoring);
    }

    public EndEffectorV2 setAutoEndScoring(boolean autoEndScoring) {
        this.autoEndScoring = autoEndScoring;
        return this;
    }

    // public EndEffectorV2 setAlgaeEnabled(boolean algaeEnabled) {
    //     this.algaeEnabled = algaeEnabled;
    //     return this;
    // }

    // public EndEffectorV2 setRollersEnabled(boolean rollersEnabled) {
    //     this.rollersEnabled = rollersEnabled;
    //     return this;
    // }

    // public EndEffectorV2 setRotatorEnabled(boolean rotatorEnabled) {
    //     this.rotatorEnabled = rotatorEnabled;
    //     return this;
    // }

    public void update(){

        // algaeExtenderStateMachine.update();
        // rollerStateMachine.update();
        // rotatorStateMachine.update();

        stateMachine.update();
        rollers.update();
        pivot.update();

        EndEffectorState state = stateMachine.getGoalState();
        EndEffectorTuple val = stateMachine.getGoalStateSetpoint();
        switch (state) {
            case L4: 
            case L3: 
            case L2: 
            case L1:  
            case Home:
            case Score:
            case Intaking:
            case Stop:
                if (val.rollerState != null) rollers.getStateMachine().setGoalState(val.rollerState);
                if (val.rotatorState != null) pivot.getStateMachine().setGoalState(val.rotatorState);
            default:
                break;
        }
    }


    public StatefulMechanism<RollerState> getRollers() {
        return rollers;
    }

    public StatefulMechanism<PivotState> getPivot() {
        return pivot;
    }

    public boolean isAutoEndScoring() {
        return autoEndScoring;
    }


    @Override
    public void periodic() {
        update();
    }
}
