package frc.robot;

import java.util.List;
import java.util.function.DoubleSupplier;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import ca.frc6390.athena.controllers.ElevatorFeedForwardsSendable;
import ca.frc6390.athena.core.RobotCore.RobotCoreConfig;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.DrivetrainIDs;
import ca.frc6390.athena.core.localization.PoseConfig;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrainConfig;
import ca.frc6390.athena.drivetrains.swerve.modules.SwerveVendorSDS;
import ca.frc6390.athena.drivetrains.swerve.sim.SwerveSimulationConfig;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.imu.AthenaImu;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.FlywheelMechanism.StatefulFlywheelMechanism;
import ca.frc6390.athena.mechanisms.MechanismConfig;
import ca.frc6390.athena.mechanisms.ElevatorMechanism.StatefulElevatorMechanism;
import ca.frc6390.athena.mechanisms.MechanismConfig.ArmSimulationParameters;
import ca.frc6390.athena.mechanisms.MechanismConfig.ElevatorSimulationParameters;
import ca.frc6390.athena.mechanisms.MechanismConfig.SimpleMotorSimulationParameters;
import ca.frc6390.athena.mechanisms.StatefulMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.TurretMechanism.StatefulTurretMechanism;
import ca.frc6390.athena.mechanisms.SuperstructureConfig;
import ca.frc6390.athena.mechanisms.sim.MechanismVisualizationConfig;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch.GenericLimitSwitchConfig;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.limelight.LimeLightConfig;
import ca.frc6390.athena.sensors.camera.photonvision.PhotonVisionConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants.EndEffector.ArmState;
import frc.robot.Constants.EndEffector.WristState;
import frc.robot.utils.Align.AutomationCamera;;

public interface Constants {
    
    String CANIVORE_CANBUS = "can";
    public interface DriveTrain {

        double TRACKWIDTH_METERS = Units.inchesToMeters(18.375); 
    
        //SIREN
        double[] ENCODER_OFFSETS = {0.23535156250000003,0.09350585937499999,0.19873046875000003,0.361572265625};
        
        SwerveDrivetrainConfig DRIVETRAIN_CONFIG = SwerveDrivetrainConfig.defualt(TRACKWIDTH_METERS)
                                                    .setIMU(AthenaImu.PIGEON2, false)
                                                    .setIds(DrivetrainIDs.SWERVE_CHASSIS_STANDARD)
                                                    .modules(
                                                            SwerveVendorSDS.MK4n.L1_PLUS.config(AthenaMotor.KRAKEN_X60,AthenaEncoder.CANCODER).setP(0.45), 
                                                            SwerveVendorSDS.MK4n.L1_PLUS.config(AthenaMotor.KRAKEN_X60,AthenaEncoder.CANCODER).setP(0.45), 
                                                            SwerveVendorSDS.MK4i.L1_PLUS.config(AthenaMotor.KRAKEN_X60,AthenaEncoder.CANCODER).setP(0.5), 
                                                            SwerveVendorSDS.MK4i.L1_PLUS.config(AthenaMotor.KRAKEN_X60,AthenaEncoder.CANCODER).setP(0.5)
                                                            )   
                                                    .setEncoderOffset(ENCODER_OFFSETS)
                                                    .setCanbus(CANIVORE_CANBUS)
                                                    .setDriveCurrentLimit(60)
                                                    .setSteerCurrentLimit(40)
                                                    .setSimulationConfig(SwerveSimulationConfig.defaults()
                                                        .withRobotMassKg(Units.lbsToKilograms(115))
                                                        .withNominalVoltage(12.0)
                                                        .withWheelCoefficientOfFriction(1.1)
                                                    )
                                                    .setFieldRelative(true);

        //UP 40
        //LEFT SIDE 8.5
        //X 4.5                                                 
        RobotLocalizationConfig LOCALIZATION_CONFIG = RobotLocalizationConfig.vision(0.8, 0.8, 9999)
                                                            .setAutoPlannerPID(7,0,0, 2,0,0)
                                                            .setVisionEnabled(true)
                                                            .addPoseConfig(PoseConfig.defaults("field"))
                                                            .setAutoPoseName("field");
        ConfigurableCamera[] CAMERAS =
        {                                                                 
        LimeLightConfig.table("limelight-left").setTrustDistance(100).setUseForLocalization(true).setCameraRobotSpace(new Transform3d(Units.inchesToMeters(-0.5),Units.inchesToMeters(9.25),Units.inchesToMeters(8),new Rotation3d(0, Units.degreesToRadians(-15), Units.degreesToRadians(-15)))).setPoseEstimateType(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).setLocalizationTagFilter(17,18,19,20,21,22,6,7,8,9,10,11), 
        LimeLightConfig.table("limelight-right").setTrustDistance(100).setUseForLocalization(true).setCameraRobotSpace(new Transform3d(Units.inchesToMeters(-0.5),Units.inchesToMeters(-9.25),Units.inchesToMeters(8),new Rotation3d(0,  Units.degreesToRadians(-15), Units.degreesToRadians(15)))).setPoseEstimateType(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).setLocalizationTagFilter(17,18,19,20,21,22,6,7,8,9,10,11),
        PhotonVisionConfig.table("Tag").setTrustDistance(1).setUseForLocalization(false).setCameraRobotSpace(new Transform3d(-Units.inchesToMeters(10.5),-Units.inchesToMeters(9.5),Units.inchesToMeters(36),new Rotation3d(0, 0, 180))).setPoseStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR),
        PhotonVisionConfig.table("TagFront").setTrustDistance(1).setUseForLocalization(false).setCameraRobotSpace(new Transform3d(-Units.inchesToMeters(7),Units.inchesToMeters(8.5),Units.inchesToMeters(40),new Rotation3d(0, 0, 0))).setPoseStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)
        };

        AutomationCamera camLeft = new AutomationCamera(new LimeLight(new LimeLightConfig("limelight-left")), Units.inchesToMeters(0.5), Units.inchesToMeters(-9), 15, 0, 0);
        AutomationCamera camRight = new AutomationCamera(new LimeLight(new LimeLightConfig("limelight-right")), Units.inchesToMeters(0.5), Units.inchesToMeters(9), -15, 0, 0);


        //X -11.75
        //Y 9 inches
        //Z 33.5
        RobotCoreConfig<SwerveDrivetrain> ROBOT_BASE = RobotCoreConfig.swerve(DRIVETRAIN_CONFIG)
                                                                      .setLocalization(LOCALIZATION_CONFIG)
                                                                      .setVision(CAMERAS);

        
    }

    public interface Controllers {
        double STICK_DEADZONE = 0.15;      
    }

    public interface Elevator {


        public enum ElevatorState implements SetpointProvider<Double> {
            //ELEVATOR HEIGHT FROM FLOOR IN INCHES
            HomeReset(Constants.Elevator.OFFSET_FROM_FLOOR),
            HomePID(Constants.Elevator.OFFSET_FROM_FLOOR),
            L1(Constants.Elevator.OFFSET_FROM_FLOOR + 3),
            Intaking(Constants.Elevator.OFFSET_FROM_FLOOR),

            Aligning(33),

            AlgaeHigh(46.77),
            AlgaeLow(32.545),
            //31.5
            L2(35.5),
            //47.25
            L3(49.5),
            //72
            L4(76.23066732041963);


            double pos;
            private ElevatorState(double pos){
                this.pos = pos;
            }

            @Override
            public Double getSetpoint() {
            return pos;
            }
        }
       
        int ENCODER = 42;

        int LEFT_MOTOR = 20;
        int RIGHT_MOTOR = 21;
        double GEAR_DIAMETER_INCHES = 3d;
        double OFFSET_FROM_FLOOR = 24;
        double ENCODER_GEAR_RATIO = 1d/1d;
        double MOTOR_GEAR_RATIO = 7d/1d;
        int LIMIT_SWITCH = 5;
    
        ProfiledPIDController CONTORLLER = new ProfiledPIDController(0.11, 0.0, 0, new Constraints(80, 70));
        ElevatorFeedForwardsSendable FEEDFORWARD = new ElevatorFeedForwardsSendable(0, 0.117, 0.1,0);

        MechanismConfig<StatefulElevatorMechanism<ElevatorState>> ELEVATOR_CONFIG = MechanismConfig.statefulElevator(new ElevatorFeedforward(0,0.117,0.1, 0), ElevatorState.HomeReset)
        .addMotors(AthenaMotor.KRAKEN_X60, 20, -21)
        .setEncoder(AthenaEncoder.CANCODER, 42)
        .setNeutralMode(MotorNeutralMode.Brake)
        .setEncoderConversion(3d)
        .setEncoderConversionOffset(OFFSET_FROM_FLOOR)
        .setTolerance(1)
        .setCanbus(CANIVORE_CANBUS)
        .setProfiledPID(0.11, 0, 0, 80 ,70)
        .setCurrentLimit(60)
        .addLowerLimitSwitch(-5, OFFSET_FROM_FLOOR, true)
        .setStateActionSupressMotors(mech -> mech.setSpeed(-0.25), ElevatorState.HomeReset, ElevatorState.Intaking)
        .setStateMachineDelay(Units.millisecondsToSeconds(40))
        .setSimulationElevator(
            new ElevatorSimulationParameters()
            .setCarriageMassKg(Units.lbsToKilograms(30))
            .setDrumRadiusMeters(Units.inchesToMeters(GEAR_DIAMETER_INCHES) / 2.0)
            .setNominalVoltage(12)
            .setSimulateGravity(false)
            .setStartingHeightMeters(Units.inchesToMeters(OFFSET_FROM_FLOOR))
            .setRangeMeters(Units.inchesToMeters(OFFSET_FROM_FLOOR), Units.inchesToMeters(76))
            .setUnitsPerMeter(Units.metersToInches(1)));
    }

    public interface EndEffector {

        int CANDLE_ID = 22;
        // Tune these for your robot geometry.
        double FIXED_ARM_LENGTH_METERS = Units.inchesToMeters(10.0);
        double ARM_LENGTH_METERS = Units.inchesToMeters(24.0);
        double WRIST_LENGTH_METERS = Units.inchesToMeters(10.0);
        double ARM_MOTOR_REDUCTION = 100.0;
        double WRIST_MOTOR_REDUCTION = 100.0;


        enum ArmState implements SetpointProvider<Double>{
            Intaking(() ->0), //150.38085937
            AlgaeHigh(() ->-112.14), //52.8
            AlgaeLow(() ->-113.5), //52.8
            Home(() ->-92), //61.083
            StartConfiguration(() ->-153), //0
            Scoring(() ->-72), //78.310546875
            TransitionState(() ->-85), //65
            ScoringL4(() ->-90), //60
            AlgaeScore(() ->-30), //120.58
            Scoringl1(() -> Robot.armSupplier); //66.08

            DoubleSupplier angle;
            ArmState(DoubleSupplier angle){
                this.angle = angle;
            }

            @Override
            public Double getSetpoint() {
               return angle.getAsDouble();
            }

        }

            enum WristState implements SetpointProvider<Double>{
                Intaking(40),
                Home(120d), //110
                Scoring(120d), //125.419921875
                AlgaeHigh(-36.123),
                AlgaeLow(-35.595),
                ScoringL4(60), //80
                TransitionState(62),
                AlgaeScore(168.92),

                StartConfiguration(0),
                Scoringl1(109.95),
                Aim(40);

    
                double angle;
                WristState(double angle){
                    this.angle = angle;
                }
    
                @Override
                public Double getSetpoint() {
                   return angle;
                }
    
            }

        enum RollerState implements SetpointProvider<Double>{
            Running(1),
            Algae(0.25),
            ReverseAlgae(-0.25),
            Slow(0.5),
            ReverseSlow(-0.5),
            Stopped(0),
            Reverse(-1);

            double speed;
            RollerState(double speed){
                this.speed = speed;
            }

            @Override
            public Double getSetpoint() {
               return speed;
            }

        }

        MechanismConfig<StatefulArmMechanism<ArmState>> ARM_CONFIG = MechanismConfig.statefulArm(new ArmFeedforward(0,0,0), ArmState.StartConfiguration)
        .addMotors(AthenaMotor.KRAKEN_X60, -31)
        .setEncoder(AthenaEncoder.CANCODER, 32)
        .setNeutralMode(MotorNeutralMode.Brake)
        .setEncoderGearRatio(1d/1d)
        .setEncoderOffset(0.7705078125)
        // .setEncoderOffset(0.76416015625)
        .setUseEncoderAbsolute(true)
        .setEncoderConversion(360)
        .setCanbus(CANIVORE_CANBUS)
        .setTolerance(5)
        .setPID(0.011, 0, 0)
        .setSimulationArm(new ArmSimulationParameters()
            .setArmLengthMeters(ARM_LENGTH_METERS)
            .setMotorReduction(ARM_MOTOR_REDUCTION)
            .setMomentOfInertia(0.05)
            .setAngleRangeRadians(Units.degreesToRadians(-180), Units.degreesToRadians(180))
            .setStartingAngleRadians(Units.degreesToRadians(ArmState.StartConfiguration.getSetpoint()))
            .setUnitsPerRadian(Units.radiansToDegrees(1.0))
            .setSimulateGravity(false))
        .setVisualizationConfig(
            MechanismVisualizationConfig.builder("Arm")
                .withStaticRootPose(new Pose3d())
                .addNode("FixedArm", "Arm",
                    mech -> new Pose3d(new Translation3d(FIXED_ARM_LENGTH_METERS, 0.0, 0.0), new Rotation3d()))
                .addNode("ArmTip", "FixedArm",
                    mech -> {
                        double angleRad = Units.degreesToRadians(mech.getPosition());
                        double x = ARM_LENGTH_METERS * Math.cos(angleRad);
                        double y = ARM_LENGTH_METERS * Math.sin(angleRad);
                        return new Pose3d(new Translation3d(x, y, 0.0), new Rotation3d(0.0, 0.0, angleRad));
                    })
                .build())
        // .setPIDIZone(7)
        // .setProfiledPID(new ProfiledPIDController(0.000, 0, 0, new Constraints(50, 50)))
        .setCurrentLimit(60);

        MechanismConfig<StatefulArmMechanism<WristState>> WRIST_CONFIG = MechanismConfig.statefulArm(new ArmFeedforward(0,0,0), WristState.StartConfiguration)
        .addMotors(AthenaMotor.KRAKEN_X60, -36)
        .setEncoder(AthenaEncoder.CANCODER, 35)
        .setNeutralMode(MotorNeutralMode.Brake)
        .setEncoderOffset(0.2587890625)
        // .setEncoderOffset(0.20849609375)
        .setEncoderGearRatio(1d/1d)
        .setTolerance(5)
        .setUseEncoderAbsolute(true)
        .setEncoderConversion(360)
        .setCanbus(CANIVORE_CANBUS)
        .setPID(0.009, 0, 0.00)
        .setSimulationArm(new ArmSimulationParameters()
            .setArmLengthMeters(WRIST_LENGTH_METERS)
            .setMotorReduction(WRIST_MOTOR_REDUCTION)
            .setAngleRangeRadians(Units.degreesToRadians(-180), Units.degreesToRadians(180))
            .setStartingAngleRadians(Units.degreesToRadians(WristState.StartConfiguration.getSetpoint()))
            .setUnitsPerRadian(Units.radiansToDegrees(1.0))
            .setSimulateGravity(false))
        .setVisualizationConfig(
            MechanismVisualizationConfig.builder("Wrist")
                .withStaticRootPose(new Pose3d())
                .addNode("WristTip", "Wrist",
                    mech -> {
                        double angleRad = Units.degreesToRadians(mech.getPosition());
                        double x = WRIST_LENGTH_METERS * Math.cos(angleRad);
                        double y = WRIST_LENGTH_METERS * Math.sin(angleRad);
                        return new Pose3d(new Translation3d(x, y, 0.0), new Rotation3d(0.0, 0.0, angleRad));
                    })
                .build())
        // .setProfiledPID(new ProfiledPIDController(0, 0, 0, new Constraints(0, 0)))
        .setCurrentLimit(60);

        MechanismConfig<StatefulMechanism<RollerState>> CORAL_ROLLERS = MechanismConfig.statefulGeneric(RollerState.Stopped)
        .addMotors(AthenaMotor.KRAKEN_X60, 37)
        .setNeutralMode(MotorNeutralMode.Brake)
        .setCanbus(CANIVORE_CANBUS)
        .setCurrentLimit(10)
        .setUseSetpointAsOutput(true);

        MechanismConfig<StatefulMechanism<RollerState>> ALGAE_ROLLERS = MechanismConfig.statefulGeneric(RollerState.Stopped)
        .addMotors(AthenaMotor.KRAKEN_X60, 33)
        .setNeutralMode(MotorNeutralMode.Brake)
        .setCanbus(CANIVORE_CANBUS)
        .setCurrentLimit(10)
        .setUseSetpointAsOutput(true);             

        
    }

    // public interface Turret {

    //     int TURRET_MOTOR_ID = 40;
    //     int TURRET_ENCODER_ID = 41;
    //     int HOOD_MOTOR_ID = 42;
    //     int HOOD_ENCODER_ID = 43;
    //     int SHOOTER_MOTOR_ID = 44;
    //     int SHOOTER_ENCODER_ID = 45;

    //     double TURRET_GEAR_RATIO = 100.0;
    //     double HOOD_GEAR_RATIO = 50.0;
    //     double HOOD_LENGTH_METERS = Units.inchesToMeters(8.0);

    //     enum TurretState implements SetpointProvider<Double> {
    //         Off(0.0),
    //         Hub(0.0),
    //         Neutral(90.0),
    //         Opponent(180.0);

    //         double angleDeg;
    //         TurretState(double angleDeg) {
    //             this.angleDeg = angleDeg;
    //         }

    //         @Override
    //         public Double getSetpoint() {
    //             return angleDeg;
    //         }
    //     }

    //     enum HoodState implements SetpointProvider<Double> {
    //         Stow(0.0),
    //         Low(15.0),
    //         High(45.0);

    //         double angleDeg;
    //         HoodState(double angleDeg) {
    //             this.angleDeg = angleDeg;
    //         }

    //         @Override
    //         public Double getSetpoint() {
    //             return angleDeg;
    //         }
    //     }

    //     enum ShooterState implements SetpointProvider<Double> {
    //         Off(0.0),
    //         SpinUp(rpm(3000.0)),
    //         Fire(rpm(4500.0));

    //         double radiansPerSecond;
    //         ShooterState(double radiansPerSecond) {
    //             this.radiansPerSecond = radiansPerSecond;
    //         }

    //         @Override
    //         public Double getSetpoint() {
    //             return radiansPerSecond;
    //         }
    //     }

    //     record TurretTuple(TurretState turret, HoodState hood, ShooterState shooter) {}

    //     enum TurretSuperState implements SetpointProvider<TurretTuple> {
    //         Stowed(new TurretTuple(TurretState.Off, HoodState.Stow, ShooterState.Off)),
    //         Aim(new TurretTuple(TurretState.Neutral, HoodState.Low, ShooterState.SpinUp)),
    //         Fire(new TurretTuple(TurretState.Neutral, HoodState.High, ShooterState.Fire));

    //         private final TurretTuple setpoint;

    //         TurretSuperState(TurretTuple setpoint) {
    //             this.setpoint = setpoint;
    //         }

    //         @Override
    //         public TurretTuple getSetpoint() {
    //             return setpoint;
    //         }
    //     }

    //     MechanismConfig<StatefulTurretMechanism<TurretState>> TURRET_CONFIG =
    //         MechanismConfig.statefulTurret(new SimpleMotorFeedforward(0.2, 0.1, 0.0), TurretState.Off)
    //             .addMotors(AthenaMotor.KRAKEN_X60, TURRET_MOTOR_ID)
    //             .setEncoder(AthenaEncoder.CANCODER, TURRET_ENCODER_ID)
    //             .setNeutralMode(MotorNeutralMode.Brake)
    //             .setEncoderGearRatio(TURRET_GEAR_RATIO)
    //             .setEncoderConversion(360.0)
    //             .setUseEncoderAbsolute(true)
    //             .setCanbus(CANIVORE_CANBUS)
    //             .setPID(0.02, 0.0, 0.0)
    //             .setBounds(0.0, 270.0)
    //             .setSimulationSimpleMotor(new SimpleMotorSimulationParameters()
    //                 .setMomentOfInertia(0.02));

    //     MechanismConfig<StatefulArmMechanism<HoodState>> HOOD_CONFIG =
    //         MechanismConfig.statefulArm(new ArmFeedforward(0.1, 0.2, 0.0, 0.0), HoodState.Stow)
    //             .addMotors(AthenaMotor.KRAKEN_X60, HOOD_MOTOR_ID)
    //             .setEncoder(AthenaEncoder.CANCODER, HOOD_ENCODER_ID)
    //             .setNeutralMode(MotorNeutralMode.Brake)
    //             .setEncoderGearRatio(HOOD_GEAR_RATIO)
    //             .setEncoderConversion(360.0)
    //             .setUseEncoderAbsolute(true)
    //             .setCanbus(CANIVORE_CANBUS)
    //             .setPID(0.02, 0.0, 0.0)
    //             .setBounds(0.0, 70.0)
    //             .setSimulationArm(new ArmSimulationParameters()
    //                 .setArmLengthMeters(HOOD_LENGTH_METERS)
    //                 .setMotorReduction(HOOD_GEAR_RATIO)
    //                 .setAngleRangeRadians(Units.degreesToRadians(0.0), Units.degreesToRadians(70.0))
    //                 .setStartingAngleRadians(Units.degreesToRadians(HoodState.Stow.getSetpoint()))
    //                 .setUnitsPerRadian(Units.radiansToDegrees(1.0))
    //                 .setSimulateGravity(false));

    //     MechanismConfig<StatefulFlywheelMechanism<ShooterState>> SHOOTER_CONFIG =
    //         MechanismConfig.statefulFlywheel(new SimpleMotorFeedforward(0.15, 0.12, 0.0), ShooterState.Off)
    //             .addMotors(AthenaMotor.KRAKEN_X44, SHOOTER_MOTOR_ID)
    //             .setEncoder(AthenaEncoder.CANCODER, SHOOTER_ENCODER_ID)
    //             .setNeutralMode(MotorNeutralMode.Coast)
    //             .setEncoderGearRatio(1.0)
    //             .setEncoderConversion(2.0 * Math.PI)
    //             .setCanbus(CANIVORE_CANBUS)
    //             .setPID(0.1, 0.0, 0.0)
    //             .setPidUseVelocity(true)
    //             .setBounds(0.0, rpm(5000.0))
    //             .setSimulationSimpleMotor(new SimpleMotorSimulationParameters()
    //                 .setMomentOfInertia(0.01));

    //     SuperstructureConfig<TurretSuperState, TurretTuple> TURRET_SUPERSTRUCTURE_CONFIG =
    //         SuperstructureConfig.create(TurretSuperState.Stowed)
    //             .addMechanism(TURRET_CONFIG, TurretTuple::turret)
    //             .addMechanism(HOOD_CONFIG, TurretTuple::hood)
    //             .addMechanism(SHOOTER_CONFIG, TurretTuple::shooter)
    //             .setStateMachineDelay(Units.millisecondsToSeconds(40));

    //     static double rpm(double rpm) {
    //         return Units.rotationsPerMinuteToRadiansPerSecond(rpm);
    //     }

    // }

    public interface Superstructure {

        record EndEffectorTuple(EndEffector.RollerState coralRollerState,
                                EndEffector.RollerState algaeRollerState,
                                EndEffector.ArmState joint1state,
                                EndEffector.WristState joint2state) {}

        enum EndEffectorState implements SetpointProvider<EndEffectorTuple> {
            L4(new EndEffectorTuple(null, null, EndEffector.ArmState.ScoringL4, EndEffector.WristState.ScoringL4)),
            L3(new EndEffectorTuple(null, null, EndEffector.ArmState.Scoring, EndEffector.WristState.Scoring)),
            L2(new EndEffectorTuple(null, null, EndEffector.ArmState.Scoring, EndEffector.WristState.Scoring)),
            L1(new EndEffectorTuple(null, null, EndEffector.ArmState.Scoringl1, EndEffector.WristState.Scoringl1)),
            Score(new EndEffectorTuple(EndEffector.RollerState.Running, EndEffector.RollerState.Running, null, null)),
            AlgaeScore(new EndEffectorTuple(EndEffector.RollerState.Stopped, EndEffector.RollerState.Stopped, EndEffector.ArmState.AlgaeScore, EndEffector.WristState.AlgaeScore)),
            ScoreAlgae(new EndEffectorTuple(EndEffector.RollerState.Stopped, EndEffector.RollerState.Running, EndEffector.ArmState.AlgaeScore, EndEffector.WristState.AlgaeScore)),

            Stop(new EndEffectorTuple(EndEffector.RollerState.Stopped, EndEffector.RollerState.Stopped, null, null)),
            StartConfiguration(new EndEffectorTuple(EndEffector.RollerState.Stopped, EndEffector.RollerState.Stopped, EndEffector.ArmState.StartConfiguration, EndEffector.WristState.StartConfiguration)),
            Home(new EndEffectorTuple(EndEffector.RollerState.Stopped, EndEffector.RollerState.Stopped, EndEffector.ArmState.Home, EndEffector.WristState.Home)),

            Reverse(new EndEffectorTuple(EndEffector.RollerState.Reverse, EndEffector.RollerState.Reverse, null, null)),
            Intaking(new EndEffectorTuple(EndEffector.RollerState.Running, EndEffector.RollerState.Slow, EndEffector.ArmState.Intaking, EndEffector.WristState.Intaking)),
            AlgaeHigh(new EndEffectorTuple(EndEffector.RollerState.Stopped, EndEffector.RollerState.Reverse, EndEffector.ArmState.AlgaeHigh, EndEffector.WristState.AlgaeHigh)),
            AlgaeLow(new EndEffectorTuple(EndEffector.RollerState.Stopped, EndEffector.RollerState.Reverse, EndEffector.ArmState.AlgaeLow, EndEffector.WristState.AlgaeLow)),

            Transition(new EndEffectorTuple(EndEffector.RollerState.Stopped, EndEffector.RollerState.Stopped, EndEffector.ArmState.TransitionState, EndEffector.WristState.TransitionState));

            private final EndEffectorTuple states;
            EndEffectorState(EndEffectorTuple states) {
                this.states = states;
            }
            @Override
            public EndEffectorTuple getSetpoint() {
                return states;
            }
        }

        record SuperstructureTuple(EndEffectorState endEffector, Elevator.ElevatorState elevator) {}

        enum SuperstructureState implements SetpointProvider<SuperstructureTuple> {
            AlgaeHigh(new SuperstructureTuple(EndEffectorState.AlgaeHigh, Elevator.ElevatorState.AlgaeHigh)),
            NONE(new SuperstructureTuple(null, null)),

            AlgaeLow(new SuperstructureTuple(EndEffectorState.AlgaeLow, Elevator.ElevatorState.AlgaeLow)),
            AlgaeScore(new SuperstructureTuple(EndEffectorState.AlgaeScore, Elevator.ElevatorState.L4)),
            ScoreAlgae(new SuperstructureTuple(EndEffectorState.ScoreAlgae, null)),

            AlgaeSpit(new SuperstructureTuple(EndEffectorState.AlgaeScore, null)),

            L4(new SuperstructureTuple(EndEffectorState.L4, Elevator.ElevatorState.L4)),
            L3(new SuperstructureTuple(EndEffectorState.L3, Elevator.ElevatorState.L3)),
            L2(new SuperstructureTuple(EndEffectorState.L2, Elevator.ElevatorState.L2)),
            L1(new SuperstructureTuple(EndEffectorState.L1, Elevator.ElevatorState.L1)),

            StartConfiguration(new SuperstructureTuple(EndEffectorState.StartConfiguration, Elevator.ElevatorState.HomeReset)),
            Align(new SuperstructureTuple(EndEffectorState.Home, Elevator.ElevatorState.Aligning)),
            Home(new SuperstructureTuple(EndEffectorState.Home, Elevator.ElevatorState.HomeReset)),
            Stopped(new SuperstructureTuple(EndEffectorState.Stop, null)),
            HomePID(new SuperstructureTuple(EndEffectorState.Home, Elevator.ElevatorState.HomePID)),

            Score(new SuperstructureTuple(EndEffectorState.Score, null)),
            Intaking(new SuperstructureTuple(EndEffectorState.Intaking, Elevator.ElevatorState.Intaking));

            private final SuperstructureTuple states;
            SuperstructureState(SuperstructureTuple states) {
                this.states = states;
            }

            @Override
            public SuperstructureTuple getSetpoint() {
                return states;
            }
        }

        GenericLimitSwitchConfig HAS_GAME_PIECE_SENSOR = GenericLimitSwitchConfig.create(-4).setDelay(Units.millisecondsToSeconds(40));

        SuperstructureConfig<EndEffectorState, EndEffectorTuple> ENDEFFECTOR_CONFIG = SuperstructureConfig.create(EndEffectorState.Home)
                        .addMechanism(EndEffector.ARM_CONFIG, EndEffectorTuple::joint1state)
                        .addMechanism(EndEffector.WRIST_CONFIG, EndEffectorTuple::joint2state)
                        .addMechanism(EndEffector.CORAL_ROLLERS, EndEffectorTuple::coralRollerState)
                        .addMechanism(EndEffector.ALGAE_ROLLERS, EndEffectorTuple::algaeRollerState)
                        .addInput("hasPiece", HAS_GAME_PIECE_SENSOR) //if name not defined we can set or override it here
                        .setStateMachineDelay(Units.millisecondsToSeconds(40));

        SuperstructureConfig<SuperstructureState, SuperstructureTuple> SUPERSTRUCTURE_CONFIG = SuperstructureConfig.create(SuperstructureState.Home)
                        .addMechanism(Elevator.ELEVATOR_CONFIG, SuperstructureTuple::elevator)
                        .addMechanism(ENDEFFECTOR_CONFIG, SuperstructureTuple::endEffector)
                        .addConstraint(SuperstructureState.Intaking,
                                ctx -> ctx.getMechanisms().superstructure(SuperstructureTuple::endEffector).input("hasPiece") //ctx should not merge inputs from other mechanisms and ctx should use same getmechanism syntax / method
                                        ? ctx.getMechanisms().elevator(SuperstructureTuple::elevator).getStateMachine().atState(Elevator.ElevatorState.Intaking)
                                        : true)
                        .setSimulation(sim -> sim
                                .attachResolved(
                                        ctx -> ctx.getMechanisms()
                                                .superstructure(SuperstructureTuple::endEffector)
                                                .getMechanisms()
                                                .arm(EndEffectorTuple::joint1state),
                                        ctx -> {
                                            var elevator = ctx.getMechanisms().elevator(SuperstructureTuple::elevator);
                                            Pose3d carriagePose = elevator.getMechanism3dPoses().get("Carriage");
                                            if (carriagePose != null) {
                                                return carriagePose;
                                            }
                                            double elevatorMeters = Units.inchesToMeters(elevator.getPosition());
                                            return new Pose3d(1.5, elevatorMeters, 0.0, new Rotation3d(0,180,0));
                                        })
                                .attachResolved(
                                        ctx -> ctx.getMechanisms()
                                                .superstructure(SuperstructureTuple::endEffector)
                                                .getMechanisms()
                                                .arm(EndEffectorTuple::joint2state),
                                        ctx -> {
                                            var arm = ctx.getMechanisms()
                                                    .superstructure(SuperstructureTuple::endEffector)
                                                    .getMechanisms()
                                                    .arm(EndEffectorTuple::joint1state);
                                            Pose3d armTip = arm.getMechanism3dPoses().get("ArmTip");
                                            if (armTip != null) {
                                                return armTip;
                                            }
                                            var elevator = ctx.getMechanisms().elevator(SuperstructureTuple::elevator);
                                            double elevatorMeters = Units.inchesToMeters(elevator.getPosition());
                                            double armAngle = Units.degreesToRadians(arm.getPosition());
                                            double x = EndEffector.FIXED_ARM_LENGTH_METERS
                                                    + EndEffector.ARM_LENGTH_METERS * Math.cos(armAngle);
                                            double y = elevatorMeters
                                                    + EndEffector.ARM_LENGTH_METERS * Math.sin(armAngle);
                                            return new Pose3d(x, y, 0.0, new Rotation3d(0.0, 0.0, armAngle));
                                        }))
                        .setStateMachineDelay(Units.millisecondsToSeconds(40));
    }
}
