package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import ca.frc6390.athena.core.RobotBase.RobotBaseConfig;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.DrivetrainIDs;
import ca.frc6390.athena.core.RobotLocalization.RobotLocalizationConfig;
import ca.frc6390.athena.devices.EncoderConfig.EncoderType;
import ca.frc6390.athena.devices.IMU.IMUType;
import ca.frc6390.athena.devices.MotorController.Motor;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorNeutralMode;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrainConfig;
import ca.frc6390.athena.drivetrains.swerve.modules.SwerveVendorSDS;
import ca.frc6390.athena.mechanisms.MechanismConfig;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.Mechanism.StatefulMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import ca.frc6390.athena.sensors.camera.photonvision.PhotonVisionConfig;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.limelight.LimeLightConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;;

public interface Constants {
    
    String CANIVORE_CANBUS = "can";
    public interface DriveTrain {

        double TRACKWIDTH_METERS = Units.inchesToMeters(18.375); 
    
        //SIREN
        double[] ENCODER_OFFSETS = {0.23535156250000003,0.09350585937499999,0.19873046875000003,0.361572265625};
        
        SwerveDrivetrainConfig DRIVETRAIN_CONFIG = SwerveDrivetrainConfig.defualt(TRACKWIDTH_METERS)
                                                    .setIMU(IMUType.CTREPigeon2, false)
                                                    .setIds(DrivetrainIDs.SWERVE_CHASSIS_STANDARD)
                                                    .modules(
                                                            SwerveVendorSDS.MK4n.L1_PLUS.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setP(0.45), 
                                                            SwerveVendorSDS.MK4n.L1_PLUS.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setP(0.45), 
                                                            SwerveVendorSDS.MK4i.L1_PLUS.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setP(0.5), 
                                                            SwerveVendorSDS.MK4i.L1_PLUS.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setP(0.5)
                                                            )   
                                                    .setEncoderOffset(ENCODER_OFFSETS)
                                                    .setCanbus(CANIVORE_CANBUS)
                                                    .setCurrentLimit(80);

        RobotLocalizationConfig LOCALIZATION_CONFIG = RobotLocalizationConfig.vision(0.1, 0.1, 9999)
                                                            .setAutoPlannerPID(7,0,0, 2,0,0).setVisionEnabled(true);
        ConfigurableCamera[] CAMERAS =
         {                                                                 
        LimeLightConfig.table("limelight-left").setUseForLocalization(true).setYawRelativeToForwards(-15).setPoseEstimateType(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).setLocalizationTagFilter(17,18,19,20,21,22,6,7,8,9,10,11), 
        LimeLightConfig.table("limelight-right").setUseForLocalization(true).setYawRelativeToForwards(15).setPoseEstimateType(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).setLocalizationTagFilter(17,18,19,20,21,22,6,7,8,9,10,11),
        PhotonVisionConfig.table("Tag").setUseForLocalization(true).setCameraRobotSpace(new Transform3d(-0.29845,0.2286,Units.inchesToMeters(36),new Rotation3d(0, 0, 180))).setPoseStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE)
        };

        //X -11.75
        //Y 9 inches
        //Z 33.5
        RobotBaseConfig<SwerveDrivetrain> ROBOT_BASE = RobotBaseConfig.swerve(DRIVETRAIN_CONFIG)
                                                                      .setLocalization(LOCALIZATION_CONFIG)
                                                                      .setVision(CAMERAS);

        
    }

    public interface Controllers {
        double STICK_DEADZONE = 0.15;      
    }

    public interface Elevator {
        int ENCODER = 42;

        int LEFT_MOTOR = 20;
        int RIGHT_MOTOR = 21;
        double GEAR_DIAMETER_INCHES = 3d;
        double OFFSET_FROM_FLOOR = 24;
        double ENCODER_GEAR_RATIO = 1d/1d;
        double MOTOR_GEAR_RATIO = 7d/1d;
        int LIMIT_SWITCH = 5;
        ProfiledPIDController CONTORLLER = new ProfiledPIDController(0.1, 0.0, 0, new Constraints(50, 17));
        ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0.14, 0.32,0.0);
    }

    public interface EndEffector {

        int CANDLE_ID = 22;

        enum ArmState implements SetpointProvider<Double>{
            Intaking(150.38085937),
            AlgaeHigh(0),
            AlgaeLow(0),

            Home(0),
            Scoring(78.310546875),
            TransitionState(65),
            ScoringL4(60),

            Scoringl1(78.310546875);

            double angle;
            ArmState(double angle){
                this.angle = angle;
            }

            @Override
            public Double getSetpoint() {
               return angle;
            }

        }

            enum WristState implements SetpointProvider<Double>{
                Intaking(62),
                Home(0d),
                Scoring(125.419921875),
                AlgaeHigh(0),
                AlgaeLow(0),
                ScoringL4(80),
                TransitionState(62),
                Scoringl1(50);

    
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

        MechanismConfig<StatefulArmMechanism<ArmState>> ARM_CONFIG = MechanismConfig.statefulArm(new ArmFeedforward(0,0,0), ArmState.Home)
        .addMotors(Motor.KRAKEN_X60, -31)
        .setEncoder(EncoderType.CTRECANcoder, 32)
        .setNeutralMode(MotorNeutralMode.Brake)
        .setEncoderGearRatio(1d/1d)
        .setEncoderOffset(0.380126953125)
        .setUseEncoderAbsolute(true)
        .setEncoderConversion(360)
        .setCanbus(CANIVORE_CANBUS)
        .setTolerance(6)
        .setPID(0.009, 0, 0)
        .setCurrentLimit(60);
        
        MechanismConfig<StatefulArmMechanism<WristState>> WRIST_CONFIG = MechanismConfig.statefulArm(new ArmFeedforward(0,0,0), WristState.Home)
        .addMotors(Motor.KRAKEN_X60, -36)
        .setEncoder(EncoderType.CTRECANcoder, 35)
        .setNeutralMode(MotorNeutralMode.Brake)
        .setEncoderOffset(0.2587890625)
        .setEncoderGearRatio(1d/1d)
        .setTolerance(2)
        .setUseEncoderAbsolute(true)
        .setEncoderConversion(360)
        .setCanbus(CANIVORE_CANBUS)
        .setPID(0.025, 0, 0)
        .setCurrentLimit(60);

        MechanismConfig<StatefulMechanism<RollerState>> CORAL_ROLLERS = MechanismConfig.statefulGeneric(RollerState.Stopped)
        .addMotors(Motor.KRAKEN_X60, 37)
        .setNeutralMode(MotorNeutralMode.Brake)
        .setCanbus(CANIVORE_CANBUS)
        .setCurrentLimit(60)
        .setUseSetpointAsOutput(true);

        MechanismConfig<StatefulMechanism<RollerState>> ALGAE_ROLLERS = MechanismConfig.statefulGeneric(RollerState.Stopped)
        .addMotors(Motor.KRAKEN_X60, 33)
        .setNeutralMode(MotorNeutralMode.Brake)
        .setCanbus(CANIVORE_CANBUS)
        .setCurrentLimit(60)
        .setUseSetpointAsOutput(true);                                                                                                                                          
    }
}
