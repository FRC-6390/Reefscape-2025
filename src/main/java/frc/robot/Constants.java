package frc.robot;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import ca.frc6390.athena.core.RobotBase.RobotBaseConfig;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.DrivetrainIDs;
import ca.frc6390.athena.core.RobotLocalization.RobotLocalizationConfig;
import ca.frc6390.athena.devices.EncoderConfig.EncoderType;
import ca.frc6390.athena.devices.IMU.IMUType;
import ca.frc6390.athena.devices.MotorController.Motor;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorControllerType;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorNeutralMode;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrainConfig;
import ca.frc6390.athena.drivetrains.swerve.modules.SwerveVendorSDS;
import ca.frc6390.athena.mechanisms.Mechanism;
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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.superstructure.EndEffector.EndEffectorTuple;

public interface Constants {
    
    String CANIVORE_CANBUS = "can";
    public interface DriveTrain {

        double TRACKWIDTH_METERS = Units.inchesToMeters(18.375); 
        

        //SIREN
        double[] ENCODER_OFFSETS = {0.23535156250000003,0.09350585937499999,0.19873046875000003,0.361572265625};
        
        //PATRICK
        // double[] ENCODER_OFFSETS = {0.863037109375,0.201171875,0.6755371093749999,0.882080078125};

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

        //PATRICK
        // SwerveDrivetrainConfig DRIVETRAIN_CONFIG = SwerveDrivetrainConfig.defualt(TRACKWIDTH_METERS)
        // .setIMU(IMUType.CTREPigeon2, false)
        // .setIds(DrivetrainIDs.SWERVE_CHASSIS_STANDARD)
        // .modules(
        //         SwerveVendorSDS.MK4i.L3.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setP(0.45),
        //         SwerveVendorSDS.MK4i.L3.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setP(0.45),
        //         SwerveVendorSDS.MK4i.L3.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setP(0.45),
        //         SwerveVendorSDS.MK4i.L3.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setP(0.45)
        //         )   
        // .setEncoderOffset(ENCODER_OFFSETS)
        // .setCanbus(CANIVORE_CANBUS);

        RobotLocalizationConfig LOCALIZATION_CONFIG = RobotLocalizationConfig.vision(0.3, 0.3, 9999)
                                                            .setAutoPlannerPID(5,0,0, 2,0,0).setVisionEnabled(true);
        ConfigurableCamera[] CAMERAS =
         {                                                                 
        LimeLightConfig.table("limelight-left").setUseForLocalization(true).setYawRelativeToForwards(-15).setPoseEstimateType(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).setLocalizationTagFilter(17,18,19,20,21,22,6,7,8,9,10,11), 
        LimeLightConfig.table("limelight-right").setUseForLocalization(true).setYawRelativeToForwards(15).setPoseEstimateType(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE).setLocalizationTagFilter(17,18,19,20,21,22,6,7,8,9,10,11),
        PhotonVisionConfig.table("OV9281").setUseForLocalization(false).setCameraRobotSpace(new Transform3d(-0.29845,0.2286,1,new Rotation3d(0, 0, 180))).setPoseStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR)
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
    public interface Climber {
        int LIMIT_SWITCH = 1;
        int ENCODER = 42;
        int LEFT_MOTOR = 43;
        int RIGHT_MOTOR = 44;
        double ENCODER_OFFSET = 0;
        double ENCODER_GEAR_RATIO = 4d/1d;
        PIDController CONTORLLER = new PIDController(0.015, 0, 0);

        enum ClimberState implements SetpointProvider<Double>{
            Home(0);

            double angle;
            ClimberState(double angle){
                this.angle = angle;
            }

            @Override
            public Double getSetpoint() {
               return angle;
            }

        }


        MechanismConfig<StatefulMechanism<ClimberState>> CLIMBER_CONFIG = MechanismConfig.statefulGeneric(ClimberState.Home)
                                                                                        .addMotors(Motor.KRAKEN_X60, 24, -26).addMotor(Motor.FALCON_500, 21)
                                                                                        .setEncoderFromMotor(24)
                                                                                        .setNeutralMode(MotorNeutralMode.Brake)
                                                                                        .setEncoderGearRatio(1d/4d)
                                                                                        .setEncoderConversion(360)
                                                                                        .setCanbus(CANIVORE_CANBUS)
                                                                                        .setPID(0.015, 0, 0)
                                                                                        .setCurrentLimit(60);
                                                                                        // .addLimitSwitch(-4, 0, true ,-1);
                                                                                       

    }

    public interface Elevator {
        int ENCODER = 42;

        int LEFT_MOTOR = 20;
        int RIGHT_MOTOR = 21;
        double GEAR_DIAMETER_INCHES = 3d;
        double OFFSET_FROM_FLOOR = 24;
        double ENCODER_GEAR_RATIO = 1d/1d;
        double MOTOR_GEAR_RATIO = 6d/1d;
        int LIMIT_SWITCH = 5;
        //MAX ACCEL WAS 15
        // ProfiledPIDController CONTORLLER = new ProfiledPIDController(0.01, 0.01, 0, new Constraints(50, 10));
        // ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0.208, 0.00,0.0);
        ProfiledPIDController CONTORLLER = new ProfiledPIDController(0.075, 0.0, 0, new Constraints(50, 5));
        ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0.165, 0.377,0.0);

        // ProfiledPIDController CONTORLLER = new ProfiledPIDController(0.11, 0, 0, new Constraints(60, 30));
        // ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0.129, 0.377,0.75);
        // ProfiledPIDController CONTORLLER = new ProfiledPIDController(0.2, 0, 0, new Constraints(0.5, 0.5));
        // ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0.12875, 0.688,0.00);

        // ProfiledPIDController CONTORLLER = new ProfiledPIDController(0.11, 0.0095, 0, new Constraints(0.5, 0), 5);
        // ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0.04, 2.67,0.006666667);

        // MechanismConfig<StatefulElevatorMechanism<ElevatorState>> ELEVATOR_CONFIG = MechanismConfig.statefulElevator(new ElevatorFeedforward(0, 0.17, 0.377,0.78), ElevatorState.StartConfiguration)
        //                                                                             .addMotors(Motor.KRAKEN_X60, 20,21)
        //                                                                             .setEncoder(EncoderType.CTRECANcoder, 23)
        //                                                                             .setCanbus(CANBUS)
        //                                                                             .setEncoderGearRatio(1d/1d)
        //                                                                             .setEncoderConversion(2d)
        //                                                                             .setProfiledPID(0.11, 0.01, 0, new Constraints(60, 18))
        //                                                                             .addLowerLimitSwitch(0, 0, true);
    }

    public interface EndEffector {
        int LIMIT_SWITCH = 3;
        int ENCODER = 40;
        int ALGAE_MOTOR = 32;
        
        int ROLLER = 33;
        double ENCODER_OFFSET = 0.455810546875;

        double ENCODER_GEAR_RATIO = 1d/1d; //from motors 125d/1d;
        PIDController CONTORLLER = new PIDController(0.015, 0, 0);

        int CANDLE_ID = 22;

        enum ArmState implements SetpointProvider<Double>{
            Intaking(145.634765625),
            Home(0),
            Scoring(78.310546875),
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
                Intaking(45.87890625),
                Home(0d),
                Scoring(125.419921875),
                ScoringL4(80),
                TransitionState(25),
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
            Reverse(-0.2);

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
        .setTolerance(2)
        .setPID(0.008, 0, 0)
        .setCurrentLimit(20);
        
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
        .setPID(0.008, 0, 0)
        .setCurrentLimit(20);

        MechanismConfig<StatefulMechanism<RollerState>> ROLLER_CONFIG = MechanismConfig.statefulGeneric(RollerState.Stopped)
        .addMotors(Motor.KRAKEN_X60, 37, 33)
        .setNeutralMode(MotorNeutralMode.Brake)
        .setCanbus(CANIVORE_CANBUS)
        .setCurrentLimit(60)
        .setUseSetpointAsOutput(true);

        // echanismConfig<Mechanism> SCORER_CONFIG = MechanismConfig.generic()
                                                                    // .addMotor(Motor.KRAKEN_X60, 32)
                                                                    // .setCanbus(CANBUS);M


        // MechanismConfig<Mechanism> ALGEA_CONFIG = MechanismConfig.generic()
        //                                                             .addMotor(Motor.KRAKEN_X60, 33)
        //                                                             .setCanbus(CANBUS)
        //                                                             .setEncoderFromMotor(32)
        //                                                             .addUpperLimitSwitch(2, 0, true);      
                                                                                                                                          
    }
}
