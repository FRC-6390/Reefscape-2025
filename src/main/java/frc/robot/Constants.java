package frc.robot;

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
import ca.frc6390.athena.mechanisms.MechanismConfig;
import ca.frc6390.athena.mechanisms.Mechanism.StatefulMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import ca.frc6390.athena.sensors.camera.limelight.LimeLightConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.superstructure.EndEffector.EndEffectorTuple;

public interface Constants {
    
    String CANIVORE_CANBUS = "can";
    public interface DriveTrain {

        double TRACKWIDTH_METERS = Units.inchesToMeters(18.375); 
        
        // double[] ENCODER_OFFSETS = {0.23535156250000003,0.09350585937499999,0.19873046875000003,0.361572265625};
        double[] ENCODER_OFFSETS = {0.863037109375,0.201171875,0.6755371093749999,0.882080078125};

        // SwerveDrivetrainConfig DRIVETRAIN_CONFIG = SwerveDrivetrainConfig.defualt(TRACKWIDTH_METERS)
        //                                             .setIMU(IMUType.CTREPigeon2, false)
        //                                             .setIds(DrivetrainIDs.SWERVE_CHASSIS_STANDARD)
        //                                             .modules(
        //                                                     SwerveVendorSDS.MK4n.L1_PLUS.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setP(0.45), 
        //                                                     SwerveVendorSDS.MK4n.L1_PLUS.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setP(0.45), 
        //                                                     SwerveVendorSDS.MK4i.L1_PLUS.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setP(0.5), 
        //                                                     SwerveVendorSDS.MK4i.L1_PLUS.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setP(0.5)
        //                                                     )   
        //                                             .setEncoderOffset(ENCODER_OFFSETS)
        //                                             .setCanbus(CANIVORE_CANBUS);
        SwerveDrivetrainConfig DRIVETRAIN_CONFIG = SwerveDrivetrainConfig.defualt(TRACKWIDTH_METERS)
        .setIMU(IMUType.CTREPigeon2, false)
        .setIds(DrivetrainIDs.SWERVE_CHASSIS_STANDARD)
        .modules(
                SwerveVendorSDS.MK4i.L3.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setP(0.45),
                SwerveVendorSDS.MK4i.L3.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setP(0.45),
                SwerveVendorSDS.MK4i.L3.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setP(0.45),
                SwerveVendorSDS.MK4i.L3.config(Motor.KRAKEN_X60,EncoderType.CTRECANcoder).setP(0.45)
                )   
        .setEncoderOffset(ENCODER_OFFSETS)
        .setCanbus(CANIVORE_CANBUS);

        RobotLocalizationConfig LOCALIZATION_CONFIG = RobotLocalizationConfig.vision(1, 1, 9999)
                                                            .setAutoPlannerPID(5,0,0, 2,0,0);
        LimeLightConfig[] LIMELIGHTS = {                                                                 LimeLightConfig.table("limelight-left").setYawRelativeToForwards(-90).setPoseEstimateType(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE), 
        LimeLightConfig.table("limelight-right").setYawRelativeToForwards(90).setPoseEstimateType(PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE)};
        RobotBaseConfig<SwerveDrivetrain> ROBOT_BASE = RobotBaseConfig.swerve(DRIVETRAIN_CONFIG)
                                                                      .setLocalization(LOCALIZATION_CONFIG)
                                                                      .setVision(
                                                                        LIMELIGHTS[0],
                                                                        LIMELIGHTS[1]// PhotonVisionConfig.table("OV9281").setCameraRobotSpace(new Transform3d(-0.25, -0.27 ,0.98, new Rotation3d())).setPoseStrategy(PoseStrategy.MULTI_TAG_PNP_ON_RIO)
                                                                        );

        
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
        int ENCODER = 23;

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
        ProfiledPIDController CONTORLLER = new ProfiledPIDController(0.06, 0.0, 0, new Constraints(50, 18));
        ElevatorFeedforward FEEDFORWARD = new ElevatorFeedforward(0, 0.15, 0.00,0.0);

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
        

        int MOTOR = 31;
        int ALGAE_MOTOR = 32;
        
        int ROLLER = 33;
        // double ENCODER_OFFSET = 0.551025390625;
        // double ENCODER_OFFSET = -0.044921875;
        double ENCODER_OFFSET = -0.368896484375;

        double ENCODER_GEAR_RATIO = 1d/1d; //from motors 125d/1d;
        PIDController CONTORLLER = new PIDController(0.015, 0, 0);
        // PIDController CONTORLLER = new PIDController(0.005, 0, 0);

        int CANDLE_ID = 22;

        // MechanismConfig<StatefulTurretMechanism<EndEffectorState>> ENDEFFECTOR_CONFIG = MechanismConfig.statefulTurret(new SimpleMotorFeedforward(0, 0,0,0), EndEffectorState.StartConfiguration)
        //                                                                             .addMotor(Motor.KRAKEN_X60, 31)
        //                                                                             .setEncoder(EncoderType.CTRECANcoder, 40)
        //                                                                             .setCanbus(CANBUS)
        //                                                                             .setEncoderGearRatio(1d/1d)
        //                                                                             .setEncoderConversion(360d)
        //                                                                             .setEncoderOffset(0)
        //                                                                             .setUseEncoderAbsolute(true)
        //                                                                             .setProfiledPID(0,0,0, new Constraints(0, 0));

        // MechanismConfig<Mechanism> SCORER_CONFIG = MechanismConfig.generic()
                                                                    // .addMotor(Motor.KRAKEN_X60, 32)
                                                                    // .setCanbus(CANBUS);


        // MechanismConfig<Mechanism> ALGEA_CONFIG = MechanismConfig.generic()
        //                                                             .addMotor(Motor.KRAKEN_X60, 33)
        //                                                             .setCanbus(CANBUS)
        //                                                             .setEncoderFromMotor(32)
        //                                                             .addUpperLimitSwitch(2, 0, true);      
                                                                                                                                          
    }
}
