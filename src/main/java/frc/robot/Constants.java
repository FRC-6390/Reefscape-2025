package frc.robot;

import ca.frc6390.athena.core.RobotBase.RobotBaseConfig;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.DrivetrainIDs;
import ca.frc6390.athena.core.RobotLocalization.RobotLocalizationConfig;
import ca.frc6390.athena.devices.EncoderConfig.EncoderType;
import ca.frc6390.athena.devices.IMU.IMUType;
import ca.frc6390.athena.devices.MotorController.Motor;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrainConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import ca.frc6390.athena.drivetrains.swerve.modules.SwerveVendorSDS;
import edu.wpi.first.math.util.Units;


public interface Constants {
    
    String CANIVORE_CANBUS = "can";
    public interface DriveTrain {

        double TRACKWIDTH_METERS = Units.inchesToMeters(20); 
        
        SwerveDrivetrainConfig DRIVETRAIN_CONFIG = SwerveDrivetrainConfig.standard(TRACKWIDTH_METERS)
                                                                        .modules(
                                                                            SwerveVendorSDS.MK4i.L3.config(Motor.KRAKEN_X60, EncoderType.CTRECANcoder).setP(0.5),
                                                                            SwerveVendorSDS.MK4i.L3.config(Motor.KRAKEN_X60, EncoderType.CTRECANcoder).setP(0.5),
                                                                            SwerveVendorSDS.MK4i.L3.config(Motor.KRAKEN_X60, EncoderType.CTRECANcoder).setP(0.5),
                                                                            SwerveVendorSDS.MK4i.L3.config(Motor.KRAKEN_X60, EncoderType.CTRECANcoder).setP(0.5)
                                                                        )
                                                                        .setIMU(IMUType.CTREPigeon2, false)
                                                                        .setCanbus(CANIVORE_CANBUS)
                                                                        .setEncoderOffset(0.86572265625+0.5,0.1965332031+0.5,0.667724609375+0.5,0.883544921875+0.5);
        RobotLocalizationConfig LOCALIZATION_CONFIG = RobotLocalizationConfig.defualt().setAutoPlannerPID(7, 0, 0, 2, 0,
                0);
        
        RobotBaseConfig<SwerveDrivetrain> ROBOT_BASE = RobotBaseConfig.swerve(DRIVETRAIN_CONFIG).setLocalization(LOCALIZATION_CONFIG);
    }

    public interface Controllers {
        double STICK_DEADZONE = 0.15;      
    }

}
