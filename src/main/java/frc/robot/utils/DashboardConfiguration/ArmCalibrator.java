// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.DashboardConfiguration;

import java.util.List;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.devices.EncoderConfig.EncoderType;
import ca.frc6390.athena.devices.MotorController.Motor;
import ca.frc6390.athena.devices.MotorControllerConfig.MotorNeutralMode;
import ca.frc6390.athena.mechanisms.ArmMechanism;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.MechanismConfig;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.EndEffector.ArmState;
import frc.robot.utils.Aiming.Loggable;

/** Add your docs here. */


public class ArmCalibrator
{

public enum DashboardType
{
    MOTOR_ENCODER,
    PID_TUNING,
    STATE_MANAGEMENT
}

public ShuffleboardLayout tab;

public GenericEntry p;
public GenericEntry i;
public GenericEntry d;

public List<Pair<String, Double>> list;

public DashboardType type;
public String title;
public StatefulArmMechanism arm;
public List<Pair<String,GenericEntry>> entries;

public ArmCalibrator(String title, DashboardType type, StatefulArmMechanism arm, Pair<String, Double>... list)
{
    this.type = type;
    this.arm = arm;
    this.list = List.of(list);
    this.title = title;
    tab = Shuffleboard.getTab("CALIBRATION").getLayout(title);
    p = tab.add("P", 0).getEntry();
    i = tab.add("I", 0).getEntry();
    d = tab.add("D", 0).getEntry();

    for (Pair<String,Double> pair : list) 
    {
        GenericEntry entry = tab.add(pair.getFirst(), pair.getSecond()).getEntry();   
        entries.add(new Pair<String,GenericEntry>(pair.getFirst(), entry));
    }
    

    tab.add("Rebuild Arm",new InstantCommand(() -> this.arm = rebuildMech()));
    tab.add("Export Config To JSON", new InstantCommand(() -> ExportConfigAsJSON()));

}

public GenericEntry getEntryFromList(String name)
{
    for (Pair<String,GenericEntry> pair : entries) {
        if(pair.getFirst() == name)
        {
            return pair.getSecond();
        }
    }
    return null;
}

public StatefulArmMechanism rebuildMech()
{
    return 
         MechanismConfig.statefulArm(new ArmFeedforward(0,0,0), ArmState.StartConfiguration)
        .addMotors(Motor.KRAKEN_X60, -31)
        .setEncoder(EncoderType.CTRECANcoder, 32)
        .setNeutralMode(MotorNeutralMode.Brake)
        .setEncoderGearRatio(1d/1d)
        .setEncoderOffset(0.76416015625)
        .setUseEncoderAbsolute(true)
        .setEncoderConversion(360)
        .setCanbus("can")
        .setTolerance(5)
        .setPID(p.getDouble(0), i.getDouble(0), d.getDouble(0))
        .setPIDIZone(7)
        // .setProfiledPID(new ProfiledPIDController(0.000, 0, 0, new Constraints(50, 50)))
        .setCurrentLimit(60)
        .build();
}


public void ExportConfigAsJSON()
{
    Loggable loggable = new Loggable(
    "C:\\Users\\User\\Documents\\GitHub\\Reefscape-2025\\src\\main\\deploy\\armConfig.json",
    new Pair<String,DoubleSupplier>("P", () -> p.getDouble(0)),
    new Pair<String,DoubleSupplier>("I", () -> i.getDouble(0)),
    new Pair<String,DoubleSupplier>("D", () -> d.getDouble(0))
    ); 

    loggable.LogDataToJson();    
}




}
