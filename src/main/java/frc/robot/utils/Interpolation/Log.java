// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Interpolation;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Reader;
import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.annotations.JsonAdapter;
import com.google.gson.stream.JsonReader;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Log extends InstantCommand {
  
  public List<Logger> supplier;
  public List<LogData> dataPoints;
  public String fileName;
  public Log(String fileName, Logger... supplier) {
    this.fileName = fileName;
    this.supplier = List.of(supplier);
  }

  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() 
  {
     
  }

  public InstantCommand LogToJson()
  {
    return new InstantCommand
    (
      ()->
      {
      for(Logger logger : supplier)
      {
      Gson gson = new Gson();
      dataPoints.add(new LogData(logger.getName(), logger.getValue()));
      try {
        FileWriter writer = new FileWriter(fileName);
        gson.toJson(dataPoints, writer);
      } catch (IOException e) {
        e.printStackTrace();
      }
      }
      }
    );
  }
}
