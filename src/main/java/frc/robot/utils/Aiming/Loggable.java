package frc.robot.utils.Aiming;

import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import edu.wpi.first.math.Pair;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableListenerPoller;
import edu.wpi.first.wpilibj2.command.button.NetworkButton;

public class Loggable
{
    private String fileName;
    private List<Pair<String, DoubleSupplier>> dataSupplier;

    public Loggable(String fileName, Pair<String, DoubleSupplier>... dataSupplier)
    {
        this.fileName = fileName;
        this.dataSupplier = List.of(dataSupplier);
    }

    public void LogDataToJson()
    {

        JSONArray logArray = new JSONArray();

        try (FileReader reader = new FileReader(fileName))
        {
            Object parsed = new JSONParser().parse(reader);
            if (parsed instanceof JSONArray)
            {
                logArray = (JSONArray) parsed;
            }
        }
        catch (IOException | ParseException e)
        {

        }

        JSONObject entry = new JSONObject();
        for (Pair<String, DoubleSupplier> loggable : dataSupplier)
        {
            entry.put(loggable.getFirst(), loggable.getSecond().getAsDouble());
        }

        logArray.add(entry);

        try (FileWriter file = new FileWriter(fileName))
        {
            file.write(logArray.toJSONString());
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    

   
}


