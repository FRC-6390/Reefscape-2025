package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class Autos {
    
  public enum AUTOS {
        
        LEFTSIDE("Choreo"),
        RIGHTSIDE("ChoreoRight"),
        TESTLEFT("ChoreoTestLeft"),
        TESTRIGHT("ChoreoTestRight"),
        TESTMID("ChoreoTestMid"),
        PRELOADLEFT("PreLoadLeft"),
        PRELOADRIGHT("PreLoadRight"),
        PRELOADMID("PreLoadMid");

        private final String auto;
    
        AUTOS(String auto){
            this.auto = auto;
        }

        public PathPlannerAuto getAuto()
        {
          return new PathPlannerAuto(this.auto);
        }

        public static SendableChooser<Command> createChooser(AUTOS defualtAuto){
            SendableChooser<Command> chooser = new SendableChooser<>();
            for (AUTOS auto : AUTOS.values()) {
                chooser.addOption(auto.name(), auto.getAuto());
            }
            chooser.setDefaultOption(defualtAuto.name(), defualtAuto.getAuto());
            return chooser;
        }
    }

    public enum PATHS {
        
      SIDEA("SideA"),
      SIDEB("SideB"),

      SIDEC("SideC"),
      SIDED("SideD"),

      SIDEE("SideE"),
      SIDEF("SideF"),

      SIDEG("SideG"),
      SIDEH("SideH"),

      SIDEI("SideI"),
      SIDEJ("SideJ"),

      SIDEK("SideK"),
      SIDEL("SideL");


      
      private PathPlannerPath path;
      private final String pathName;
      
        
      PATHS(String pathName){
        this.pathName = pathName;
        try {
          this.path = PathPlannerPath.fromPathFile(pathName);
        } catch (FileVersionException | IOException | ParseException e) {
            e.printStackTrace();
            this.path =null;
          }
      }

      public PathPlannerPath getPath()
      {
        return path;
      }

      public String getPathName() {
          return pathName;
      }
  }
}
