package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;

public class Autos {
    
  public enum AUTOS {
        
        LEFTSIDE("LeftSide", false),
        LEFTLOLIPOP("LeftLoliPop", false),


        BaseLineLeft("BaseLineLeft", false),
        BaseLineMid("BaseLineMid", false),
        BaseLineRight("BaseLineRight", false),

        PreLoadMid("PreLoadMid", false);

        private final String auto;
        private final boolean choreo;
        AUTOS(String auto, boolean choreo){
            this.auto = auto;
            this.choreo = choreo;
        }

        public Command getAuto(){
          return choreo ? getChoreoAuto() : getPathPlannerAuto();
        }

        public Command getPathPlannerAuto()
        {
          return new PathPlannerAuto(this.auto);
        }

        public Command getChoreoAuto()
        {
          try {
            var path = PathPlannerPath.fromChoreoTrajectory(this.auto);
            return AutoBuilder.followPath(path);
          } catch (FileVersionException | IOException | ParseException e) {
            DriverStation.reportError("Could not load Choreo Auto", e.getStackTrace()); 
            return null;
          }
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
