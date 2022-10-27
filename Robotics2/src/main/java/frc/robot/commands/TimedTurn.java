package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class TimedTurn extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
    private long lD = 10400;
        private long sD = 3900;
        private long ta = 1500;
        //private long tas = 100;
    private long timerStart = 0;
   // private long firstDuration = 5000;
   // private long timerEnd = 0;
   // private long secondDuration = 5000;
    private boolean isdone = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public TimedTurn(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    }
    @Override
    public void initialize(){
        m_subsystem.arcadeDrive(0.5,0);
        timerStart = System.currentTimeMillis();
    }
    public boolean isFinished(){
        //is current system time 5000 or more millis more than starttime
        return isdone;
    
    }
    @Override
    public void execute(){
      long te = System.currentTimeMillis()- timerStart;
      if(te <= lD){
        //Subtracts value from every number add on to it example it goes 1000 so if the next command is 2000 it takes 1000 from it the next command after that it will subtract 3000 so go higher then that
        m_subsystem.arcadeDrive(0.5,0);
        //Moves forward
      }
      

      else if(te <= ta + lD){
        m_subsystem.arcadeDrive(0,0.5);
        //Turns angle
      }
      // else if(te <= tas){
      //   m_subsystem.arcadeDrive(0,0.5);
      //   //Turns angle
      // }
      
      else if(te <= sD){
        m_subsystem.arcadeDrive(0.5,0);
        //Moves staight short
      }

      else if(te <= ta +lD){
        m_subsystem.arcadeDrive(0,0.5);
        //Turns angle
      }

      // else if(te <= tas){
      //   m_subsystem.arcadeDrive(0,0.5);
      //   //Turns angle
      // }

      else if(te <= lD){
        m_subsystem.arcadeDrive(0.5,0);
        //Moves staight
      }

      else if(te <= ta + lD){
        m_subsystem.arcadeDrive(0,0.5);
        //Turns angle
      }

      // else if(te <= tas){
      //   m_subsystem.arcadeDrive(0,0.5);
      //   //Turns angle
      // }

      else if(te <= sD){
        m_subsystem.arcadeDrive(0.5,0);
        //Moves staight short
      }

      else if(te <= ta + lD){
        m_subsystem.arcadeDrive(0,0.5);
        //Turns angle
      }

      // else if(te <= tas){
      //   m_subsystem.arcadeDrive(0,0.5);
      //   //Turns angle
      // }

      else{
        isdone = true;
      }

        
      }
    
    public void end(){
        m_subsystem.arcadeDrive(0, 0);
    }
}