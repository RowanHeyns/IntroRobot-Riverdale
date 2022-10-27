package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DriveSubsystem;

/** An example command that uses an example subsystem. */
public class GyroTurn extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final DriveSubsystem m_subsystem;
  double targetAngle = 90;
  double targetAntiAngle = 180;
    // private long lD = 10400;
    //     private long sD = 3900;
    //     private long ta = 1500;
    //     //private long tas = 100;
    // private long timerStart = 0;
   // private long firstDuration = 5000;
   // private long timerEnd = 0;
   // private long secondDuration = 5000;
    // private boolean isdone = false;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public GyroTurn(DriveSubsystem subsystem) {
    m_subsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
    }
    @Override
    public void initialize(){
//cheese
      m_subsystem.zeroHeading();
    }
    @Override
    public void execute(){
      m_subsystem.arcadeDrive(0, 0.5);
    }
    public boolean isFinished(){
      System.out.println(m_subsystem.getHeading());
        //is current system time 5000 or more millis more than starttime
        return m_subsystem.getHeading() >= targetAngle;
      }
}