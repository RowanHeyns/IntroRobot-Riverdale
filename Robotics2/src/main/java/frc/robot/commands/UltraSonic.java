package frc.robot.commands;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class UltraSonic extends CommandBase{
    public DriveSubsystem m_drive;
    public int distance = 12;
    
    public UltraSonic(DriveSubsystem m_drive, int distance) {
        this.distance = distance;
        this.m_drive = m_drive;
    }

    public void execute() {
        m_drive.arcadeDrive(0.5, 0);
    }
    public boolean isFinished(){
        System.out.println(m_drive.getRange());
        return m_drive.getRange() <= distance;
        
    }
}
