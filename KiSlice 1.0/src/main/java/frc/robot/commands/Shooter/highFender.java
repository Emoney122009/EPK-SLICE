package frc.robot.commands.Shooter;
import frc.robot.Constants.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class highFender extends CommandBase{
    private final Shooter sub_shooter;

    public highFender(Shooter sub_shooter){
        this.sub_shooter = sub_shooter;

        addRequirements(sub_shooter);
    }
    @Override
    public void initialize(){

        SmartDashboard.putBoolean("Shooter Running", true);
    }
    @Override
    public void execute(){
      
        sub_shooter.setShooterVelocity(c.sh.topHighFender, c.sh.botHighFender);
        
    }
    @Override
    public void end(boolean interrupted){
        sub_shooter.setShooterSpeed(0);
        SmartDashboard.putBoolean("Shooter Running", false);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
