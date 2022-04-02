package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class AutoShoot extends CommandBase{
    private final Shooter sub_shooter;

    public AutoShoot(Shooter sub_shooter){
        this.sub_shooter = sub_shooter;

        addRequirements(sub_shooter);
    }
    @Override
    public void initialize(){
        sub_shooter.turnOffTracking();
        sub_shooter.turnOnTracking();
        SmartDashboard.putBoolean("Shooter Running", true);
    }
    @Override
    public void execute(){
      
        sub_shooter.autoShooterSpeed();
        
    }
    @Override
    public void end(boolean interrupted){
        sub_shooter.turnOffTracking();
        sub_shooter.setShooterSpeed(0);
        SmartDashboard.putBoolean("Shooter Running", false);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
