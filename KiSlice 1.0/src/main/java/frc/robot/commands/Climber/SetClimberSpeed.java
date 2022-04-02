package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class SetClimberSpeed extends CommandBase{
    private final Climber sub_climber;
    private final double speed;

    public SetClimberSpeed(Climber sub_climber , double speed){
        this.sub_climber = sub_climber;
        this.speed = speed;

        addRequirements(sub_climber);
    }
    @Override
    public void initialize(){
        SmartDashboard.putBoolean("Climber Running", true);
    }
    @Override
    public void execute(){
        sub_climber.setClimberSpeed(speed);
    }
    @Override
    public void end(boolean interrupted){
        sub_climber.setClimberSpeed(0);
        SmartDashboard.putBoolean("Climber Running", false);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
