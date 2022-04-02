package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class SetClimberHeight extends CommandBase{
    private final Climber sub_climber;
    private final double height;

    public SetClimberHeight(Climber sub_climber , double height){
        this.sub_climber = sub_climber;
        this.height = height;

        addRequirements(sub_climber);
    }
    @Override
    public void initialize(){
        SmartDashboard.putBoolean("Climber Running", true);
    }
    @Override
    public void execute(){
        sub_climber.setClimberPosition(height);
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
