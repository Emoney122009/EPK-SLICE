package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class movePneumatics extends CommandBase{
    private final Climber sub_climber;

    public movePneumatics(Climber sub_climber){
        this.sub_climber = sub_climber;

        addRequirements(sub_climber);
    }
    @Override
    public void initialize(){
       sub_climber.moveArms();
    }
    @Override
    public void execute(){

    }
    @Override
    public void end(boolean interrupted){
       
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
