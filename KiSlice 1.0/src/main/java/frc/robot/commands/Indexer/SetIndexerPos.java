package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class SetIndexerPos extends CommandBase{
    private final Indexer sub_indexer;
    private final double pos;

    public SetIndexerPos(Indexer sub_indexer , double pos){
        this.sub_indexer = sub_indexer;
        this.pos = pos;

        addRequirements(sub_indexer);
    }
    @Override
    public void initialize(){
        SmartDashboard.putBoolean("Indexer Running", true);
    }
    @Override
    public void execute(){
        sub_indexer.setIndexerEncoderPosition(pos);
    }
    @Override
    public void end(boolean interrupted){
        sub_indexer.setIndexerSpeed(0);
        SmartDashboard.putBoolean("Indexer Running", false);
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}
