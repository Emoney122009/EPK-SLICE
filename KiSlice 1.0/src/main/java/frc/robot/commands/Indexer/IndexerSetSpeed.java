package frc.robot.commands.Indexer;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Indexer;

public class IndexerSetSpeed extends CommandBase{
    private final Indexer sub_indexer;
    private final double speed;

    public IndexerSetSpeed(Indexer sub_indexer ,double speed ){
        this.sub_indexer = sub_indexer;
        this.speed = speed;
        addRequirements(sub_indexer);
    }
    @Override
    public void initialize(){
        SmartDashboard.putBoolean("Indexer Running", true);
    }
    @Override
    public void execute(){
        sub_indexer.setIndexerSpeed(speed);
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
