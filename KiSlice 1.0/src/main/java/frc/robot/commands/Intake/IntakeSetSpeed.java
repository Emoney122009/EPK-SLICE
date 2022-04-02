package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Intake;

public class IntakeSetSpeed extends CommandBase{
    private final Intake sub_intake;
    private final double speed;

    /**
     * 
     * @param sub_intake
     * @param speed 0 - 1 
     */
    public IntakeSetSpeed(Intake sub_intake ,double speed ){
        this.sub_intake = sub_intake;
        this.speed = speed;
        addRequirements(sub_intake);
    }
    @Override
    public void initialize(){
        SmartDashboard.putBoolean("Intake Running", true);
    }
    @Override
    public void execute(){
     
        sub_intake.setIntakeSpeed(speed);
    }
    @Override
    public void end(boolean interrupted){
        sub_intake.setIntakeSpeed(0);
        SmartDashboard.putBoolean("Intake Running", false);
    }
    @Override
    public boolean isFinished(){
            return false;
    }
}
