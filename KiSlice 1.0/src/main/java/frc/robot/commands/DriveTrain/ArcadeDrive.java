package frc.robot.commands.DriveTrain;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase{
    private final DriveTrain sub_drive;
    private final Supplier<Double> xSpeed , zRotation;
    private final double xMod, yMod;

    public ArcadeDrive(DriveTrain sub_drive ,Supplier<Double> xSpeed, Supplier<Double> zRotation , double xMod , double yMod ){
        this.sub_drive = sub_drive;
        this.xSpeed = xSpeed;
        this.zRotation = zRotation;
        this.xMod = xMod;
        this.yMod = yMod;
        addRequirements(sub_drive);
    }
    @Override
    public void initialize(){
        SmartDashboard.putBoolean("DriveTrain Running", true);
    }
    @Override
    public void execute(){
        sub_drive.drive(xSpeed.get(), zRotation.get(), xMod, yMod);
    }
    @Override
    public void end(boolean interrupted){
        sub_drive.setDriveTrainSpeed(0);
        SmartDashboard.putBoolean("DriveTrain Running", false);
    }
    @Override
    public boolean isFinished(){
            return false;
    }
}
