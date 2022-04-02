package frc.robot.commands.DriveTrain;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveFoward extends CommandBase{
    private final DriveTrain sub_drive;
    private final double distance;
    private double encoderSetpoint;

    public DriveFoward(DriveTrain sub_drive , double distance){
        this.sub_drive = sub_drive;
        this.distance = distance;

        addRequirements(sub_drive);
    }
    @Override
    public void initialize(){
        encoderSetpoint = sub_drive.getrDriveTrainPOS() + distance;
        SmartDashboard.putBoolean("DriveTrain Running", true);
    }
    @Override
    public void execute(){
        sub_drive.setrDriveTrainEncoderPosition(encoderSetpoint);
        sub_drive.setlDriveTrainEncoderPosition(encoderSetpoint);
    }
    @Override
    public void end(boolean interrupted){
        sub_drive.setDriveTrainSpeed(0);
        SmartDashboard.putBoolean("DriveTrain Running", false);
    }
    @Override
    public boolean isFinished(){
        if (sub_drive.getrDriveTrainPOS() > encoderSetpoint){
            return true;
        } else {
            return false;
        }
    }
}
