package frc.robot.subsystems;

/*REV ROBOTICS IMPORTS*/
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.*;

/*GYRO IMPORTS*/
import com.kauailabs.navx.frc.*;
import edu.wpi.first.wpilibj.SerialPort;

/*COMMAND BASE IMPORTS*/
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*INTERNAL IMPORTS*/
import frc.robot.Constants.*;
import frc.robot.Constants.id;

/*DIFFERENTIAL DRIVE IMPORTS*/
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/*SMARTDASHBOARD IMPORTS*/
import edu.wpi.first.wpilibj.smartdashboard.*;


public class DriveTrain extends SubsystemBase {
    /* Left Drive Train Motors */
    public final CANSparkMax m_flDrive = new CANSparkMax(id.flDrive, MotorType.kBrushless);
    public final CANSparkMax m_blDrive = new CANSparkMax(id.blDrive, MotorType.kBrushless);
    public final RelativeEncoder e_flDrive = m_flDrive.getEncoder();
    //public final RelativeEncoder e_blDrive = m_blDrive.getEncoder(); // Since The Front Motor is leading the back motor there is no need to get the same values twice.
    public final SparkMaxPIDController p_flDrive = m_flDrive.getPIDController();

    /* Right Drive Train Motors */
    public final CANSparkMax m_frDrive = new CANSparkMax(id.frDrive, MotorType.kBrushless);
    public final CANSparkMax m_brDrive = new CANSparkMax(id.brDrive, MotorType.kBrushless);
    public final RelativeEncoder e_frDrive = m_frDrive.getEncoder();
    //public final RelativeEncoder e_brDrive = m_brDrive.getEncoder(); // Since The Front Motor is leading the back motor there is no need to get the same values twice.
    public final SparkMaxPIDController p_frDrive = m_frDrive.getPIDController();

    /*DRIVETRAIN KINEMATICS */
    public final DifferentialDrive mc_DifferentialDrive = new DifferentialDrive(m_flDrive , m_frDrive);
    public final double
     kP = c.dt.kP ,
     kI =  c.dt.kI,
     kD = c.dt.kD,
     kFF = 0 ,
     kiZ = 0 ; 

    /*GYRO*/
    public final AHRS g_Ahrs = new AHRS(SerialPort.Port.kUSB);

    public DriveTrain() {
        m_blDrive.restoreFactoryDefaults();
        m_brDrive.restoreFactoryDefaults();
        m_flDrive.restoreFactoryDefaults();
        m_frDrive.restoreFactoryDefaults();
        /**
         * Initiates Motors
         */
        m_frDrive.setInverted(true);
        m_flDrive.setInverted(false);
        m_blDrive.follow(m_flDrive);
        m_brDrive.follow(m_frDrive);
        
        g_Ahrs.calibrate();
        

        /**
         * Initiates PID
         */
        setPID(kP, kI, kD, kFF, kiZ);
    }

    public void setPID(double P, double I, double D, double FF, double iZ){
        p_flDrive.setP(P);
        p_frDrive.setP(P);
        p_flDrive.setI(I);
        p_frDrive.setI(I);
        p_flDrive.setD(D);
        p_frDrive.setD(D);
        p_flDrive.setFF(FF);
        p_frDrive.setFF(FF);
        p_flDrive.setIZone(iZ);
        p_frDrive.setIZone(iZ);
        p_flDrive.setOutputRange(-1, 1);
        p_frDrive.setOutputRange(-1, 1);
    }

    public double getlDriveTrainVelocity(){
        return e_flDrive.getVelocity();
    }
    public double getrDriveTrainVelocity(){
        return e_frDrive.getVelocity();
    }
    public double getlDriveTrainPOS(){
        return e_flDrive.getPosition();
    }
    public double getrDriveTrainPOS(){
        return e_frDrive.getPosition();
    }
    public double getAngle(){
        return g_Ahrs.getAngle();
    }
    public boolean gyroRotating(){
        return g_Ahrs.isRotating();  
    }
    public void resetAngle(){
        g_Ahrs.reset();
    }

    /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from 180 to 180
   */
  public double getHeading() {
    return Math.IEEEremainder(g_Ahrs.getAngle(), 360) * (c.kGyroReversed ? -1.0 : 1.0);
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return g_Ahrs.getRate() * (c.kGyroReversed ? -1.0 : 1.0);
  }

    /**
     * Sets the power of the left drive train.
     * 
     * @param speed -1 - 1
     */
    public void setlDriveTrainSpeed(double speed){
        m_flDrive.set(speed);
    }
    /**
     * Sets the power of the right drive train.
     * 
     * @param speed -1 - 1
     */
    public void setrDriveTrainSpeed(double speed){
        m_frDrive.set(speed);
    }
    /**
     * Sets the drive trains speed for both drive sides
     * @param speed -1 - 1 
     */
    public void setDriveTrainSpeed(double speed){
        m_frDrive.set(speed);
        m_flDrive.set(speed);
    }
// honestly dont know why i need this, figure out later.
    public double distanceToEncoderSteps(double dist){
        return (dist / c.wDiameter ) * c.dtGearRatio;
    }


    /**
     * Used to accuratley place the drive train at a curtain position.
     * @param position
     */
    public void setlDriveTrainEncoderPosition(double position){
        p_flDrive.setReference(position, ControlType.kPosition);
        SmartDashboard.putNumber("LDT SET POS", position);
    }

    public void setrDriveTrainEncoderPosition(double position){
        p_frDrive.setReference(position, ControlType.kPosition);
        SmartDashboard.putNumber("RDT SET POS", position); //RIGHT DRIVE TRAIN SET POSITION
    }



    /**
     * Always Make sure that the default is having the fr inverted
     * when calling this functino make sure to have a press and release function in order to make it act like a while pressed button. So you would need to say when pressed activated and when release activated. 
     */
    public void setDriveTrainReverse(){
        if(m_frDrive.getInverted() == true){
            m_frDrive.setInverted(false);
            m_flDrive.setInverted(true);
            SmartDashboard.putBoolean("DT inverted", true);
        } else if(m_frDrive.getInverted() == false);
            m_frDrive.setInverted(true);
            m_flDrive.setInverted(false);
            SmartDashboard.putBoolean("DT inverted", false);
    }
    
    public void driveTrainMotorCoastMode(){
        m_flDrive.setIdleMode(IdleMode.kCoast);
        m_blDrive.setIdleMode(IdleMode.kCoast);
        m_frDrive.setIdleMode(IdleMode.kCoast);
        m_brDrive.setIdleMode(IdleMode.kCoast);
        SmartDashboard.putString("DT mode", "coast");
    }

    public void driveTrainMotorBreakMode(){
        m_flDrive.setIdleMode(IdleMode.kBrake);
        m_blDrive.setIdleMode(IdleMode.kBrake);
        m_frDrive.setIdleMode(IdleMode.kBrake);
        m_brDrive.setIdleMode(IdleMode.kBrake);
        SmartDashboard.putString("DT mode", "break");
    }

    public void driveTrainMotorModeSwitch(){
        if(m_flDrive.getIdleMode() == IdleMode.kBrake){
            driveTrainMotorCoastMode();
        } else if(m_flDrive.getIdleMode() == IdleMode.kCoast){
            driveTrainMotorBreakMode();
        }

    }

    /**
     * Emmulates acarde drive in the drivetrain sub system.
     *
     * @param xSpeed -1 - 1 
     * @param zRotation -1 - 1
     * @param xMod -1 - 1
     * @param yMod -1 - 1
     */
    public void drive(double xSpeed, double zRotation , double xMod , double yMod){
        mc_DifferentialDrive.arcadeDrive(xSpeed * xMod, zRotation *yMod);
        SmartDashboard.putNumber("DT SetSpeed", xSpeed*xMod);
        SmartDashboard.putNumber("DT SetRotation", zRotation*yMod);

    }


    public void periodic(){
        SmartDashboard.putNumber("Left DriveTrain Velocity", e_flDrive.getVelocity());
        SmartDashboard.putNumber("Right DriveTrain Velocity", e_frDrive.getVelocity());
        SmartDashboard.putNumber("DriveTrain angle", g_Ahrs.getAngle());
        SmartDashboard.putNumber("DriveTrain Velocity x ", g_Ahrs.getVelocityX());
        SmartDashboard.putNumber("DriveTrain Velocity y ", g_Ahrs.getVelocityY());
        SmartDashboard.putNumber("DriveTrain Velocity z ", g_Ahrs.getVelocityZ());
        SmartDashboard.putNumber("Left DriveTrain Position", e_flDrive.getPosition());
        SmartDashboard.putNumber("Right DriveTrain Position", e_frDrive.getPosition());
    }
}



