package frc.robot.subsystems;

/*REV ROBOTICS IMPORTS*/
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.*;

/*COMMAND BASE IMPORTS*/
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*INTERNAL IMPORTS*/
import frc.robot.Constants.*;
import frc.robot.Constants.id;

/*LIMELIGHT IMPORTS*/
import edu.wpi.first.networktables.*;

/*SMARTDASHBOARD IMPORTS*/
import edu.wpi.first.wpilibj.smartdashboard.*;


public class Shooter extends SubsystemBase {
    /* Left Shooter Motor */
    public final CANSparkMax m_blShooter = new CANSparkMax(id.blShooter , MotorType.kBrushless);
    public final RelativeEncoder e_blShooter = m_blShooter.getEncoder();
    public final SparkMaxPIDController p_blShooter = m_blShooter.getPIDController();

    /* Right Drive Train Motors */
    public final CANSparkMax m_tShooter = new CANSparkMax(id.tShooter, MotorType.kBrushless);
    public final RelativeEncoder e_tShooter = m_tShooter.getEncoder();
    public final SparkMaxPIDController p_tShooter = m_tShooter.getPIDController();


    public final double kP = 0.00001 ,kI = 0.00000001 ,kD = 0 ,kFF = 0 ,kiZ = 0 ; 



    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    NetworkTableEntry light;
    NetworkTableEntry cam;
    double x;
    double y;
    double area;

    public Shooter() {
        /**
         * Initiates Motors
         */
        m_tShooter.setInverted(true);
        
        /**
         * Initiates PID
         */
        setPID(kP, kI, kD, kFF, kiZ);
    }

    public void setPID(double P, double I, double D, double FF, double iZ){
        p_blShooter.setP(P);
        p_tShooter.setP(P);
        p_blShooter.setI(I);
        p_tShooter.setI(I);
        p_blShooter.setD(D);
        p_tShooter.setD(D);
        p_blShooter.setFF(FF);
        p_tShooter.setFF(FF);
        p_blShooter.setIZone(iZ);
        p_tShooter.setIZone(iZ);
        p_blShooter.setOutputRange(-1, 1);
        p_tShooter.setOutputRange(-1, 1);
    }

    public double getBottomShooterVelocity(){
        return e_blShooter.getVelocity();
    }
    public double getTopShooterVelocity(){
        return e_tShooter.getVelocity();
    }
    /**
     * Sets the power of the left drive train.
     * 
     * @param speed -1 - 1
     */
    public void setBottomShooterSpeed(double speed){
        m_blShooter.set(speed);
    }
    /**
     * Sets the power of the right drive train.
     * 
     * @param speed -1 - 1
     */
    public void setTopShooterSpeed(double speed){
        m_tShooter.set(speed);
    }
    /**
     * Sets the drive trains speed for both drive sides
     * @param speed -1 - 1 
     */
    public void setShooterSpeed(double speed){
        m_tShooter.set(speed);
        m_blShooter.set(speed);
    }

    /**
     * 
     * @param offset 
     * @return 
     */
    public double calcDist(double offset){
        double distance = (int) ((0.0643409*((y-9.80908 + offset)*(y-9.80908 + offset)))+47.266);
        SmartDashboard.putNumber("Shooter Distance", distance);
        return distance; 
    }

    /**
     * NOT IN USE // for hood / turret
     * @param error
     * @return
     */
    public double calcAngle(double error){
        double angle = x *-1 *error; 
        return angle;
    }

    /**
     * Calculates the top shooter speed based on a exponential curve 
     * @return
     */
    public double calcTopShooterSpeed(){
        double speed = (int) ((c.sh.topA * (Math.pow(calcDist(0) + c.sh.topB , 2)) ) + c.sh.topC);//make trapazoidal
        return speed;
    }

    /**
     * Calculates the bot shooter speed base on exponential curve
     * @return
     */
    public double calcBottomShooterSpeed(){
        double speed = (int) ((c.sh.botA * (Math.pow(calcDist(0) + c.sh.botB, 2)))+ c.sh.botC);
        return speed;
    }

    /**
     * automatically sets the reference points for both flywheels.
     */
    public void autoShooterSpeed(){
        p_blShooter.setReference(calcBottomShooterSpeed(), ControlType.kVelocity);
        p_tShooter.setReference(calcTopShooterSpeed(), ControlType.kVelocity); 
        
    }

    /**
     * sets the shooter velocity manually
     * mainly for testing or fender shots
     * @param tspeed
     * @param bspeed
     */
    public void setShooterVelocity(double tspeed, double bspeed){
        p_blShooter.setReference(bspeed, ControlType.kVelocity);
        p_tShooter.setReference(tspeed, ControlType.kVelocity);    
    }
    
    public void turnOnTracking(){
        light.setNumber(3);
        cam.setNumber(0);
    }
    public void turnOffTracking(){
        light.setNumber(1);
        cam.setNumber(1);
    }


public void periodic(){
    table = NetworkTableInstance.getDefault().getTable("limelight-slice");
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");
    ta = table.getEntry("ta");
    light = table.getEntry("ledMode");
    cam = table.getEntry("camMode");
    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    }
}



