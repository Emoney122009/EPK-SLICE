/**
 * Code is basically finished in this section...
 * Things to do:
 *  Tune PID
 *  Calculate exact encoder to meters
 *  make things more abstract
 
 * 
 */





package frc.robot.subsystems;

/*REV ROBOTICS IMPORTS*/
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.*;

/*COMMAND BASE IMPORTS*/
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*PNEUMARIC IMPORTS*/
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/*INTERNAL IMPORTS*/
import frc.robot.Constants.*;
import frc.robot.Constants.id;

/*SMARTDASHBOARD IMPORTS*/
import edu.wpi.first.wpilibj.smartdashboard.*;


public class Climber extends SubsystemBase {
    /* Left Climber Motor */
    public final CANSparkMax m_lClimber = new CANSparkMax(id.lClimber , MotorType.kBrushless);
    public final RelativeEncoder e_lClimber = m_lClimber.getEncoder();
    public final SparkMaxPIDController p_lClimber = m_lClimber.getPIDController();

    /* Right Drive Train Motors */
    public final CANSparkMax m_rClimber = new CANSparkMax(id.rClimber , MotorType.kBrushless);
    public final RelativeEncoder e_rClimber = m_rClimber.getEncoder();
    public final SparkMaxPIDController p_rClimber = m_rClimber.getPIDController();

    /* PID Values*/
    public final double kP = 0.05 ,kI = 0 ,kD = 0 ,kFF = 0 ,kiZ = 0 ; 
    public double climberPositionDiff;


    /*Pneumatics*/
    public final Compressor pn_compressor = new Compressor(PneumaticsModuleType.CTREPCM);
    boolean pn_enable = pn_compressor.enabled();
    boolean pn_pressureSwitch = pn_compressor.getPressureSwitchValue();
    double pn_current = pn_compressor.getCurrent();
    public final DoubleSolenoid pn_doubleSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 1);
    boolean extended = false;
    double lowMaxExtension = c.lowMaxClimbHeight;

    public Climber() {
        m_lClimber.restoreFactoryDefaults();
        m_rClimber.restoreFactoryDefaults();
        /**
         * Initiates Motors
         */
        m_rClimber.setInverted(true);
        
        /**
         * Initiates PID
         */
        setPID(kP, kI, kD, kFF, kiZ);
    }

    public void setPID(double P, double I, double D, double FF, double iZ){
        p_lClimber.setP(P);
        p_rClimber.setP(P);
        p_lClimber.setI(I);
        p_rClimber.setI(I);
        p_lClimber.setD(D);
        p_rClimber.setD(D);
        p_lClimber.setFF(FF);
        p_rClimber.setFF(FF);
        p_lClimber.setIZone(iZ);
        p_rClimber.setIZone(iZ);
        p_lClimber.setOutputRange(-1, 1);
        p_rClimber.setOutputRange(-1, 1);
    }

    /**
     * gets the Left Climber velocity
     * @return double velocity RPM
     */
    public double getlClimberVelocity(){
        return e_lClimber.getVelocity();
    }

    /**
     * gets the Right Climber velocity
     * @return double velocity RPM
     */
    public double getrClimberVelocity(){
        return e_rClimber.getVelocity();
    }

    /**
     * gets the Left climber encoder position.
     * @return double encoder position
     */
    public double getlClimberPOS(){
        return e_lClimber.getPosition();
    }

    /**
     * gets the right climber encoder position.
     * @return double encoder position
     */
    public double getrClimberPOS(){
        return e_rClimber.getPosition();
    }

    /**
     * Sets the power of the left drive train.
     * 
     * @param speed -1 - 1
     */
    public void setlClimberSpeed(double speed){
        m_lClimber.set(speed);
        SmartDashboard.putNumber("LC Set Speed", speed);
    }



    /**
     * Sets the power of the right drive train.
     * 
     * @param speed -1 - 1
     */
    public void setrClimberSpeed(double speed){
        m_rClimber.set(speed);
        SmartDashboard.putNumber("RC Set Speed", speed);
    }
    /**
     * Sets the drive trains speed for both drive sides
     * @param speed -1 - 1 
     */
    public void setClimberSpeed(double speed){
        m_rClimber.set(speed);
        m_lClimber.set(speed);
        SmartDashboard.putNumber("Climber Set Speed", speed);
    }


// honestly dont know why i need this, figure out later.
    public double distanceToEncoderSteps(double dist){
        return (dist / c.wDiameter * Math.PI) * c.dtGearRatio;
        
    }

    /**
     * sets the left climber position in encoder steps
     * @param position encoder steps
     */
    public void setlClimberPosition(double position){
        p_lClimber.setReference(position, ControlType.kPosition);
        SmartDashboard.putNumber("LC SET POS", position);
    }

    /**
     * sets the right climber position in encoder steps
     * @param position encoder steps
     */
    public void setrClimberPosition(double position){
        p_rClimber.setReference(position, ControlType.kPosition);
        SmartDashboard.putNumber("RC SET POS", position);
    }

    /**
     * move the both climbers to a position
     * @param position
     */
    public void setClimberPosition(double position){
        setrClimberPosition(position);
        setlClimberPosition(position);
    }





    /**
     * WIP, need to figure out how to set encoder to zero without moving the climbers.
     */
    public void resetClimberEncoderPosition(){

        //needs work, want to set encode read out to zero.
    }

    /**
     * Retracts the climber arms
     */
    public void retractArms(){
            pn_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);
            extended = false;
            SmartDashboard.putBoolean("Extended", extended);
    }

    /**
     * Extends the climber arms
     */
    public void extendArms(){
            pn_doubleSolenoid.set(DoubleSolenoid.Value.kForward);
            extended = true;
            SmartDashboard.putBoolean("Extended", extended);
    }

    /**
     * Move the climbers in the opposite direction they are currently in.
     */
    public void moveArms(){
            if(extended == true ){
                retractArms();
            } else if(extended == false){
                extendArms();
            }
    }

    /**
     * puts the climber motors in coast mode
     */
    public void climberMotorCoastMode(){
        m_lClimber.setIdleMode(IdleMode.kCoast);
        m_rClimber.setIdleMode(IdleMode.kCoast);
        SmartDashboard.putString("Climber Motor Mode", "Coast");
    }

    /**
     * puts the climber motors in break mode.
     */
    public void climberMotorBreakMode(){
 
        m_lClimber.setIdleMode(IdleMode.kBrake);
        m_rClimber.setIdleMode(IdleMode.kBrake);
        SmartDashboard.putString("Climber Motor Mode", "Break");
    }

    /**
     * Puts the climber motor in the opposite mode they are currently in
     */
    public void climberMotorModeSwitch(){
        if(m_lClimber.getIdleMode() == IdleMode.kBrake){
            climberMotorCoastMode();
        } else if(m_lClimber.getIdleMode() == IdleMode.kCoast){
            climberMotorBreakMode();
        }
    }

    /**
     * Updates SmartDashboard values that can change without function manipulation.
     */
    public void periodic(){
        climberPositionDiff = e_lClimber.getPosition()-e_rClimber.getPosition();
        SmartDashboard.putNumber("Climber Position Difference", climberPositionDiff);
        SmartDashboard.putNumber("LC POS", e_lClimber.getPosition());
        SmartDashboard.putNumber("RC POS", e_rClimber.getPosition());
        SmartDashboard.putNumber("LC Speed", e_lClimber.getVelocity());
        SmartDashboard.putNumber("RC Speed", e_rClimber.getVelocity());
    }
}



