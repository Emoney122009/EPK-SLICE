package frc.robot.subsystems;

/*REV ROBOTICS IMPORTS*/
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.*;

/*GAME PIECE IMPORTS*/
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.I2C;

/*COMMAND BASE IMPORTS*/
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*INTERNAL IMPORTS*/
import frc.robot.Constants.*;
import frc.robot.Constants.id;

/*SMARTDASHBOARD IMPORTS*/
import edu.wpi.first.wpilibj.smartdashboard.*;


public class Indexer extends SubsystemBase {
    /* Left Indexer Motor */
    public final CANSparkMax m_indexer = new CANSparkMax(id.lIndexer , MotorType.kBrushless);
    public final RelativeEncoder e_indexer = m_indexer.getEncoder();
    public final SparkMaxPIDController p_indexer = m_indexer.getPIDController();

    public final double kP = 0.01 ,kI = 0 ,kD = 0 ,kFF = 0 ,kiZ = 0 ; 

    /* game pieces */
    public final ColorSensorV3 s_color = new ColorSensorV3(I2C.Port.kOnboard);
    public final ColorMatch s_match = new ColorMatch();
    public final Color kBlueTarget = new Color(0.143, 0.427, 0.429);
    public final Color kRedTarget = new Color(0.561, 0.232, 0.114);
    public int ballCount;
    public ColorMatchResult s_cMatch;
    public Color s_c;
    public String ballColor;

    public Indexer() {
        m_indexer.restoreFactoryDefaults();
        s_match.addColorMatch(kBlueTarget);
        s_match.addColorMatch(kRedTarget);
        ballCount = 1;// loaded with one ball
        /**
         * Initiates PID
         */
        setPID(kP, kI, kD, kFF, kiZ);
    }

    public void setPID(double P, double I, double D, double FF, double iZ){
        p_indexer.setP(P);
  
        p_indexer.setI(I);
    
        p_indexer.setD(D);
      
        p_indexer.setFF(FF);
  
        p_indexer.setIZone(iZ);

        p_indexer.setOutputRange(-1, 1);
  
    }

    public double getIndexerVelocity(){
        return e_indexer.getVelocity();
    }

    public double getIndexerPOS(){
        return e_indexer.getPosition();
    }

    /**
     * Sets the power of the left drive train.
     * 
     * @param speed -1 - 1
     */
    public void setIndexerSpeed(double speed){
        m_indexer.set(speed);
    }

    
// honestly dont know why i need this, figure out later.
    public double distanceToEncoderSteps(double dist){
        return (dist / c.wDiameter ) * c.dtGearRatio;
    }
    /**
     * Used to accuratley place the drive train at a curtain position.
     * @param position
     */
    public void setIndexerEncoderPosition(double position){

        p_indexer.setReference(position, ControlType.kPosition);
        SmartDashboard.putNumber("Indexer SET POS", position);
        SmartDashboard.putNumber("Indexer POS", e_indexer.getPosition());
    }

    public Color detectBallColor(){
        return s_cMatch.color;
    }

    public int getBallCount(){
        return ballCount;
    }

    public void inBall(){
        ballCount++;
        if(ballCount > 2){
            System.out.println("BALL COUNTER ERROR ++");
        }
    }
    public void outBall(){
        ballCount--;
        if(ballCount < 0){
            System.out.println("BALL COUNTER ERROR --");
        }
    }

    public void periodic(){
        s_c = s_color.getColor();
        s_cMatch = s_match.matchClosestColor(s_c);
        
        if (s_cMatch.color == kBlueTarget) {
            ballColor = "Blue";
          } else if (s_cMatch.color == kRedTarget) {
            ballColor = "Red";
          } else {
            ballColor = "Unknown";
          }
          SmartDashboard.putNumber("Red", s_c.red);
          SmartDashboard.putNumber("Green", s_c.green);
          SmartDashboard.putNumber("Blue", s_c.blue);
          SmartDashboard.putNumber("Confidence", s_cMatch.confidence);
          SmartDashboard.putString("Detected Color", ballColor);
          SmartDashboard.putNumber("Proximity", s_color.getProximity());
    }
}



