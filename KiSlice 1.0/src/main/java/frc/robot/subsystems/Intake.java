package frc.robot.subsystems;

/*PHEONIC IMPORTS*/
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

/*COMMAND BASE IMPORTS*/
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*INTERNAL IMPORTS*/
import frc.robot.Constants.id;

/*SMARTDASHBOARD IMPORTS*/
import edu.wpi.first.wpilibj.smartdashboard.*;


public class Intake extends SubsystemBase {

  public final TalonFX m_intake = new TalonFX(id.rIntake);
  public Intake() {
  
    m_intake.setInverted(true);
  }

  public double getIntakeResistance(){
    return m_intake.getSupplyCurrent() - m_intake.getStatorCurrent();
  }

  public void invertIntake(){
    if(m_intake.getInverted() == true){
      m_intake.setInverted(false);
      SmartDashboard.putBoolean("Intake Inverted" , true);
    } else if(m_intake.getInverted() == false){
      m_intake.setInverted(true);
      SmartDashboard.putBoolean("Intake Inverted", false);
    }

  }
  /**
   * Controlls the Speed of the intake motor.
   * @param speed 0 - 1
   */
  public void setIntakeSpeed(double speed){
    m_intake.set(ControlMode.PercentOutput,speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake Speed", m_intake.getMotorOutputPercent());
    SmartDashboard.putNumber("Intake Pos", m_intake.getSelectedSensorPosition());
  }
}
