package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.SparkPIDController;
import com.revrobotics.RelativeEncoder;

public class Intake extends SubsystemBase{
    //吸入
    private CANSparkMax IntakeMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeMotorPort, MotorType.kBrushless);
    //角度(上下)
    private CANSparkMax IntakeCtrlMotor = new CANSparkMax(Constants.IntakeConstants.kIntakeCtrlMotorPort, MotorType.kBrushless);
    private SparkPIDController intakeController;
    private RelativeEncoder intakeEncoder;
    private double kP, kI, kD, kFF, kMaxOutput, kMinOutput;
    private double setPoint;

    public Intake(){
        IntakeMotor.restoreFactoryDefaults();
        IntakeCtrlMotor.restoreFactoryDefaults();
        IntakeCtrlMotor.setIdleMode(IdleMode.kBrake);

        intakeEncoder = IntakeCtrlMotor.getEncoder();
        intakeController = IntakeCtrlMotor.getPIDController();

        kP = 0.05;
        kI = 0.2;
        kD = 0.04;
        kFF = 0.0;
        kMaxOutput = 0.5;
        kMinOutput = -0.5;

        intakeController.setP(kP);
        intakeController.setI(kI);
        intakeController.setD(kD);
        intakeController.setFF(kFF);
        intakeController.setOutputRange(kMinOutput, kMaxOutput);

        IntakeMotor.burnFlash();
        IntakeCtrlMotor.burnFlash();
        SmartDashboard.putNumber("Setpoint", setPoint);
    }

    @Override
    public void periodic() {
        double s = SmartDashboard.getNumber("Setpoint", 0);
        intakeController.setOutputRange(-0.5, 0.5);
        intakeController.setP(0.07);
        intakeController.setI(0.0);
        intakeController.setD(0.03);


        setIntakPosition(s);
        SmartDashboard.putNumber("Angle", intakeEncoder.getPosition());
        SmartDashboard.putNumber("Setpoint:", s);

    }   
    
    

    // public Command IntakeUp(){
    //     return startEnd(
    //         () -> intakeController.setReference(0, ControlType.kPosition), 
    //         () -> intakeController.setReference(0, ControlType.kPosition)
    //     );
    // }

    // public Command IntakeDown(){
    //     return startEnd(
    //         () -> intakeController.setReference(30, ControlType.kPosition),
    //         () -> intakeController.setReference(30, ControlType.kPosition)
    //     );
    // }

    public double getP(){
        return intakeController.getP();
    }

    public double getI(){
        return intakeController.getI();
    }

    public double getD(){
        return intakeController.getD();
    }

    public void setIntakPosition(double setPoint){
        intakeController.setReference(setPoint, ControlType.kPosition);
    }

    public void setP(double kP) {
        intakeController.setP(kP);
    }

}