package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elvator extends SubsystemBase{
    private CANSparkMax Left = new CANSparkMax(Constants.ElvatorConstants.kElvatorLeftMotorID, MotorType.kBrushless);
    private CANSparkMax Right = new CANSparkMax(Constants.ElvatorConstants.kElvatorRightMotorID, MotorType.kBrushless);


public Elvator(){
    Left.restoreFactoryDefaults();
    Right.restoreFactoryDefaults();
    Left.setIdleMode(IdleMode.kBrake);
    Right.setIdleMode(IdleMode.kBrake);
    Left.setInverted(true);


}

public void stopMotor(){
    Left.stopMotor();
    Right.stopMotor();

}

public void setElvatorL(double speed){
    Left.set(speed);
}

public void setElvatorR (double speed){
    Right.set(speed);
}
}
