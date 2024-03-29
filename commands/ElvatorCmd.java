package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elvator;

public class ElvatorCmd extends Command{
    private Elvator elvator;
    private double rightSpeed, leftSpeed;

    public ElvatorCmd(Elvator elvator, double RightSpeed, double LeftSpeed){
        this.elvator = elvator;
        this.rightSpeed = RightSpeed;
        this.leftSpeed = LeftSpeed;

        addRequirements(elvator);
    }

    //初始化(進到Command會跑一次)
    @Override
    public void initialize(){
        System.out.println("ShooterCmd Started!");
    }

    //循環(進到Command後循環)
    @Override
    public void execute(){
        elvator.setElvatorL(this.leftSpeed);
        elvator.setElvatorR(this.rightSpeed);
    }

    @Override
    public void end(boolean interrupted){
        elvator.stopMotor();
        System.out.println("ShooterCmd finished!");
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
