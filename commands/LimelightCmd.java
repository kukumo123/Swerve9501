 package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightHelpers;
import frc.robot.subsystems.Shooter;


 public class LimelightCmd extends Command{
     private final Shooter shooter = new Shooter();

     @Override
     public void execute() {
         System.out.println("LimeLight ON");
         if (LimelightHelpers.getFiducialID("") == 1){
             System.out.println(LimelightHelpers.getTA(""));
             shooter.setShooterR(1);
             shooter.setShooterL(-1);
         }else {
             shooter.stopMotor();
         }
     }
 }