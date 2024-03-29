package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final CANSparkMax turningMotor;

    private final RelativeEncoder turningEncoder;

    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final CANcoder FLcancoder = new CANcoder(9, "rio");
    private final CANcoder FRcancoder = new CANcoder(10, "rio");
    private final CANcoder BLcancoder = new CANcoder(11, "rio");
    private final CANcoder BRcancoder = new CANcoder(12, "rio");


    private final double absoluteEncoderOffsetRad;


    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        absoluteEncoder = new CANcoder(absoluteEncoderId);

        driveMotor = new TalonFX(driveMotorId);
        turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        turningEncoder = turningMotor.getEncoder();

        turningEncoder.setPositionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad);
        turningEncoder.setVelocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        turningPidController = new PIDController(ModuleConstants.kPTurning, ModuleConstants.kITurning, ModuleConstants.kDTurning);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
        SmartDashboard.putNumber("FL", FLcancoder.getAbsolutePosition().getValue()*(Math.PI*2));
        SmartDashboard.putNumber("FR", FRcancoder.getAbsolutePosition().getValue()*(Math.PI*2));
        SmartDashboard.putNumber("BL", BLcancoder.getAbsolutePosition().getValue()*(Math.PI*2));
        SmartDashboard.putNumber("BR", BRcancoder.getAbsolutePosition().getValue()*(Math.PI*2));

        

        
    }

    public double getDrivePosition() {
        return driveMotor.getPosition().getValue() * Constants.ModuleConstants.kDriveMotorGearRatio;
    }

    public double getTurningPosition() {
        return turningEncoder.getPosition();
        //return absoluteEncoder.getAbsolutePosition().getValue();
    }

    public double getDriveVelocity() {
        return driveMotor.getVelocity().getValue() * Constants.ModuleConstants.kDriveEncoderRPM2MeterPerSec;
    }

    // public double getTurningVelocity() {
    //     return turningEncoder.getVelocity();
    // }

    public double getAbsoluteEncoderRad() {
        double angle = absoluteEncoder.getAbsolutePosition().getValue();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        SmartDashboard.putNumber("InitialAngle", angle);
        return angle;
    }

    public void resetEncoders() {
        turningEncoder.setPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    Pose3d poseA = new Pose3d();
Pose3d poseB = new Pose3d();

// WPILib
StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose3d.struct).publish();
StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

    void periodic() {
    publisher.set(poseA);
    arrayPublisher.set(new Pose3d[] {poseA, poseB});}

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            driveMotor.getPosition().getValue(),
            new Rotation2d(getTurningPosition())
        );
    }
}