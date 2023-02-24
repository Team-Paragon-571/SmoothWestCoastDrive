package frc.robot.Subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DrivetrainSubsystem extends SubsystemBase {
    WPI_TalonFX leftMaster;
    WPI_TalonFX leftFollower;
    WPI_TalonFX rightMaster;
    WPI_TalonFX rightFollower;

    DifferentialDrive drive;

    public DrivetrainSubsystem() {
        leftMaster = initMotor(1);
        leftMaster.setInverted(TalonFXInvertType.Clockwise);
        leftFollower = initMotor(2);
        leftFollower.follow(leftMaster);
        leftFollower.setInverted(TalonFXInvertType.FollowMaster);

        rightMaster = initMotor(3);
        rightMaster.setInverted(TalonFXInvertType.CounterClockwise);
        rightFollower = initMotor(4);
        rightFollower.follow(rightMaster);
        rightFollower.setInverted(TalonFXInvertType.FollowMaster);


        drive = new DifferentialDrive(leftMaster, rightMaster);
    }

    private static WPI_TalonFX initMotor(int canId) {
        WPI_TalonFX motor = new WPI_TalonFX(canId);
        motor.configFactoryDefault();
        return motor;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left master sensor position", leftMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("Left master sensor velocity", leftMaster.getSelectedSensorVelocity());
        SmartDashboard.putNumber("Right master sensor position", rightMaster.getSelectedSensorPosition());
        SmartDashboard.putNumber("Right master sensor velocity", rightMaster.getSelectedSensorVelocity());
    }

    public void drive(double speed, double turn) {
        drive.curvatureDrive(speed, turn, true);
    }

    public void setNeutralMode(NeutralMode neutralMode) {
        leftMaster.setNeutralMode(neutralMode);
        leftFollower.setNeutralMode(neutralMode);
        rightMaster.setNeutralMode(neutralMode);
        rightFollower.setNeutralMode(neutralMode);
    }

    public void stop() {
        leftMaster.set(0);
        rightMaster.set(0);
    }

    
}
