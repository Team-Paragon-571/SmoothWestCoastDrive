package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class DriveCommand extends CommandBase {
    private DoubleSupplier speed;
    private DoubleSupplier turn;
    private DrivetrainSubsystem drivetrainSubsystem;
    private boolean squareInputs = false;

    private double speedScale = 1;
    double positiveSpeedRateLimit = 5.0;
    double negativeSpeedRateLimit = -5.0;
    private SlewRateLimiter speedLimiter = new SlewRateLimiter(positiveSpeedRateLimit, negativeSpeedRateLimit, 0);

    private double turnScale = 0.5;
    double positiveTurnRateLimit = 10.0;
    double negativeTurnRateLimit = -10.0;
    private SlewRateLimiter turnLimiter = new SlewRateLimiter(positiveTurnRateLimit, negativeTurnRateLimit, 0);

    public DriveCommand(DoubleSupplier speed, DoubleSupplier turn, DrivetrainSubsystem drivetrainSubsystem) {
        this.speed = speed;
        this.turn = turn;
        this.drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void initialize() {
        drivetrainSubsystem.stop();
        SmartDashboard.putNumber("Speed scale", speedScale);
        SmartDashboard.putNumber("Turn scale", turnScale);
        SmartDashboard.putBoolean("Square inputs", squareInputs);
    }

    @Override
    public void execute() {
        speedScale = SmartDashboard.getNumber("Speed scale", speedScale);
        turnScale = SmartDashboard.getNumber("Turn scale", turnScale);
        squareInputs = SmartDashboard.getBoolean("Square inputs", squareInputs);

        double driveSpeed = speed.getAsDouble() * speedScale * (squareInputs ? Math.abs(speed.getAsDouble()) : 1);
        double turnSpeed = turn.getAsDouble() * turnScale * (squareInputs ? Math.abs(turn.getAsDouble()) : 1);

        // drivetrainSubsystem.drive(driveSpeed, turnSpeed);
        drivetrainSubsystem.drive(speedLimiter.calculate(driveSpeed), turnLimiter.calculate(turnSpeed));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
