package frc.robot.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Subsystems.DrivetrainSubsystem;

/**
 * Command to drive the robot, using speed and rotation values. Requires the DrivetrainSubsystem.
 */
public class DriveCommand extends CommandBase {
    // Speed values
    private DoubleSupplier speed;
    // Value to scale joystick speed input by. Should probably be less than 1.
    private double speedScale = 1;
    // SlewRateLimiter and values to smooth acceleration
    double positiveSpeedRateLimit = 5.0;
    double negativeSpeedRateLimit = -5.0;
    private SlewRateLimiter speedLimiter = new SlewRateLimiter(positiveSpeedRateLimit, negativeSpeedRateLimit, 0);

    // Rotation values
    private DoubleSupplier turn;
    // Value to scale joystick rotation input by. Should probably be less than 1.
    private double turnScale = 0.5;
    // SlewRateLimiter and values to smooth acceleration
    double positiveTurnRateLimit = 10.0;
    double negativeTurnRateLimit = -10.0;
    private SlewRateLimiter turnLimiter = new SlewRateLimiter(positiveTurnRateLimit, negativeTurnRateLimit, 0);

    // Controls whether inputs are squared.
    private boolean squareInputs = false;

    private DrivetrainSubsystem drivetrainSubsystem;

    /**
     * Command to drive the robot, using speed and rotation values.
     * @param speed Joystick speed in [-1.0, 1.0]
     * @param turn Joystick rotation in [-1.0, 1.0]
     * @param drivetrainSubsystem The DrivetrainSubsystem
     */
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
        // Update values from the SmartDashboard
        speedScale = SmartDashboard.getNumber("Speed scale", speedScale);
        turnScale = SmartDashboard.getNumber("Turn scale", turnScale);
        squareInputs = SmartDashboard.getBoolean("Square inputs", squareInputs);

        // Calculate speed values
        double driveSpeed = speed.getAsDouble() * speedScale * (squareInputs ? Math.abs(speed.getAsDouble()) : 1);
        double turnSpeed = turn.getAsDouble() * turnScale * (squareInputs ? Math.abs(turn.getAsDouble()) : 1);

        // Drive, applying SlewRateLimiter to smooth out values
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
