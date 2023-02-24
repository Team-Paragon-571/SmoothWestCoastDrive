package frc.robot.Commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.DrivetrainSubsystem;

/**
 * Command to set the drivetrain's neutral mode
 * @see frc.robot.Subsystems.DrivetrainSubsystem#setNeutralMode
 */
public class SetNeutralModeCommand extends InstantCommand {
    private final NeutralMode brakeMode;
    private DrivetrainSubsystem drivetrainSubsystem;

    public SetNeutralModeCommand(NeutralMode brakeMode, DrivetrainSubsystem drivetrainSubsystem) {
        this.brakeMode = brakeMode;
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    @Override
    public void execute() {
        drivetrainSubsystem.setNeutralMode(brakeMode);
    }
    
}
