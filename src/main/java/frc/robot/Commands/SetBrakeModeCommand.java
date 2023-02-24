package frc.robot.Commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Subsystems.DrivetrainSubsystem;

public class SetBrakeModeCommand extends InstantCommand {
    private final NeutralMode brakeMode;
    private DrivetrainSubsystem drivetrainSubsystem;

    public SetBrakeModeCommand(NeutralMode brakeMode, DrivetrainSubsystem drivetrainSubsystem) {
        this.brakeMode = brakeMode;
        this.drivetrainSubsystem = drivetrainSubsystem;
    }

    @Override
    public void execute() {
        drivetrainSubsystem.setNeutralMode(brakeMode);
    }
    
}
