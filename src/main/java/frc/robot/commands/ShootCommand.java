package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    private final ShooterSubsystem shooter;

    public ShootCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        System.out.println("Starting shooter motors");
        shooter.shootCoral();
    }

    @Override
    public void execute() {
        // State management is handled in the subsystem
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            System.out.println("Shooter stopped after interruption");
            shooter.emergencyStop();
        } else {
            System.out.println("Shooter stopped normally");
            // No need to stop motors here as the subsystem handles this automatically
        }
    }

    @Override
    public boolean isFinished() {
        // Command is finished when the shooter returns to NO_CORAL state
        return shooter.getState() == ShooterSubsystem.ShooterState.NO_CORAL;
    }
}
