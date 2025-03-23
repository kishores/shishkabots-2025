package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.Logger;

/**
 * Command to shoot a coral and then lower the elevator.
 * This starts the shooter motors and waits until the coral is ejected,
 * then automatically lowers the elevator to level 1 (intake position).
 */
public class ShootCommand extends SequentialCommandGroup {
    
    /**
     * Creates a command to shoot a coral and then lower the elevator
     * @param shooter The shooter subsystem
     * @param elevator The elevator subsystem
     */
    public ShootCommand(ShooterSubsystem shooter, ElevatorSubsystem elevator) {
        addCommands(
            // First shoot the coral
            new ShootCoralCommand(shooter, elevator)
        );
    }
    
    /**
     * Inner command that handles just the shooting part
     */
    private static class ShootCoralCommand extends Command {
        private final ShooterSubsystem shooter;
        private final ElevatorSubsystem elevator;

        public ShootCoralCommand(ShooterSubsystem shooter, ElevatorSubsystem elevator) {
            this.shooter = shooter;
            this.elevator = elevator;
            addRequirements(shooter);
        }

        @Override
        public void initialize() {
            Logger.log("Starting shooter motors");
            // have to create a elevator level for very bottom in order for this to run
            if (elevator.getCurrentLevel() == 0) {
                shooter.shootBottomLevelCoral();
            } else if (elevator.getCurrentLevel() == 3) {
                shooter.shootHighestLevelCoral();
            } else shooter.shootCoral();
        }

        @Override
        public void execute() {
            // State management is handled in the subsystem
        }

        @Override
        public void end(boolean interrupted) {
            if (interrupted) {
                Logger.log("Shooter stopped after interruption");
                shooter.emergencyStop();
            } else {
                Logger.log("Shooter stopped normally");
                // No need to stop motors here as the subsystem handles this automatically
            }
        }

        @Override
        public boolean isFinished() {
            // Command is finished when the shooter returns to NO_CORAL state
            return shooter.getState() == ShooterSubsystem.ShooterState.NO_CORAL;
        }
    }
}
