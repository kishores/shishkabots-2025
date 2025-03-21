package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.williamAlignTagCommand.RotateUntilDesiredAngleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class william180ReverseTech extends SequentialCommandGroup{
    public william180ReverseTech(LimelightSubsystem limelight, DriveSubsystem drive) {
        addCommands(
            new williamAlignTagCommand(limelight, drive),
            new angleForReversal(drive),
            new reversalTechnique(drive)
        );
    }

    private static class angleForReversal extends Command{
        private final DriveSubsystem drive;
        private Timer rotationTimeOut = new Timer();
        private static final int ANGLE_DIFF_THRESHOLD = 2;
        private double currentAngle;
        private double desiredAngle;

        public angleForReversal(DriveSubsystem drive){
            this.drive = drive;
            addRequirements(drive);
        }
        @Override
        public void initialize() {
            rotationTimeOut.restart();
            double currentAngle = drive.getGyroRotation().getDegrees();
            desiredAngle = ((currentAngle + 180) % 360 + 360) % 360 - 180;
        }
        @Override
        public void execute(){
            currentAngle = drive.getGyroRotation().getDegrees();
            double angleDiff = (desiredAngle - currentAngle + 360) % 360;
            
            // Ensure shortest rotation direction
            if (angleDiff > 180) angleDiff -= 360;

            double rotationSpeed = (angleDiff > 0) ? 0.5 : -0.5;
            drive.drive(0, 0, rotationSpeed);
        }
        @Override
        public boolean isFinished() {
            return (Math.abs(drive.getGyroRotation().getDegrees() - desiredAngle) < ANGLE_DIFF_THRESHOLD
                    || rotationTimeOut.hasElapsed(3));
        }

        @Override
        public void end(boolean interrupted) {
            drive.stop();
        }
    }
    private static class reversalTechnique extends Command {
        private final DriveSubsystem drive;
        private Timer reversalTimeout = new Timer();

        public reversalTechnique(DriveSubsystem drive){
            this.drive = drive;
            addRequirements(drive);
        }
        @Override
        public void initialize(){
            reversalTimeout.restart();
        }
        @Override  
        public void execute() {
            drive.drive(0, -0.5, 0.0);
        }
        @Override
        public boolean isFinished() {
            return reversalTimeout.hasElapsed(3) || drive.getCurrentSpeeds().vyMetersPerSecond <= 0.4;
        }
        @Override
        public void end(boolean interrupted) {
            drive.stop();
        }
    }
}
