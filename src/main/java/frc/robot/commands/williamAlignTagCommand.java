package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class williamAlignTagCommand extends SequentialCommandGroup {
    
    public williamAlignTagCommand(LimelightSubsystem limelight, DriveSubsystem drive) {
        int aprilTagID = limelight.getTargetID();
        AprilTagFieldLayout aprilTagField = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        Pose3d tag = aprilTagField.getTagPose(aprilTagID).orElse(null);
        if (tag == null) {
            return;
        }
        double desiredAngle = (tag.getRotation().getAngle() - 180) % 360;
        // make sure desiredAngle between -180 and 180
        desiredAngle = ((desiredAngle + 180) % 360 + 360) % 360 - 180;
        double movementDirection = limelight.getY() > 0 ? 1.0 : -1.0;
        if (tag != null) {
            addRequirements(limelight, drive);
        addCommands(
            new RotateUntilDesiredAngleCommand(drive, desiredAngle),
        new MoveUntilAlignedCommand(drive, limelight, movementDirection));
        }
    }

    public static class RotateUntilDesiredAngleCommand extends Command {
        private final DriveSubsystem drive;
        private final double desiredAngle;
        private Timer rotationTimeOut = new Timer();
        private static final int ANGLE_DIFF_THRESHOLD = 2;

        public RotateUntilDesiredAngleCommand(DriveSubsystem drive, double angle) {
            this.drive = drive;
            this.desiredAngle = angle;
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            rotationTimeOut.restart();
        }

        @Override
        public void execute() {
            double currentAngle = drive.getGyroRotation().getDegrees();
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

    private static class MoveUntilAlignedCommand extends Command {
        private final DriveSubsystem drive;
        private final LimelightSubsystem limelight;
        private final double movementDirection;
        private final Timer moveTimeout = new Timer();
        private static final double HORIZONTAL_ANGLE_DIFFERENCE = 3;

        public MoveUntilAlignedCommand(DriveSubsystem drive, LimelightSubsystem limelight, double direction) {
            this.drive = drive;
            this.limelight = limelight;
            this.movementDirection = direction;
            addRequirements(drive);
        }
        @Override
        public void initialize() {
            moveTimeout.restart();
        }
        @Override
        public void execute() {
            drive.drive(0.1 * movementDirection, 0, 0);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(limelight.getY()) < HORIZONTAL_ANGLE_DIFFERENCE
            || moveTimeout.hasElapsed(2);
        }

        @Override
        public void end(boolean interrupted) {
            drive.stop();
        }
    }
}