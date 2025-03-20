package frc.robot.commands;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LimelightSubsystem;

public class AprilTagAlignmentCommand extends SequentialCommandGroup {
    
    public AprilTagAlignmentCommand(LimelightSubsystem limelight, DriveSubsystem drive) {
        int aprilTagID = limelight.getTargetID();
        AprilTagFieldLayout aprilTagField = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
        Pose3d tag = aprilTagField.getTagPose(aprilTagID).orElse(null);
        if (tag == null) {
            return;
        }
        double desiredAngle = tag.getRotation().getAngle() - 180;
        
        double movementDirection = limelight.getX() > 0 ? 1.0 : -1.0;
        addCommands(
            new RotateUntilDesiredAngleCommand(drive, desiredAngle),
        new MoveUntilAlignedCommand(drive, limelight, movementDirection));
    }

    private static class RotateUntilDesiredAngleCommand extends Command {
        private final DriveSubsystem drive;
        private final double desiredAngle;
        private Timer rotationTimeOut = new Timer();
        private static final int THRESHOLD = 5;

        public RotateUntilDesiredAngleCommand(DriveSubsystem drive, double angle) {
            this.drive = drive;
            this.desiredAngle = angle;
            addRequirements(drive);
        }

        @Override
        public void initialize() {
            rotationTimeOut.start();
        }

        @Override
        public void execute() {
            drive.drive(0, 0, 0.5);
        }

        @Override
        public boolean isFinished() {
            return (Math.abs(drive.getGyroRotation().getDegrees() - desiredAngle) < THRESHOLD
                    || rotationTimeOut.hasElapsed(2));
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
        private Timer moveTimeout;
        private static final double ANGLE_DIFF_THRESHOLD = 5;

        public MoveUntilAlignedCommand(DriveSubsystem drive, LimelightSubsystem limelight, double direction) {
            this.drive = drive;
            this.limelight = limelight;
            this.movementDirection = direction;
            Timer moveTimeout = new Timer();
            addRequirements(drive);
        }
        @Override
        public void initialize() {
            moveTimeout.start();
        }
        @Override
        public void execute() {
            drive.drive(0.1 * movementDirection, 0, 0);
        }

        @Override
        public boolean isFinished() {
            return Math.abs(limelight.getX()) < ANGLE_DIFF_THRESHOLD
            || moveTimeout.hasElapsed(2);
        }

        @Override
        public void end(boolean interrupted) {
            drive.stop();
        }
    }
}