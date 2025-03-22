package frc.robot.subsystems;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PoseEstimator extends SubsystemBase{
    private SwerveDrivePoseEstimator m_swerveEstimator;
    private DriveSubsystem m_driveSubsystem;
    // Commented out Limelight subsystem as it's no longer used
    // private LimelightSubsystem m_limelight;
    private static final Vector<N3> stateStdDevs =
      VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
    private static final Vector<N3> visionMeasurementStdDevs =
      VecBuilder.fill(0.5, 0.5, Units.degreesToRadians(10));

     public PoseEstimator(DriveSubsystem drive) {
        m_driveSubsystem = drive;
        // m_limelight = limelight; // Commented out as Limelight is no longer used
        m_swerveEstimator =
            new SwerveDrivePoseEstimator(
                m_driveSubsystem.getDriveKinematics(), 
                m_driveSubsystem.getHeading(), 
                m_driveSubsystem.getModulePositions(), 
                new Pose2d(), 
                stateStdDevs, 
                visionMeasurementStdDevs);
    }

    public Pose2d getPose2d() {
        return m_swerveEstimator.getEstimatedPosition();
    }

    public void setCurrentPose(Pose2d pose) {
        m_swerveEstimator.resetPosition(
            m_driveSubsystem.getHeading(),
            m_driveSubsystem.getModulePositions(),
            pose);
    }

    public void resetFieldPosition() {
        setCurrentPose(new Pose2d());
    }
    @Override
    public void periodic() {
        // Update pose estimator with drivetrain data
        m_swerveEstimator.update(
            m_driveSubsystem.getHeading(),
            m_driveSubsystem.getModulePositions());
        
        // Commented out Limelight vision updates as they're no longer used
        /*
        // If we have a vision measurement, add it to the pose estimator
        if (m_limelight.isTargetValid()) {
        Pose2d visionMeasurement = m_limelight.getPose(m_driveSubsystem.getGyroRotation());
        double resultTimestamp = m_limelight.getTimeRecordedInMilis();
        m_swerveEstimator.addVisionMeasurement(visionMeasurement, resultTimestamp);
        }
        */
    }
}
