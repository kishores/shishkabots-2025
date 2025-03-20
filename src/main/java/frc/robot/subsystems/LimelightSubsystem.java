package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LimelightConstants;

public class LimelightSubsystem extends SubsystemBase {
    private final NetworkTable m_limelightTable;
    private AprilTagFieldLayout aprilTagField = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);
    private double millisTimeRecorded;
    
    // NetworkTable entries for common Limelight values
    private NetworkTableEntry tx;  // Horizontal offset from crosshair to target
    private NetworkTableEntry ty;  // Vertical offset from crosshair to target
    private NetworkTableEntry ta;  // Target area (0% to 100% of image)
    private NetworkTableEntry tv;  // Whether the limelight has any valid targets (0 or 1)
    
    public LimelightSubsystem() {
        m_limelightTable = NetworkTableInstance.getDefault().getTable("limelight");
        
        // Initialize NetworkTable entries
        tx = m_limelightTable.getEntry("tx");
        ty = m_limelightTable.getEntry("ty");
        ta = m_limelightTable.getEntry("ta");
        tv = m_limelightTable.getEntry("tv");
        
        // Set default pipeline
        setPipeline(0);
    }
    
    @Override
    public void periodic() {
        // Update values from NetworkTables
        double currentX = tx.getDouble(0.0);
        double currentY = ty.getDouble(0.0);
        double currentArea = ta.getDouble(0.0);
        double currentTarget = tv.getDouble(0.0);

        millisTimeRecorded = WPIUtilJNI.now() * 1e-3;
        
        // You can also log these values to SmartDashboard for debugging
        SmartDashboard.putNumber("Limelight X", currentX);
        SmartDashboard.putNumber("Limelight Y", currentY);
        SmartDashboard.putNumber("Limelight Area", currentArea);
        SmartDashboard.putBoolean("Limelight Has Target", currentTarget > 0.5);
    }
    
    /**
     * @return horizontal offset from crosshair to target (-27 degrees to 27 degrees)
     */
    public double getX() {
        return tx.getDouble(0.0);
    }
    
    /**
     * @return vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees)
     */
    public double getY() {
        return ty.getDouble(0.0);
    }
    
    /**
     * @return target area (0% to 100% of image)
     */
    public double getArea() {
        return ta.getDouble(0.0);
    }
    
    /**
     * @return whether the limelight has any valid targets (0 or 1)
     */
    public boolean hasValidTarget() {
        double targetValue = getRawTargetValue();
        return targetValue > 0.5;
    }
    
    /**
     * Sets LED mode
     * @param mode 0 = use pipeline mode, 1 = force off, 2 = force blink, 3 = force on
     */
    public void setLEDMode(int mode) {
        m_limelightTable.getEntry("ledMode").setNumber(mode);
    }
    
    /**
     * Sets camera mode
     * @param mode 0 = vision processor, 1 = driver camera
     */
    public void setCameraMode(int mode) {
        m_limelightTable.getEntry("camMode").setNumber(mode);
    }
    
    /**
     * Sets current pipeline
     * @param pipeline Pipeline index (0-9)
     */
    public void setPipeline(int pipeline) {
        m_limelightTable.getEntry("pipeline").setNumber(pipeline);
    }
    
    /**
     * Sets the exposure time for the camera
     * @param exposure Exposure time in milliseconds (0-100)
     */
    public void setExposure(double exposure) {
        m_limelightTable.getEntry("exposure").setNumber(exposure);
    }

    /**
     * Sets the black level offset
     * @param blackLevel Black level offset (0-100)
     */
    public void setBlackLevel(double blackLevel) {
        m_limelightTable.getEntry("black_level").setNumber(blackLevel);
    }

    /**
     * Gets the raw target detection value from NetworkTables
     * @return raw tv value (0.0 if no target, 1.0 if target detected)
     */
    public double getRawTargetValue() {
        return m_limelightTable.getEntry("tv").getDouble(0.0);
    }

    /**
     * Gets the detected AprilTag ID
     * @return AprilTag ID number, or 0.0 if no tag detected
     */
    public int getTargetID() {
        return (int) m_limelightTable.getEntry("tid").getDouble(0.0);
    }
    public boolean isTargetValid() {
        return m_limelightTable.getEntry("tv").getDouble(0.0) == 1;
    }
    
    // get the timestamp of robot in miliseconds
    public double getTimeRecordedInMilis() {
        return millisTimeRecorded;
    }
    /*
     * If it detects an AprilTag ID, get it's pose to estimate the robot location with tx, ty, heightDiff
     */
    public synchronized Pose2d getPose(Rotation2d robotRotation2d) {
        // if not valid return null 
        if (!isTargetValid()) {
            return null;    
        } 
        Optional<Pose3d> tagPoseOptional = aprilTagField.getTagPose(getTargetID());
        if (tagPoseOptional.isEmpty()) {
            return null;
        }

        Pose3d tag = tagPoseOptional.get();

        double yaw = robotRotation2d.getRadians();
        double heightDiff = tag.getZ() - LimelightConstants.kCameraToRobot.getZ();
        double distance = heightDiff / Math.tan(Math.toRadians(getY()));
        double beta = yaw - Math.toRadians(getX());
        double x = Math.cos(beta) * distance;
        double y = Math.sin(beta) * distance;
        Translation2d tagToCamera = new Translation2d(-x, -y);

        Pose2d cameraPose =
        new Pose2d(tag.toPose2d().getTranslation().plus(tagToCamera), new Rotation2d(yaw));

        Translation2d offset = LimelightConstants.kCameraToRobot.toTranslation2d().rotateBy(robotRotation2d);
        return new Pose2d(cameraPose.getTranslation().minus(offset), new Rotation2d(yaw));
    }
}
