package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;

public class LimelightSubsystem extends SubsystemBase {
    private final String limelightName = "limelight";
    
    // LED Modes
    public static final int LED_MODE_PIPELINE = 0;  // Use LED Mode set in pipeline
    public static final int LED_MODE_FORCE_OFF = 1; // Force OFF
    public static final int LED_MODE_FORCE_BLINK = 2; // Force blink
    public static final int LED_MODE_FORCE_ON = 3; // Force ON
    
    // Camera Modes
    public static final int CAMERA_MODE_VISION = 0; // Vision processor
    public static final int CAMERA_MODE_DRIVER = 1; // Driver Camera (Increases exposure, disables vision processing)

    public LimelightSubsystem() {
        // Set default pipeline on init
        setPipeline(0);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    /**
     * @return horizontal offset from crosshair to target (-27 degrees to 27 degrees)
     */
    public double getX() {
        return LimelightHelpers.getTX(limelightName);
    }
    
    /**
     * @return vertical offset from crosshair to target (-20.5 degrees to 20.5 degrees)
     */
    public double getY() {
        return LimelightHelpers.getTY(limelightName);
    }
    
    /**
     * @return target area (0% to 100% of image)
     */
    public double getArea() {
        return LimelightHelpers.getTA(limelightName);
    }
    
    /**
     * @return true if the Limelight has a valid target
     */
    public boolean hasValidTarget() {
        return LimelightHelpers.getTV(limelightName);
    }

    /**
     * Gets the latency of the pipeline (time between capture and data in NetworkTables)
     * @return latency in milliseconds
     */
    public double getPipelineLatency() {
        return LimelightHelpers.getLatency_Pipeline(limelightName);
    }

    /**
     * Gets the latency contribution from image capture
     * @return latency in milliseconds
     */
    public double getCaptureLatency() {
        return LimelightHelpers.getLatency_Capture(limelightName);
    }

    /**
     * Gets the robot's pose in 3D space relative to the field
     * Note: Requires AprilTags and proper camera calibration
     * @return Pose3d object containing position and rotation
     */
    public Pose3d getBotPose3d() {
        return LimelightHelpers.getBotPose3d(limelightName);
    }

    /**
     * Gets the robot's pose in 2D space relative to the field
     * Note: Requires AprilTags and proper camera calibration
     * @return Pose2d object containing position and rotation
     */
    public Pose2d getBotPose2d() {
        return LimelightHelpers.getBotPose2d(limelightName);
    }
    
    /**
     * Sets LED mode
     * @param mode Use LED_MODE_* constants
     */
    public void setLEDMode(int mode) {
        LimelightHelpers.setLEDMode(limelightName, mode);
    }
    
    /**
     * Sets camera mode
     * @param mode Use CAMERA_MODE_* constants
     */
    public void setCameraMode(int mode) {
        LimelightHelpers.setCameraMode(limelightName, mode);
    }
    
    /**
     * Sets current pipeline
     * @param pipeline Pipeline index (0-9)
     */
    public void setPipeline(int pipeline) {
        LimelightHelpers.setPipelineIndex(limelightName, pipeline);
    }

    /**
     * Takes a snapshot. Useful for debugging vision pipelines.
     * @param takeSnapshot 0 to stop taking snapshots, 1 to take a snapshot
     */
    public void takeSnapshot(int takeSnapshot) {
        LimelightHelpers.takeSnapshot(limelightName, takeSnapshot);
    }

    /**
     * Sets the stream mode for viewing camera feed
     * @param mode 0 = Side-by-side, 1 = PiP Main, 2 = PiP Secondary
     */
    public void setStreamMode(int mode) {
        LimelightHelpers.setStreamMode(limelightName, mode);
    }
}
