package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LimelightSubsystem;

public class LimelightDebugCommand extends Command {
    private final LimelightSubsystem m_limelight;
    private final SendableChooser<Integer> m_ledModeChooser = new SendableChooser<>();
    private final SendableChooser<Integer> m_cameraModeChooser = new SendableChooser<>();
    private final SendableChooser<Integer> m_streamModeChooser = new SendableChooser<>();
    private boolean m_lastSnapshotState = false;

    public LimelightDebugCommand(LimelightSubsystem limelight) {
        m_limelight = limelight;
        addRequirements(limelight);

        // Setup LED mode chooser
        m_ledModeChooser.setDefaultOption("Pipeline Default", LimelightSubsystem.LED_MODE_PIPELINE);
        m_ledModeChooser.addOption("Force OFF", LimelightSubsystem.LED_MODE_FORCE_OFF);
        m_ledModeChooser.addOption("Force Blink", LimelightSubsystem.LED_MODE_FORCE_BLINK);
        m_ledModeChooser.addOption("Force ON", LimelightSubsystem.LED_MODE_FORCE_ON);
        SmartDashboard.putData("Limelight/LED Mode", m_ledModeChooser);

        // Setup camera mode chooser
        m_cameraModeChooser.setDefaultOption("Vision Processor", LimelightSubsystem.CAMERA_MODE_VISION);
        m_cameraModeChooser.addOption("Driver Camera", LimelightSubsystem.CAMERA_MODE_DRIVER);
        SmartDashboard.putData("Limelight/Camera Mode", m_cameraModeChooser);

        // Setup stream mode chooser
        m_streamModeChooser.setDefaultOption("Side-by-side", 0);
        m_streamModeChooser.addOption("PiP Main", 1);
        m_streamModeChooser.addOption("PiP Secondary", 2);
        SmartDashboard.putData("Limelight/Stream Mode", m_streamModeChooser);

        // Add pipeline selector
        SmartDashboard.putNumber("Limelight/Pipeline", 0);

        // Add snapshot button
        SmartDashboard.putBoolean("Limelight/Take Snapshot", false);
    }

    @Override
    public void execute() {
        // Update camera controls from dashboard
        m_limelight.setLEDMode(m_ledModeChooser.getSelected());
        m_limelight.setCameraMode(m_cameraModeChooser.getSelected());
        m_limelight.setStreamMode(m_streamModeChooser.getSelected());
        
        // Update pipeline
        int pipeline = (int) SmartDashboard.getNumber("Limelight/Pipeline", 0);
        m_limelight.setPipeline(pipeline);

        // Handle snapshot toggle
        boolean snapshotRequested = SmartDashboard.getBoolean("Limelight/Take Snapshot", false);
        if (snapshotRequested && !m_lastSnapshotState) {
            m_limelight.takeSnapshot(1);
        } else if (!snapshotRequested && m_lastSnapshotState) {
            m_limelight.takeSnapshot(0);
        }
        m_lastSnapshotState = snapshotRequested;

        // Basic target information
        boolean hasTarget = m_limelight.hasValidTarget();
        double tx = m_limelight.getX();
        double ty = m_limelight.getY();
        double ta = m_limelight.getArea();

        // Latency information
        double pipelineLatency = m_limelight.getPipelineLatency();
        double captureLatency = m_limelight.getCaptureLatency();
        double totalLatency = pipelineLatency + captureLatency;

        // Get pose information (if AprilTags are being used)
        Pose2d botPose2d = m_limelight.getBotPose2d();
        Pose3d botPose3d = m_limelight.getBotPose3d();

        // Output to SmartDashboard
        SmartDashboard.putBoolean("Limelight/Has Target", hasTarget);
        SmartDashboard.putNumber("Limelight/Target X", tx);
        SmartDashboard.putNumber("Limelight/Target Y", ty);
        SmartDashboard.putNumber("Limelight/Target Area", ta);
        
        SmartDashboard.putNumber("Limelight/Pipeline Latency (ms)", pipelineLatency);
        SmartDashboard.putNumber("Limelight/Capture Latency (ms)", captureLatency);
        SmartDashboard.putNumber("Limelight/Total Latency (ms)", totalLatency);

        // Only show pose data if we have a target
        if (hasTarget) {
            // 2D Pose Data
            SmartDashboard.putNumber("Limelight/Field Position/X", botPose2d.getX());
            SmartDashboard.putNumber("Limelight/Field Position/Y", botPose2d.getY());
            SmartDashboard.putNumber("Limelight/Field Position/Rotation", botPose2d.getRotation().getDegrees());
            
            // 3D Pose Data
            SmartDashboard.putNumber("Limelight/Field Position/Z", botPose3d.getZ());
            SmartDashboard.putNumber("Limelight/Field Position/Pitch", botPose3d.getRotation().getY());
            SmartDashboard.putNumber("Limelight/Field Position/Roll", botPose3d.getRotation().getX());
        }

        // Status information
        SmartDashboard.putString("Limelight/Current Mode", 
            m_cameraModeChooser.getSelected() == LimelightSubsystem.CAMERA_MODE_VISION ? 
            "Vision Processing" : "Driver Camera");
        
        SmartDashboard.putString("Limelight/LED State",
            switch(m_ledModeChooser.getSelected()) {
                case LimelightSubsystem.LED_MODE_PIPELINE -> "Pipeline Default";
                case LimelightSubsystem.LED_MODE_FORCE_OFF -> "Force OFF";
                case LimelightSubsystem.LED_MODE_FORCE_BLINK -> "Force Blink";
                case LimelightSubsystem.LED_MODE_FORCE_ON -> "Force ON";
                default -> "Unknown";
            });
    }

    @Override
    public boolean isFinished() {
        // This command never finishes on its own - it runs until interrupted
        return false;
    }
}
