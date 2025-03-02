package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import java.io.IOException;
import java.net.HttpURLConnection;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.concurrent.CompletableFuture;

public class LimelightHelpers {

    public static class LimelightTarget_Retro {
        public double tx;
        public double ty;
        public double ta;
        public double ts;
        public double tl;
        public double tshort;
        public double tlong;
        public double thor;
        public double tvert;
        public double getLatency_Pipeline() {
            return tl;
        }
        public double getLatency_Capture() {
            return tl_cap;
        }
        private double tl_cap;
        private double pipeline_latency;
        public double getPipelineLatency() {
            return pipeline_latency;
        }
    }

    public static class LimelightTarget_Fiducial {
        public double tx;
        public double ty;
        public double ta;
        public double ts;
        public double tl;
        public double tshort;
        public double tlong;
        public double thor;
        public double tvert;
        public double fID;
        public double family;
        public Pose3d targetpose_robotspace;
        public Pose3d targetpose_cameraspace;
        public Pose3d cameraPose_targetspace;
        public Pose3d robotPose_targetspace;
        public Pose3d robotPose_fieldspace;
        public double tx_rob;
        public double ty_rob;
        public double tz_rob;
        public double rx_rob;
        public double ry_rob;
        public double rz_rob;
        public double tl_cap;
        private double pipeline_latency;
        public double getPipelineLatency() {
            return pipeline_latency;
        }
        public double getLatency_Pipeline() {
            return tl;
        }
        public double getLatency_Capture() {
            return tl_cap;
        }
    }

    public static class Results {
        public double latency_pipeline;
        public double latency_capture;
        public double valid;
        public double ts;
        public LimelightTarget_Retro[] targets_Retro;
        public LimelightTarget_Fiducial[] targets_Fiducials;
        public Results() {
            targets_Retro = new LimelightTarget_Retro[0];
            targets_Fiducials = new LimelightTarget_Fiducial[0];
        }
    }

    public static class LimelightResults {
        public Results targetingResults;
        public String error;
        public LimelightResults() {
            targetingResults = new Results();
            error = "";
        }
    }

    private static NetworkTable getLimelightNTTable(String tableName) {
        return NetworkTableInstance.getDefault().getTable(tableName);
    }

    private static NetworkTableEntry getLimelightNTTableEntry(String tableName, String entryName) {
        return getLimelightNTTable(tableName).getEntry(entryName);
    }

    private static double getLimelightNTDouble(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getDouble(0.0);
    }

    private static void setLimelightNTDouble(String tableName, String entryName, double val) {
        getLimelightNTTableEntry(tableName, entryName).setDouble(val);
    }

    private static void setLimelightNTString(String tableName, String entryName, String val) {
        getLimelightNTTableEntry(tableName, entryName).setString(val);
    }

    private static String getLimelightNTString(String tableName, String entryName) {
        return getLimelightNTTableEntry(tableName, entryName).getString("");
    }

    public static double getTX(String limelightName) {
        return getLimelightNTDouble(limelightName, "tx");
    }

    public static double getTY(String limelightName) {
        return getLimelightNTDouble(limelightName, "ty");
    }

    public static double getTA(String limelightName) {
        return getLimelightNTDouble(limelightName, "ta");
    }

    public static boolean getTV(String limelightName) {
        return getLimelightNTDouble(limelightName, "tv") != 0;
    }

    public static double getLatency_Pipeline(String limelightName) {
        return getLimelightNTDouble(limelightName, "tl");
    }

    public static double getLatency_Capture(String limelightName) {
        return getLimelightNTDouble(limelightName, "tl_cap");
    }

    public static void setPipelineIndex(String limelightName, int pipelineIndex) {
        setLimelightNTDouble(limelightName, "pipeline", pipelineIndex);
    }

    public static void setLEDMode(String limelightName, int ledMode) {
        setLimelightNTDouble(limelightName, "ledMode", ledMode);
    }

    public static void setCameraMode(String limelightName, int cameraMode) {
        setLimelightNTDouble(limelightName, "camMode", cameraMode);
    }

    public static void setStreamMode(String limelightName, int streamMode) {
        setLimelightNTDouble(limelightName, "stream", streamMode);
    }

    public static void takeSnapshot(String limelightName, int snapshotMode) {
        setLimelightNTDouble(limelightName, "snapshot", snapshotMode);
    }

    public static void setPythonScriptData(String limelightName, String data) {
        setLimelightNTString(limelightName, "llrobot", data);
    }

    public static Pose3d getBotPose3d(String limelightName) {
        double[] poseArray = getLimelightNTTable(limelightName).getEntry("botpose").getDoubleArray(new double[6]);
        return new Pose3d(
            new Translation3d(poseArray[0], poseArray[1], poseArray[2]),
            new Rotation3d(Units.degreesToRadians(poseArray[3]), Units.degreesToRadians(poseArray[4]), Units.degreesToRadians(poseArray[5]))
        );
    }

    public static Pose2d getBotPose2d(String limelightName) {
        double[] poseArray = getLimelightNTTable(limelightName).getEntry("botpose").getDoubleArray(new double[6]);
        return new Pose2d(poseArray[0], poseArray[1], Rotation2d.fromDegrees(poseArray[5]));
    }
}
