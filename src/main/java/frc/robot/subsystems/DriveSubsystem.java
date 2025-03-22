package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.RobotState;
import frc.robot.util.Logger;

public class DriveSubsystem extends SubsystemBase {
    // Commented out limelight camera as it's no longer used
    // private LimelightSubsystem m_LimelightSubsystem;
    // PoseEstimator for tracking robot pose
    private PoseEstimator m_PoseEstimator;

    // Locations for the swerve drive modules relative to the robot center
    private final Translation2d m_frontLeftLocation = DriveConstants.FRONT_LEFT_LOCATION;
    private final Translation2d m_frontRightLocation = DriveConstants.FRONT_RIGHT_LOCATION;
    private final Translation2d m_backLeftLocation = DriveConstants.BACK_LEFT_LOCATION;
    private final Translation2d m_backRightLocation = DriveConstants.BACK_RIGHT_LOCATION;

    // Slew rate limiters to make joystick inputs more gentle
    private final SlewRateLimiter m_xSpeedLimiter = new SlewRateLimiter(DriveConstants.MAX_MAGNITUDE_SLEW_RATE);
    private final SlewRateLimiter m_ySpeedLimiter = new SlewRateLimiter(DriveConstants.MAX_MAGNITUDE_SLEW_RATE);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(DriveConstants.MAX_ROTATIONAL_SLEW_RATE_RPS);

    // Motor controllers for the swerve drive modules
    private final SwerveModule m_frontLeft = new SwerveModule(
        DriveConstants.DRIVE_FRONT_LEFT_CAN_ID, 
        DriveConstants.DRIVE_TURN_FRONT_LEFT_CAN_ID, 
        DriveConstants.FRONT_LEFT_CHASIS_ANGULAR_OFFSET,
        true, "FrontLeft");

    private final SwerveModule m_frontRight = new SwerveModule(
        DriveConstants.DRIVE_FRONT_RIGHT_CAN_ID, 
        DriveConstants.DRIVE_TURN_FRONT_RIGHT_CAN_ID, 
        DriveConstants.FRONT_RIGHT_CHASIS_ANGULAR_OFFSET,
        false, "FrontRight");
        
    private final SwerveModule m_backLeft = new SwerveModule(
        DriveConstants.DRIVE_REAR_LEFT_CAN_ID, 
        DriveConstants.DRIVE_TURN_REAR_LEFT_CAN_ID, 
        DriveConstants.BACK_LEFT_CHASIS_ANGULAR_OFFSET,
        true, "BackLeft");

    private final SwerveModule m_backRight = new SwerveModule(
        DriveConstants.DRIVE_REAR_RIGHT_CAN_ID, 
        DriveConstants.DRIVE_TURN_REAR_RIGHT_CAN_ID, 
        DriveConstants.BACK_RIGHT_CHASIS_ANGULAR_OFFSET,
        false, "BackRight");

    public static final double kMaxDriveVEL = 6.58; // m/s
    
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    private final Pigeon2 m_gyro = new Pigeon2(DriveConstants.PIGEON_CAN_ID); // Update the ID based on your Pigeon's CAN ID
    // initialize the field for simulator tracking
    private final Field2d m_field = new Field2d();

    private int updateCounter = 0;

    private DoubleLogEntry m_speedLog;
    private DoubleLogEntry m_headingLog;

    public DriveSubsystem() {
        m_PoseEstimator = new PoseEstimator(this);
        // Reset the gyro
        m_gyro.reset();

        // log field into smartdashboard
        SmartDashboard.putData("Field", m_field);

        try{
            DriveConstants.pathPlannerConfig = RobotConfig.fromGUISettings();
          } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
          }

        // Configure AutoBuilder last
        AutoBuilder.configure(
                this::getPose, // Robot pose supplier
                this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getCurrentSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds, feedforwards) -> drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(1.0, 0.0, 0.0) // Rotation PID constants
                ),
                DriveConstants.pathPlannerConfig, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false; 
                }, 
                this // Reference to this subsystem to set requirements
        ); 

        // Initialize DataLogManager entries
        DataLog log = DataLogManager.getLog();
        m_speedLog = new DoubleLogEntry(log, "/drive/speed");
        m_headingLog = new DoubleLogEntry(log, "/drive/heading");
    }

    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed Speed of the robot in the x direction (forward).
     * @param ySpeed Speed of the robot in the y direction (sideways).
     * @param rot Angular rate of the robot.
     */
    public void drive(double xSpeed, double ySpeed, double rot) {
        // Debug input values
        SmartDashboard.putNumber("Drive/Input/X", xSpeed);
        SmartDashboard.putNumber("Drive/Input/Y", ySpeed);
        SmartDashboard.putNumber("Drive/Input/Rot", rot);

        // If all inputs are zero, stop the motors
        if (Math.abs(xSpeed) < 1E-6 && Math.abs(ySpeed) < 1E-6 && Math.abs(rot) < 1E-6) {
            stop();
            return;
        }

        // Convert the commanded speeds from [-1, 1] to real speeds
        xSpeed = xSpeed * DriveConstants.MAX_SPEED_IN_MPS;
        ySpeed = ySpeed * DriveConstants.MAX_SPEED_IN_MPS;
        rot = rot * DriveConstants.MAX_ANGULAR_SPEED_IN_RPS;

        /*if (!RobotState.isAutonomous()) {
            // Apply slew rate limiters to smooth out the inputs
            xSpeed = m_xSpeedLimiter.calculate(xSpeed);
            ySpeed = m_ySpeedLimiter.calculate(ySpeed);
            
            // If changing rotation direction, use a higher slew rate or bypass
            double currentRot = m_rotLimiter.calculate(0); // Get current value without changing it
            if (currentRot * rot < 0) { // If signs are different (changing direction)
                rot = rot * 0.8; // Apply directly with slight reduction
            } else {
                rot = m_rotLimiter.calculate(rot); // Normal slew rate
            }
        } */

        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rot);
        var swerveModuleStates = kinematics.toSwerveModuleStates(speeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, 4.0);

        SmartDashboard.putNumber("Chasis Speeds X", speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Chasis Speeds Y", speeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Chasis Speeds Rotation", speeds.omegaRadiansPerSecond);


        // Log detailed turning motor commands
        Logger.log("Setting module states:");
        Logger.log("Front Left - Target Speed: " + swerveModuleStates[0].speedMetersPerSecond + 
                  " m/s, Target Angle: " + swerveModuleStates[0].angle.getDegrees() + "°");
        Logger.log("Front Right - Target Speed: " + swerveModuleStates[1].speedMetersPerSecond + 
                  " m/s, Target Angle: " + swerveModuleStates[1].angle.getDegrees() + "°");
        Logger.log("Back Left - Target Speed: " + swerveModuleStates[2].speedMetersPerSecond + 
                  " m/s, Target Angle: " + swerveModuleStates[2].angle.getDegrees() + "°");
        Logger.log("Back Right - Target Speed: " + swerveModuleStates[3].speedMetersPerSecond + 
                  " m/s, Target Angle: " + swerveModuleStates[3].angle.getDegrees() + "°");

        m_frontLeft.setDesiredState(swerveModuleStates[0]);
        m_frontRight.setDesiredState(swerveModuleStates[1]);
        m_backLeft.setDesiredState(swerveModuleStates[2]);
        m_backRight.setDesiredState(swerveModuleStates[3]);
    }

    public void drive(ChassisSpeeds speeds) {
        double xSpeed = speeds.vxMetersPerSecond;
        double ySpeed = speeds.vyMetersPerSecond;
        double rot = speeds.omegaRadiansPerSecond;
        drive(xSpeed, ySpeed, rot);
    }

    public void stop() {
        m_frontLeft.stop();
        m_frontRight.stop();
        m_backLeft.stop();
        m_backRight.stop();
    }

    /**
     * Returns the gyro rotation as a Rotation2d object
     */
    public Rotation2d getGyroRotation() {
        return Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble());
    }

    /**
     * Returns the current pose of the robot
     */
    public Pose2d getPose() {
        return m_PoseEstimator.getPose2d();
    }

    /**
     * Resets the odometry to a known pose
     */
    public void resetOdometry(Pose2d pose) {
        m_PoseEstimator.setCurrentPose(pose);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    public SwerveDriveKinematics getDriveKinematics() {
        return kinematics;
    }

    @Override
    public void periodic() {
        // Only update SmartDashboard every 10 cycles to reduce NT traffic
        updateCounter++;
        if (updateCounter >= 50) {
            try {
                // set robot position in the field
                m_field.setRobotPose(m_PoseEstimator.getPose2d());
                // log array of all swerve modules to be put into advantagescope simulation
                double loggingState[] = {
                    m_frontLeft.getSteerAngle(),
                    m_frontLeft.getDriveSpeed(),
                    m_frontRight.getSteerAngle(),
                    m_frontRight.getDriveSpeed(),
                    m_backLeft.getSteerAngle(),
                    m_backLeft.getDriveSpeed(),
                    m_backRight.getSteerAngle(),
                    m_backRight.getDriveSpeed()
                };

                SmartDashboard.putNumberArray("SwerveModuleStates", loggingState);
                
                // Log detailed turning motor data for each module
                Logger.log("Turning Motor Debug Data:");
                Logger.log("Front Left - Angle: " + Math.toDegrees(m_frontLeft.getSteerAngle()) + 
                          "°, Target: " + Math.toDegrees(m_frontLeft.getDesiredAngle()) + "°");
                Logger.log("Front Right - Angle: " + Math.toDegrees(m_frontRight.getSteerAngle()) + 
                          "°, Target: " + Math.toDegrees(m_frontRight.getDesiredAngle()) + "°");
                Logger.log("Back Left - Angle: " + Math.toDegrees(m_backLeft.getSteerAngle()) + 
                          "°, Target: " + Math.toDegrees(m_backLeft.getDesiredAngle()) + "°");
                Logger.log("Back Right - Angle: " + Math.toDegrees(m_backRight.getSteerAngle()) + 
                          "°, Target: " + Math.toDegrees(m_backRight.getDesiredAngle()) + "°");
                

                // Add odometry data to SmartDashboard
                var pose = getPose();
                SmartDashboard.putNumber("Pose/X", pose.getX());
                SmartDashboard.putNumber("Pose/Y", pose.getY());
                SmartDashboard.putNumber("Pose/Rotation", pose.getRotation().getDegrees());
                SmartDashboard.putNumber("Gyro/Angle", getGyroRotation().getDegrees());

                // Log important values
                var chassisSpeeds = kinematics.toChassisSpeeds(getModuleStates());
                double speed = Math.sqrt(
                    chassisSpeeds.vxMetersPerSecond * chassisSpeeds.vxMetersPerSecond +
                    chassisSpeeds.vyMetersPerSecond * chassisSpeeds.vyMetersPerSecond
                );

                // Log to DataLog (saved to file)
                m_speedLog.append(speed);
                m_headingLog.append(getGyroRotation().getDegrees());

                // Log to SmartDashboard (network tables, viewable in Shuffleboard)
                SmartDashboard.putNumber("Drive/Speed (m/s)", speed);
                SmartDashboard.putNumber("Drive/Heading (deg)", getGyroRotation().getDegrees());
                SmartDashboard.putString("Drive/Pose", getPose().toString());
            } catch (Exception e) {
                System.err.println("Error updating SmartDashboard: " + e.getMessage());
            }
            updateCounter = 0;
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, kMaxDriveVEL);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
    }

    public SwerveModuleState[] getModuleStates() {
        return new SwerveModuleState[] {
            m_frontLeft.getState(),
            m_frontRight.getState(),
            m_backLeft.getState(),
            m_backRight.getState()
        };
    }
    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
        };
    }

    public Rotation2d getHeading() {
        return getGyroRotation();
    }
    
    public Command driveToEndPose(Pose2d endPose) {
        PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI);

        return AutoBuilder.pathfindToPose(endPose, constraints, 0.0);
    }

    
    @Override
    public void simulationPeriodic() {
        m_frontLeft.updateSimulatorState();
        m_frontRight.updateSimulatorState();
        m_backLeft.updateSimulatorState();
        m_backRight.updateSimulatorState();

        double angularVelocity = kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond;
        updateGyroSimulatorState(angularVelocity);
    }

    public void updateGyroSimulatorState(double angularVelocity) {
        // convert radians per second to degrees per second
        double angularVelocityDegrees = angularVelocity * (180 / Math.PI);
        double newYaw = m_gyro.getYaw().getValueAsDouble() +  angularVelocityDegrees * 0.02;
        m_gyro.getSimState().setRawYaw(newYaw);
    }
}
