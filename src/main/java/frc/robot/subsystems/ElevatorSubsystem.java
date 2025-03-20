package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.filter.LinearFilter;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.wpilibj.Timer;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax primaryElevatorMotor;
    private final SparkMax secondaryElevatorMotor;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;
    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    // Simulation
    private final DCMotor elevatorDCMotor;
    private final SparkMaxSim primaryElevatorMotorSim;
    private final SparkMaxSim secondaryElevatorMotorSim;

    // Constants
    private static final double MAX_OUTPUT = 1.0;
    private static final double MIN_OUTPUT = -1.0;
    private static final double TOLERANCE = 0.5;

    // Elevator Position Constants (in encoder units)
    private static final double BOTTOM_THRESHOLD = -5.0;
    private static final double TOP_THRESHOLD = 40.0;  // Adjust based on actual max height

    // Predefined heights in encoder units
    private static final double LEVEL_1_HEIGHT = 9.66;  // First level
    private static final double LEVEL_2_HEIGHT = 19.66;  // Mid level
    private static final double LEVEL_3_HEIGHT = 30.0;  // Top level

    // PID Constants - Tune these values during testing
    private static final double kP = 0.6;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kFF = 0.0;

    // Motion profile constants - controls speed in closed-loop mode
    // These are kept for future reference but not currently used
    private static final double MAX_VELOCITY = 20.0; // Maximum velocity in encoder units per second
    private static final double MAX_ACCELERATION = 40.0; // Maximum acceleration in encoder units per second squared
    private static final boolean USE_MOTION_PROFILE = false; // Set to true to use motion profiling

    // Torque mode constants
    private static final double ELEVATOR_TORQUE = 0.1; // Initial torque for movement (0-1)
    private static final double TORQUE_TIMEOUT = 0.8; // Time in seconds to apply torque before switching to PID
    private static final double POSITION_ERROR_THRESHOLD = 2.0; // Error threshold to switch to torque mode

    private static final int MAX_CURRENT = 40;

    // Position Control
    private double targetPosition = 0.0;
    private boolean inTorqueMode = false;
    private Timer torqueModeTimer = new Timer();

    // Error filter for smoother transitions
    private LinearFilter errorFilter;

    // Periodic counter for status updates
    private int periodicCounter = 0;

    /*
     * Elevator max height = 63 inches
     * First level = 29 inches
     * Second level = 44.5 inches
     * Third level = 70 inches
     */

    public ElevatorSubsystem(int primaryMotorCanId, int secondaryMotorCanId, int topLimitSwitchId, int bottomLimitSwitchId) {
        // Initialize motors
        primaryElevatorMotor = new SparkMax(primaryMotorCanId, SparkMax.MotorType.kBrushless);
        secondaryElevatorMotor = new SparkMax(secondaryMotorCanId, SparkMax.MotorType.kBrushless);

        // Initialize limit switches
        topLimitSwitch = new DigitalInput(topLimitSwitchId);
        bottomLimitSwitch = new DigitalInput(bottomLimitSwitchId);

        // Get encoder and controller from primary motor
        encoder = primaryElevatorMotor.getEncoder();
        closedLoopController = primaryElevatorMotor.getClosedLoopController();

        // Initialize error filter (single pole IIR filter with 0.1 time constant)
        errorFilter = LinearFilter.singlePoleIIR(0.1, 0.02);

        // Configure the primary motor with PID
        SparkMaxConfig primaryConfig = new SparkMaxConfig();
        primaryConfig
            .idleMode(IdleMode.kBrake)
            .inverted(false)
            .smartCurrentLimit(MAX_CURRENT);

        primaryConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .velocityFF(kFF)
            .outputRange(MIN_OUTPUT, MAX_OUTPUT);

        // Configure motion profiling if enabled
        if (USE_MOTION_PROFILE) {
            // We'll use standard PID with higher output limits instead of SmartMotion
            primaryConfig.closedLoop
                .outputRange(-1.0, 1.0); // Full range for faster movement
        }

        primaryElevatorMotor.configure(
            primaryConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // Configure the secondary motor (follower)
        SparkMaxConfig secondaryConfig = new SparkMaxConfig();
        secondaryConfig
            .idleMode(IdleMode.kBrake)
            .follow(primaryElevatorMotor, false)
            .smartCurrentLimit(MAX_CURRENT);

        secondaryElevatorMotor.configure(
            secondaryConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // Reset encoder position
        resetEncoder();

        // Initialize simulation objects if in simulation mode
        if (RobotBase.isSimulation()) {
            elevatorDCMotor = DCMotor.getNEO(1);
            primaryElevatorMotorSim = new SparkMaxSim(primaryElevatorMotor, elevatorDCMotor);
            secondaryElevatorMotorSim = new SparkMaxSim(secondaryElevatorMotor, elevatorDCMotor);
        } else {
            elevatorDCMotor = null;
            primaryElevatorMotorSim = null;
            secondaryElevatorMotorSim = null;
        }

        // Log initialization
        System.out.println("Elevator subsystem initialized");
    }

    /**
     * Set the target position for the elevator
     * @param position Target position in encoder units
     */
    public void setTargetPosition(double position) {
        // Safety check to ensure position is within bounds
        position = Math.min(Math.max(position, BOTTOM_THRESHOLD), TOP_THRESHOLD);

        targetPosition = position;
        System.out.println("Setting elevator position to " + position + " (current: " + getCurrentPosition() + ")");

        // Calculate error to determine if we need torque mode
        double error = Math.abs(targetPosition - getCurrentPosition());

        if (error > POSITION_ERROR_THRESHOLD) {
            // If error is large, use torque mode for initial movement
            enableTorqueMode();
        } else {
            // For small adjustments, just use PID
            closedLoopController.setReference(position, ControlType.kPosition);
        }
    }

    /**
     * Enable torque mode for initial movement
     */
    private void enableTorqueMode() {
        inTorqueMode = true;
        torqueModeTimer.reset();
        torqueModeTimer.start();

        // Determine direction based on error
        double direction = targetPosition > getCurrentPosition() ? 1.0 : -1.0;

        // Apply torque in the correct direction
        double torqueOutput = ELEVATOR_TORQUE * direction;

        // Set both motors to the same torque output
        primaryElevatorMotor.set(torqueOutput);

        System.out.println("Enabling torque mode with output: " + torqueOutput);
    }

    /**
     * Disable torque mode and switch to PID control
     */
    private void disableTorqueMode() {
        inTorqueMode = false;
        torqueModeTimer.stop();

        // Switch to PID control
        closedLoopController.setReference(targetPosition, ControlType.kPosition);

        System.out.println("Switching to PID control");
    }

    /**
     * Get the current position of the elevator in encoder units
     * @return Current position
     */
    public double getCurrentPosition() {
        return encoder.getPosition();
    }

    /**
     * Check if the elevator is at the target position
     * @return True if at target position
     */
    public boolean atTargetPosition() {
        return Math.abs(getCurrentPosition() - targetPosition) < TOLERANCE;
    }

    /**
     * Check if the elevator is at the top limit
     * @return True if at top limit
     */
    public boolean isAtTop() {
        return !topLimitSwitch.get();  // Limit switches are typically active LOW
    }

    /**
     * Check if the elevator is at the bottom limit
     * @return True if at bottom limit
     */
    public boolean isAtBottom() {
        return !bottomLimitSwitch.get();  // Limit switches are typically active LOW
    }

    /**
     * Reset the encoder position to zero
     */
    public void resetEncoder() {
        encoder.setPosition(0);
    }

    /**
     * Stop the elevator motors
     */
    public void stop() {
        System.out.println("***** Stopping elevator at position: " + getCurrentPosition());
        primaryElevatorMotor.stopMotor(); // Stop both motors
        inTorqueMode = false;
        torqueModeTimer.stop();
    }

    /**
     * Sets the elevator to a predefined level
     * @param level 1 for bottom, 2 for middle, 3 for top
     */
    public void goToLevel(int level) {
        System.out.println("Moving elevator to level " + level);
        switch (level) {
            case 1:
                setTargetPosition(LEVEL_1_HEIGHT);
                break;
            case 2:
                setTargetPosition(LEVEL_2_HEIGHT);
                break;
            case 3:
                setTargetPosition(LEVEL_3_HEIGHT);
                break;
            default:
                throw new IllegalArgumentException("Invalid level: " + level);
        }
    }

    /**
     * Returns the current level of the elevator (1, 2, or 3)
     * Returns 0 if between levels
     */
    public int getCurrentLevel() {
        double position = getCurrentPosition();

        if (Math.abs(position - LEVEL_1_HEIGHT) < TOLERANCE) {
            return 1;
        } else if (Math.abs(position - LEVEL_2_HEIGHT) < TOLERANCE) {
            return 2;
        } else if (Math.abs(position - LEVEL_3_HEIGHT) < TOLERANCE) {
            return 3;
        } else {
            return 0; // Between levels
        }
    }
    /**
     * Get the filtered error between current and target position
     */
    private double getFilteredError() {
        double error = targetPosition - getCurrentPosition();
        return errorFilter.calculate(error);
    }

    @Override
    public void periodic() {
        // Safety checks - stop if either limit switch is triggered OR position exceeds thresholds
        if (isAtTop() || getCurrentPosition() > TOP_THRESHOLD) {
            if (primaryElevatorMotor.get() > 0) {
                System.out.println("Elevator at top limit or exceeded threshold - STOPPING");
                stop();
            }
        }

        if (isAtBottom() || getCurrentPosition() < BOTTOM_THRESHOLD) {
            if (primaryElevatorMotor.get() < 0) {
                System.out.println("Elevator at bottom limit or exceeded threshold - STOPPING");
                stop();
            }
        }

        // Handle torque mode transition
        if (inTorqueMode) {
            // Debug information for torque mode
            double currentPosition = getCurrentPosition();
            double currentError = targetPosition - currentPosition;
            double filteredError = getFilteredError();
            double torqueTime = torqueModeTimer.get();
            double motorOutput = primaryElevatorMotor.get();

            System.out.println(String.format(
                "TORQUE_MODE_DEBUG - Time: %.3fs, Pos: %.2f, Target: %.2f, Error: %.2f, Filtered: %.2f, Output: %.2f",
                torqueTime, currentPosition, targetPosition, currentError, filteredError, motorOutput));

            // Check if we should exit torque mode based on timer
            if (torqueModeTimer.get() >= TORQUE_TIMEOUT) {
                disableTorqueMode();
            }
            // Alternatively, exit torque mode if we're close to the target
            else if (Math.abs(getFilteredError()) < TOLERANCE * 2) {
                disableTorqueMode();
            }
        }

        // Print periodic status every 50 calls (about once per second)
        if (periodicCounter++ % 50 == 0) {
            System.out.println(String.format("Elevator Status - Pos: %.2f, Target: %.2f, P1 Speed: %.2f, P2 Speed: %.2f, TorqueMode: %b",
                getCurrentPosition(), targetPosition, 
                primaryElevatorMotor.get(), secondaryElevatorMotor.get(), inTorqueMode));
        }

        updateTelemetry();

        // Update simulation
        if (RobotBase.isSimulation()) {
            updateSimulatorState();
        }
    }

    public void updateSimulatorState() {
        double positionError = targetPosition - encoder.getPosition();
        double velocityInchPerSec = positionError / 0.02;  // Basic simulation
        primaryElevatorMotorSim.iterate(velocityInchPerSec, primaryElevatorMotor.getBusVoltage(), 0.02);
        secondaryElevatorMotorSim.iterate(velocityInchPerSec, secondaryElevatorMotor.getBusVoltage(), 0.02);
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Elevator/CurrentPosition", getCurrentPosition());
        SmartDashboard.putNumber("Elevator/TargetPosition", targetPosition);
        SmartDashboard.putNumber("Elevator/CurrentLevel", getCurrentLevel());
        SmartDashboard.putBoolean("Elevator/AtTop", isAtTop());
        SmartDashboard.putBoolean("Elevator/AtBottom", isAtBottom());
        SmartDashboard.putBoolean("Elevator/AtTarget", atTargetPosition());
        SmartDashboard.putBoolean("Elevator/TorqueMode", inTorqueMode);
        SmartDashboard.putNumber("Elevator/FilteredError", getFilteredError());

        SmartDashboard.putNumber("Elevator/Primary/Current", primaryElevatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator/Primary/Voltage", primaryElevatorMotor.getBusVoltage());
        SmartDashboard.putNumber("Elevator/Primary/Speed", primaryElevatorMotor.get());

        SmartDashboard.putNumber("Elevator/Secondary/Current", secondaryElevatorMotor.getOutputCurrent());
        SmartDashboard.putNumber("Elevator/Secondary/Voltage", secondaryElevatorMotor.getBusVoltage());
        SmartDashboard.putNumber("Elevator/Secondary/Speed", secondaryElevatorMotor.get());
    }
}