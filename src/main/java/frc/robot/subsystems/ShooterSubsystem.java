package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Logger;

/**
 * Subsystem for controlling the shooter mechanism
 */
public class ShooterSubsystem extends SubsystemBase {
    // Shooter states
    public enum ShooterState {
        NO_CORAL,           // No coral in shooter, motors stopped
        READY_TO_INTAKE,    // Motors spinning at intake velocity, waiting for coral
        CORAL_INSIDE,       // Coral inside shooter, motors stopped
        SHOOT_CORAL         // Shooting coral, motors at shooting velocity
    }

    private final SparkMax leftMotor;
    private final SparkMax rightMotor;
    
    // Color sensor for game piece detection
    private ColorSensorV3 colorSensor;
    private static final int PROXIMITY_THRESHOLD = 100; // Adjust based on testing
    private int lastProximity = 0;
    private boolean hasColorSensor = false;

    // State management
    private ShooterState currentState = ShooterState.NO_CORAL;
    private final Timer stateTimer = new Timer();
    private static final double INTAKE_TIMEOUT = 10; // seconds to wait for coral to be fully inside

    // Motor configuration constants
    private static final double L4_SHOOTING_POWER = 0.6;
    private static final double SHOOTING_POWER = 0.4; // 35% power for shooting
    private static final double INTAKE_POWER = 0.325;   // 30% power for intake
    private static final double FINE_TUNE_POWER = 0.2; // 15% power for fine tuning
    private static final int MAX_CURRENT = 40; // Amps
    
    // Keep these for reference but they're not used with open-loop control
    private static final double kP = 2.7;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kFF = 0.000175;
    
    private static final double SHOOT_DURATION = 2.0; // seconds

    // telemetery timer
    private int periodicCounter = 0;

    public ShooterSubsystem(int leftMotorCanId, int rightMotorCanId) {
        leftMotor = new SparkMax(leftMotorCanId, MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorCanId, MotorType.kBrushless);

        // Try to initialize color sensor on the I2C port
        try {
            colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
            hasColorSensor = true;
            Logger.log("Color sensor initialized successfully");
        } catch (Exception e) {
            Logger.log("Color sensor not detected, running without game piece detection");
            hasColorSensor = false;
        }

        // Configure the left motor (leader)
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig
            .idleMode(IdleMode.kCoast)  // Coast mode for less wear on the motors
            .inverted(false)
            .smartCurrentLimit(MAX_CURRENT)
            .openLoopRampRate(0.05);     // Add ramp rate to smooth acceleration
        
        // Still configure PID in case we need it later, but we're not using it now
        leftConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .velocityFF(kFF)
            .outputRange(-1, 1);
            
        leftMotor.configure(
            leftConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // Configure the right motor (follower)
        SparkMaxConfig rightConfig = new SparkMaxConfig();
        rightConfig
            .inverted(true)
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(MAX_CURRENT)
            .openLoopRampRate(0.05);     // Add ramp rate to smooth acceleration
            
        rightMotor.configure(
            rightConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // Initialize motors stopped
        stopMotors();
        
        Logger.log("Shooter subsystem initialized in " + currentState + " state");
    }

    /**
     * Prepare the shooter to intake a coral
     */
    public void prepareForIntake() {
        if (currentState == ShooterState.NO_CORAL) {
            Logger.log("Preparing shooter for intake");
            setMotorPower(INTAKE_POWER);
            currentState = ShooterState.READY_TO_INTAKE;
            stateTimer.reset();
            stateTimer.start();
        }
    }

    /**
     * Shoot the coral if one is inside the shooter
     */
    public void shootCoral() {
        if (currentState == ShooterState.CORAL_INSIDE) {
            Logger.log("Shooting coral");
            setMotorPower(SHOOTING_POWER);
            currentState = ShooterState.SHOOT_CORAL;
            stateTimer.reset();
            stateTimer.start();
        } else {
            Logger.log("Cannot shoot - no coral inside shooter");
        }
    }

    /**
     * Fine-tune the shooter intake at a slower speed
     * Used when coral doesn't go in fully and needs adjustment
     */
    public void fineTuneIntake(double FINE_TUNE_POWER) {
        Logger.log("Fine tuning shooter intake");
        setMotorPower(FINE_TUNE_POWER);
        // Don't change the state - this can be called from multiple states
    }
    
    public void shootHighestLevelCoral() {
        if (currentState == ShooterState.CORAL_INSIDE) {
            Logger.log("Shooting coral to bottom level");
            leftMotor.set(L4_SHOOTING_POWER);
            rightMotor.set(L4_SHOOTING_POWER);
            stateTimer.reset();
            stateTimer.start();
        } else {
            Logger.log("Cannot shoot - no coral inside shooter");
        }
    }
    public void shootBottomLevelCoral() {
        if (currentState == ShooterState.CORAL_INSIDE) {
            Logger.log("Shooting coral to bottom level");
            leftMotor.set(SHOOTING_POWER - 0.1);
            rightMotor.set(SHOOTING_POWER + 0.3);
            stateTimer.reset();
            stateTimer.start();
        } else {
            Logger.log("Cannot shoot - no coral inside shooter");
        }
    }
    /**
     * Emergency stop for the shooter
     */
    public void emergencyStop() {
        stopMotors();
        currentState = ShooterState.NO_CORAL;
        Logger.log("Emergency stop triggered - shooter reset to NO_CORAL state");
    }

    /**
     * Set the motor power using open-loop control
     * @param percentOutput Target percentage output (-1.0 to 1.0)
     */
    private void setMotorPower(double percentOutput) {
        Logger.log("Setting shooter power to " + percentOutput);
        leftMotor.set(percentOutput);
        rightMotor.set(percentOutput);
        // Right motor follows left motor automatically
    }

    /**
     * Stop the shooter motors
     */
    private void stopMotors() {
        Logger.log("Stopping shooter motors");
        leftMotor.stopMotor();
        rightMotor.stopMotor();
        // Right motor follows left motor, so no need to stop it separately
    }

    /**
     * Public method to stop the motors
     * Can be called from commands
     */
    public void stop() {
        stopMotors();
    }

    /**
     * Checks if a game piece has entered the shooter using the color sensor's proximity reading
     * @return true if a game piece is detected at the entry of the shooter, false if no sensor
     */
    public boolean hasGamePieceEntered() {
        if (!hasColorSensor) {
            return false;
        }

        int proximity = colorSensor.getProximity();
        boolean isClose = proximity > PROXIMITY_THRESHOLD;
        
        // If we detect a sudden increase in proximity, a game piece likely entered
        if (isClose && lastProximity <= PROXIMITY_THRESHOLD) {
            var detectedColor = colorSensor.getColor();
            Logger.log(String.format("Coral detected! Color: R=%.2f, G=%.2f, B=%.2f, Proximity=%d",
                detectedColor.red, detectedColor.green, detectedColor.blue, proximity));
        }
        
        lastProximity = proximity;
        return isClose;
    }

    /**
     * Checks if a game piece has exited the shooter by checking if proximity drops after being high
     * @return true if a game piece is detected leaving the shooter, false if no sensor
     */
    public boolean hasGamePieceExited() {
        if (!hasColorSensor) {
            return false;
        }

        int proximity = colorSensor.getProximity();
        boolean hasExited = lastProximity > PROXIMITY_THRESHOLD && proximity <= PROXIMITY_THRESHOLD;
        Logger.log("proxity sensor value = " + proximity + " hasExited = " + hasExited);
        if (hasExited) {
            Logger.log("Coral has exited the shooter");
        }
        
        lastProximity = proximity;
        return hasExited;
    }

    /**
     * Get the current state of the shooter
     * @return Current shooter state
     */
    public ShooterState getState() {
        return currentState;
    }

    @Override
    public void periodic() {
        // Handle state transitions based on current state
        switch (currentState) {
            case READY_TO_INTAKE:
                // Check if coral has entered shooter
                if (hasGamePieceEntered() || stateTimer.get() > INTAKE_TIMEOUT) {
                    Logger.log("Coral detected or timeout reached - stopping motors");
                    stopMotors();
                    currentState = ShooterState.CORAL_INSIDE;
                    stateTimer.stop();
                }
                break;
                
            case SHOOT_CORAL:
                // Check if shooting time is complete
                if (stateTimer.get() >= SHOOT_DURATION) {
                    Logger.log("Shooting complete - stopping motors");
                    stopMotors();
                    currentState = ShooterState.NO_CORAL;
                    stateTimer.stop();
                }
                break;
                
            case CORAL_INSIDE:
                // Just waiting for shoot command
                break;
                
            case NO_CORAL:
                // Just waiting for prepare command
                break;
        }
        
        // Update telemetry
        if (periodicCounter++ % 50 == 0) {
            updateTelemetry();
        }
    }

    private void updateTelemetry() {
        // Consolidate motor telemetry into Logger.log statements
        Logger.log("Shooter Left Motor - Velocity: " + leftMotor.getEncoder().getVelocity() + 
                  ", Current: " + leftMotor.getOutputCurrent() + 
                  ", Voltage: " + leftMotor.getBusVoltage() + 
                  ", Power: " + leftMotor.get());
        
        Logger.log("Shooter Right Motor - Velocity: " + rightMotor.getEncoder().getVelocity() + 
                  ", Current: " + rightMotor.getOutputCurrent() + 
                  ", Voltage: " + rightMotor.getBusVoltage() + 
                  ", Power: " + rightMotor.get());

        // State telemetry
        Logger.log("Shooter State: " + currentState.toString() + 
                  ", State Timer: " + stateTimer.get() + 
                  ", Target Power: " + (currentState == ShooterState.SHOOT_CORAL ? SHOOTING_POWER : 
                                       currentState == ShooterState.READY_TO_INTAKE ? INTAKE_POWER : 0));

        // Color sensor telemetry (only if sensor is present)
        if (hasColorSensor) {
            var color = colorSensor.getColor();
            int proximity = colorSensor.getProximity();
            Logger.log("Shooter Color Sensor - R: " + color.red + 
                      ", G: " + color.green + 
                      ", B: " + color.blue + 
                      ", Proximity: " + proximity);
        }
        Logger.log("Coral Present: " + (currentState == ShooterState.CORAL_INSIDE));
    }
}
