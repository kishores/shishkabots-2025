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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
    private static double INTAKE_TIMEOUT = 1.2; // seconds to wait for coral to be fully inside

    // Motor configuration constants
    private static final double SHOOTING_POWER = 0.35; // 10% power for shooting
    private static final double INTAKE_POWER = 0.45;   // 20% power for intake
    private static final int MAX_CURRENT = 40; // Amps
    
    // Keep these for reference but they're not used with open-loop control
    private static final double kP = 1.3;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kFF = 0.000175;
    
    private static final double SHOOT_DURATION = 2.0; // seconds

    public ShooterSubsystem(int leftMotorCanId, int rightMotorCanId) {
        leftMotor = new SparkMax(leftMotorCanId, MotorType.kBrushless);
        rightMotor = new SparkMax(rightMotorCanId, MotorType.kBrushless);

        // Try to initialize color sensor on the I2C port
        try {
            colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
            hasColorSensor = true;
            System.out.println("Color sensor initialized successfully");
            // give the intake more time to run if we have a color sensor since it will automatically turn off
            INTAKE_TIMEOUT = 3.5;
        } catch (Exception e) {
            System.out.println("Color sensor not detected, running without game piece detection");
            hasColorSensor = false;
        }

        // Configure the left motor (leader)
        SparkMaxConfig leftConfig = new SparkMaxConfig();
        leftConfig
            .idleMode(IdleMode.kCoast)  // Coast mode for less wear on the motors
            .inverted(false)
            .smartCurrentLimit(MAX_CURRENT)
            .openLoopRampRate(0.1);     // Add ramp rate to smooth acceleration
        
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
            .openLoopRampRate(0.1);     // Add ramp rate to smooth acceleration
            
        rightMotor.configure(
            rightConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters
        );

        // Initialize motors stopped
        stopMotors();
        
        System.out.println("Shooter subsystem initialized in " + currentState + " state");
    }

    /**
     * Prepare the shooter to intake a coral
     */
    public void prepareForIntake() {
        if (currentState == ShooterState.NO_CORAL) {
            System.out.println("Preparing shooter for intake");
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
            System.out.println("Shooting coral");
            setMotorPower(SHOOTING_POWER);
            currentState = ShooterState.SHOOT_CORAL;
            stateTimer.reset();
            stateTimer.start();
        } else {
            System.out.println("Cannot shoot - no coral inside shooter");
        }
    }

    public void shootBottomLevelCoral() {
        if (currentState == ShooterState.CORAL_INSIDE) {
            System.out.println("Shooting coral to bottom level");
            leftMotor.set(SHOOTING_POWER);
            rightMotor.set(SHOOTING_POWER + 0.175);
            stateTimer.reset();
            stateTimer.start();
        } else {
            System.out.println("Cannot shoot - no coral inside shooter");
        }
    }
    /**
     * Emergency stop for the shooter
     */
    public void emergencyStop() {
        stopMotors();
        currentState = ShooterState.NO_CORAL;
        System.out.println("Emergency stop triggered - shooter reset to NO_CORAL state");
    }

    /**
     * Set the motor power using open-loop control
     * @param percentOutput Target percentage output (-1.0 to 1.0)
     */
    private void setMotorPower(double percentOutput) {
        System.out.println("Setting shooter power to " + percentOutput);
        leftMotor.set(percentOutput);
        rightMotor.set(percentOutput);
        // Right motor follows left motor automatically
    }

    /**
     * Stop the shooter motors
     */
    private void stopMotors() {
        System.out.println("Stopping shooter motors");
        leftMotor.stopMotor();
        rightMotor.stopMotor();
        // Right motor follows left motor, so no need to stop it separately
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
            System.out.println(String.format("Coral detected! Color: R=%.2f, G=%.2f, B=%.2f, Proximity=%d",
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
        
        if (hasExited) {
            System.out.println("Coral has exited the shooter");
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
                    System.out.println("Coral detected or timeout reached - stopping motors");
                    stopMotors();
                    currentState = ShooterState.CORAL_INSIDE;
                    stateTimer.stop();
                }
                break;
                
            case SHOOT_CORAL:
                // Check if shooting time is complete
                if (stateTimer.get() >= SHOOT_DURATION) {
                    System.out.println("Shooting complete - stopping motors");
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
        updateTelemetry();
    }

    private void updateTelemetry() {
        // Left motor telemetry
        SmartDashboard.putNumber("Shooter/Left/Velocity", leftMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/Left/Current", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Left/Voltage", leftMotor.getBusVoltage());
        SmartDashboard.putNumber("Shooter/Left/Power", leftMotor.get());
        
        // Right motor telemetry
        SmartDashboard.putNumber("Shooter/Right/Velocity", rightMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Shooter/Right/Current", rightMotor.getOutputCurrent());
        SmartDashboard.putNumber("Shooter/Right/Voltage", rightMotor.getBusVoltage());
        SmartDashboard.putNumber("Shooter/Right/Power", rightMotor.get());

        // State telemetry
        SmartDashboard.putString("Shooter/State", currentState.toString());
        SmartDashboard.putNumber("Shooter/StateTimer", stateTimer.get());
        
        // Add power output telemetry
        SmartDashboard.putNumber("Shooter/TargetPower", 
            currentState == ShooterState.SHOOT_CORAL ? SHOOTING_POWER : 
            currentState == ShooterState.READY_TO_INTAKE ? INTAKE_POWER : 0);

        // Color sensor telemetry (only if sensor is present)
        if (hasColorSensor) {
            var color = colorSensor.getColor();
            int proximity = colorSensor.getProximity();
            SmartDashboard.putNumber("Shooter/Sensor/Red", color.red);
            SmartDashboard.putNumber("Shooter/Sensor/Green", color.green);
            SmartDashboard.putNumber("Shooter/Sensor/Blue", color.blue);
            SmartDashboard.putNumber("Shooter/Sensor/Proximity", proximity);
        }
        SmartDashboard.putBoolean("Shooter/CoralPresent", currentState == ShooterState.CORAL_INSIDE);
    }
}
