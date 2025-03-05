package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.RelativeEncoder;
import com.revrobotics.sim.SparkMaxSim;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.system.plant.DCMotor;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax elevatorMotor;
    private final SparkMax elevatorMotor2;
    private final RelativeEncoder encoder;
    private final SparkClosedLoopController closedLoopController;
    private final DigitalInput topLimitSwitch;
    private final DigitalInput bottomLimitSwitch;

    // Simulation
    private final DCMotor elevatorDCMotor;
    private final SparkMaxSim elevatorMotorSim;

    // Constants
    private static final double MAX_OUTPUT = 1.0;
    private static final double MIN_OUTPUT = -1.0;
    private static final double TOLERANCE = 0.5;

    // Elevator Position Constants (in encoder units)
    private static final double BOTTOM_THRESHOLD = 0.0;
    private static final double TOP_THRESHOLD = 100.0;

    // PID Constants
    private static final double kP = 0.1;
    private static final double kI = 0.0;
    private static final double kD = 0.0;
    private static final double kFF = 0.0;
    
    private double targetPosition = 0.0;

    public ElevatorSubsystem(int motorCanId1, int motorCanId2, int topLimitSwitchId, int bottomLimitSwitchId) {
        elevatorMotor = new SparkMax(motorCanId1, SparkMax.MotorType.kBrushless);
        elevatorMotor2 = new SparkMax(motorCanId2, SparkMax.MotorType.kBrushless);
        
        encoder = elevatorMotor.getEncoder();
        closedLoopController = elevatorMotor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(kP, kI, kD)
            .velocityFF(kFF)
            .outputRange(MIN_OUTPUT, MAX_OUTPUT);
        
        elevatorMotor.configure(config);
        elevatorMotor2.follow(elevatorMotor); // Ensure motor2 follows motor1

        topLimitSwitch = new DigitalInput(topLimitSwitchId);
        bottomLimitSwitch = new DigitalInput(bottomLimitSwitchId);

        encoder.setPosition(0);
        
        elevatorDCMotor = DCMotor.getNEO(1);
        elevatorMotorSim = new SparkMaxSim(elevatorMotor, elevatorDCMotor);
    }

    public void setTargetPosition(double position) {
        position = Math.max(BOTTOM_THRESHOLD, Math.min(position, TOP_THRESHOLD));
        targetPosition = position;
        closedLoopController.setReference(position, ControlType.kPosition);
    }

    public double getCurrentPosition() {
        return encoder.getPosition();
    }

    public void setSpeed(double speed) {
        if ((speed > 0 && isAtTop()) || (speed < 0 && isAtBottom())) {
            stop();
            return;
        }
        speed = Math.max(MIN_OUTPUT, Math.min(MAX_OUTPUT, speed));
        elevatorMotor.set(speed);
        elevatorMotor2.set(speed);
    }

    public boolean isAtTop() {
        return !topLimitSwitch.get();
    }

    public boolean isAtBottom() {
        return !bottomLimitSwitch.get();
    }

    public void stop() {
        elevatorMotor.stopMotor();
        elevatorMotor2.stopMotor();
    }

    @Override
    public void periodic() {
        if (isAtTop() && getCurrentPosition() > TOP_THRESHOLD) {
            stop();
        }
        if (isAtBottom() && getCurrentPosition() < BOTTOM_THRESHOLD) {
            stop();
        }
        updateTelemetry();
    }

    private void updateTelemetry() {
        SmartDashboard.putNumber("Elevator/CurrentPosition", getCurrentPosition());
        SmartDashboard.putNumber("Elevator/TargetPosition", targetPosition);
        SmartDashboard.putBoolean("Elevator/AtTop", isAtTop());
        SmartDashboard.putBoolean("Elevator/AtBottom", isAtBottom());
    }
}
