package frc.robot.subsystems;

import com.ctre.phoenix.Util;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private WPI_VictorSPX actuatorMotor = new WPI_VictorSPX(IntakeConstants.ACTUATOR_MOTOR_PORT);
    private CANSparkMax leftIntake;
    private CANSparkMax rightIntake;

    private CANCoder actuatorEncoder = new CANCoder(IntakeConstants.ACTUATOR_ENCODER_PORT);

    private final PIDController actuatorPID = new PIDController(1.5, 0.0, 0.0);

    public static enum IntakeState {
        STOWED(-30),
        PICKUP(120),
        PLACE(60),
        HIGH(45);

        private double angleDegrees;

        IntakeState(double angleDegrees) {
            this.angleDegrees = angleDegrees;
        }
    }

    private IntakeState intakeState = IntakeState.STOWED;

    public Intake() {

        this.leftIntake = new CANSparkMax(IntakeConstants.WHEEL_LEFT_PORT, MotorType.kBrushless);
        this.rightIntake = new CANSparkMax(IntakeConstants.WHEEL_RIGHT_PORT, MotorType.kBrushless);

        leftIntake.setIdleMode(IdleMode.kBrake);
        rightIntake.setIdleMode(IdleMode.kBrake);

        leftIntake.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);
        rightIntake.setSmartCurrentLimit(IntakeConstants.INTAKE_CURRENT_LIMIT);
    }

    public IntakeState getIntakeState() {
        return this.intakeState;
    }

    @Override
    public void periodic() {
        double currentAngle = (Units.degreesToRadians(actuatorEncoder.getAbsolutePosition())
                - IntakeConstants.OFFSET_RADIANS) * IntakeConstants.ACTUATOR_GEAR_RATIO;

        actuatorMotor
                .set(Util.cap(actuatorPID.calculate(currentAngle, Units.degreesToRadians(intakeState.angleDegrees)),
                        IntakeConstants.MAX_SPEED));
        SmartDashboard.putNumber("Intake Current (A)", leftIntake.getOutputCurrent());
    }

    public void setIntakeSpeed(double speed) {
        leftIntake.set(speed);
        rightIntake.set(-speed);
    }

    public void stop() {
        leftIntake.set(0);
        rightIntake.set(0);
        actuatorMotor.set(0);
    }

    public void setIntakeState(IntakeState state) {
        this.intakeState = state;
    }
}
