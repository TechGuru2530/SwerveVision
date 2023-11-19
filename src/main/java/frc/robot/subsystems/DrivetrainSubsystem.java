package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.swerve.SwerveModule;

public class DrivetrainSubsystem extends SubsystemBase {
    private final SwerveModule frontLeft = new SwerveModule(DrivetrainConstants.FL_STEER_ID, DrivetrainConstants.FL_DRIVE_ID,
            DrivetrainConstants.FL_ABSOLUTE_ENCODER_PORT, DrivetrainConstants.FL_OFFSET_ROTATIONS,
            DrivetrainConstants.FL_ABSOLUTE_ENCODER_REVERSED,
            DrivetrainConstants.FL_DRIVE_MOTOR_REVERSED, DrivetrainConstants.FL_STEER_MOTOR_REVERSED);

    private final SwerveModule frontRight = new SwerveModule(DrivetrainConstants.FR_STEER_ID, DrivetrainConstants.FR_DRIVE_ID,
            DrivetrainConstants.FR_ABSOLUTE_ENCODER_PORT, DrivetrainConstants.FR_OFFSET_ROTATIONS,
            DrivetrainConstants.FR_ABSOLUTE_ENCODER_REVERSED,
            DrivetrainConstants.FR_DRIVE_MOTOR_REVERSED, DrivetrainConstants.FR_STEER_MOTOR_REVERSED);

    private final SwerveModule backRight = new SwerveModule(DrivetrainConstants.BR_STEER_ID, DrivetrainConstants.BR_DRIVE_ID,
            DrivetrainConstants.BR_ABSOLUTE_ENCODER_PORT, DrivetrainConstants.BR_OFFSET_ROTATIONS,
            DrivetrainConstants.BR_ABSOLUTE_ENCODER_REVERSED,
            DrivetrainConstants.BR_DRIVE_MOTOR_REVERSED, DrivetrainConstants.BR_STEER_MOTOR_REVERSED);

    private final SwerveModule backLeft = new SwerveModule(DrivetrainConstants.BL_STEER_ID, DrivetrainConstants.BL_DRIVE_ID,
            DrivetrainConstants.BL_ABSOLUTE_ENCODER_PORT, DrivetrainConstants.BL_OFFSET_ROTATIONS,
            DrivetrainConstants.BL_ABSOLUTE_ENCODER_REVERSED,
            DrivetrainConstants.BL_DRIVE_MOTOR_REVERSED, DrivetrainConstants.BL_STEER_MOTOR_REVERSED);

    private final AHRS navX = new AHRS(SPI.Port.kMXP);

    private final Field2d field = new Field2d();

    private final SwerveDriveOdometry odometry = new SwerveDriveOdometry(DrivetrainConstants.KINEMATICS, getRotation2d(),
            getModulePositions());

    public DrivetrainSubsystem() {
        zeroHeading();
        // new Thread(() -> {
        // try {
        // Thread.sleep(1000);
        // zeroHeading();
        // } catch (Exception e) {
        // // TODO: handle exception
        // }
        // }).start();
    }

    @Override
    public void periodic() {
        double rads = getPose().getRotation().getRadians(); 
        odometry.update(getRotation2d(), getModulePositions());
        field.setRobotPose(getPose());
        SmartDashboard.putData("Field", field);

        SmartDashboard.putString("Robot Pose",
                getPose().toString());

        SmartDashboard.putNumber("Spin Velocity", (getPose().getRotation().getRadians() - rads) / 0.02);
    }

    public void zeroHeading() {
        // Reset pose to the same translation but rotation of 0
        var pose = odometry.getPoseMeters();
        var newPose = new Pose2d(pose.getTranslation(), new Rotation2d());
        navX.reset();
        resetOdometry(newPose);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(getRotation2d(), getModulePositions(), pose);
    }

    /**
     * Gets the robot heading.
     * @return robot heading in radians [-Pi, Pi) with counter-clockwise positive
     */
    public double getHeading() {
        return -Units.degreesToRadians(Math.IEEEremainder(navX.getAngle(), 360d));
    }

    /**
     * Gets the robot heading.
     * @return robot heading in radians [-Pi, Pi) with counter-clockwise positive
     */
    public Rotation2d getRotation2d() {
        return new Rotation2d(getHeading());
    }

    public void stopDrive() {
        frontLeft.stop();
        frontRight.stop();
        backLeft.stop();
        backRight.stop();
    }

    public void setModules(SwerveModuleState[] states) {
        // Normalize speeds so they are all obtainable
        SwerveDriveKinematics.desaturateWheelSpeeds(states, DrivetrainConstants.MAX_MODULE_VELOCITY);
        backRight.setModuleState(states[0]);
        frontRight.setModuleState(states[1]);
        backLeft.setModuleState(states[2]);
        frontLeft.setModuleState(states[3]);
    }

    public void setXstance() {
        frontLeft.setModuleStateRaw(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setModuleStateRaw(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setModuleStateRaw(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setModuleStateRaw(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    public ChassisSpeeds getChassisSpeeds() {
        ChassisSpeeds speeds = DrivetrainConstants.KINEMATICS.toChassisSpeeds(
            backRight.getModuleState(),
            frontRight.getModuleState(),
            backLeft.getModuleState(),
            frontLeft.getModuleState());

        return speeds;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        states[0] = backRight.getModulePosition();
        states[1] = frontRight.getModulePosition();
        states[2] = backLeft.getModulePosition();
        states[3] = frontLeft.getModulePosition();

        return states;
    }
}
