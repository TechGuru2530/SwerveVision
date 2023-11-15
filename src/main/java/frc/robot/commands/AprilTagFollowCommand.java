package frc.robot.commands;
import static edu.wpi.first.math.MathUtil.clamp;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DrivetrainConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.limelight.LimelightFiducial;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AprilTagFollowCommand extends CommandBase {

    private static final int TAG_TO_CHASE = 1;
    
    private final DrivetrainSubsystem drivetrainSubsystem;
    private final LimelightSubsystem limelightSubsystem;
    private final ShuffleboardLayout visionLayout;
    private LimelightFiducial lastTarget;

    private final PIDController pidControllerX = new PIDController(1, 0, 0);
    private final PIDController pidControllerY = new PIDController(1.5, 0, 0);
    private final PIDController pidControllerOmega = new PIDController(.5, 0, 0);

    public AprilTagFollowCommand(
        DrivetrainSubsystem drivetrainSubsystem,
        LimelightSubsystem limelightSubsystems,
        ShuffleboardLayout visionLayout) 
    {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.limelightSubsystem = limelightSubsystems;
        this.visionLayout = visionLayout;

        addRequirements(drivetrainSubsystem);
  }

  @Override
  public void initialize() {
    super.initialize();
    pidControllerX.reset();
    pidControllerY.reset();
    pidControllerOmega.reset();

    pidControllerX.setSetpoint(Units.inchesToMeters(36)); // Move forward/backwork to keep 36 inches from the target
    pidControllerX.setTolerance(Units.inchesToMeters(2.5));

    pidControllerY.setSetpoint(0); // Move side to side to keep target centered
    pidControllerY.setTolerance(Units.inchesToMeters(2.5));

    pidControllerOmega.setSetpoint(Units.degreesToRadians(180)); // Rotate the keep perpendicular with the target
    pidControllerOmega.setTolerance(Units.degreesToRadians(1));

  }

  //Need help here does not know how to move robot using limelight co-ordinates
  @Override
  public void execute() {
    // If the target is visible, get the new translation. If the target isn't visible we'll use the last known translation.
    var firstTarget = limelightSubsystem.getLatestFiducialTarget(LimelightConstants.limeLightName, "16H5C", TAG_TO_CHASE);
    //System.out.println("I am inside april tag command execute");
    if(firstTarget.isPresent()){
      // This is new target data, so recalculate the goal
      lastTarget = firstTarget.get();
      visionLayout.add("Target X", lastTarget.targetXPixels);
      visionLayout.add("Target Y", lastTarget.targetXPixels);
      visionLayout.add("Target Rotation X", lastTarget.targetXDegrees);
      visionLayout.add("Target Rotation Y", lastTarget.targetYDegrees);
      double targetOffsetAngle_Vertical = lastTarget.targetYDegrees;
      double angleToGoalDegrees = LimelightConstants.limelightMountAngleDegrees + targetOffsetAngle_Vertical; //(a2)
      double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0); //
      double distanceFromLimelightToGoalInches = (LimelightConstants.aprilTagGoalHeightInches - LimelightConstants.limelightLensHeightInches)/Math.tan(angleToGoalRadians);
      
      var xSpeed = pidControllerX.calculate(distanceFromLimelightToGoalInches);
      if (pidControllerX.atSetpoint()) {
        xSpeed = 0;
      }

         // Handle alignment side-to-side
      var ySpeed = pidControllerY.calculate(lastTarget.targetYDegrees);
      if (pidControllerY.atSetpoint()) {
        ySpeed = 0;
      }

      // Handle rotation using target Yaw/Z rotation
      var omegaSpeed = pidControllerOmega.calculate(angleToGoalRadians);
      if (pidControllerOmega.atSetpoint()) {
        omegaSpeed = 0;
      }
      
      SwerveModuleState[] calculatedModuleStates = DrivetrainConstants.KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, omegaSpeed));
      drivetrainSubsystem.setModules(calculatedModuleStates);
    }
  }

  public Translation2d DeadBand(Translation2d input, double deadzone) {
    double mag = input.getNorm();
    Translation2d norm = input.div(mag);

    if (mag < deadzone) {
        return new Translation2d(0.0, 0.0);
    } else {
        // TODO: Check is it sqrt2 or 1.0...
        Translation2d result = norm.times((mag - deadzone) / (1.0 - deadzone));
        return new Translation2d(
                clamp(result.getX(), -1.0, 1.0),
                clamp(result.getY(), -1.0, 1.0));
    }
}

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.stopDrive();
  }
    
}