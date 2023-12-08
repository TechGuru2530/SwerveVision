package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.LimelightConstants;
import frc.robot.limelight.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import static edu.wpi.first.math.MathUtil.clamp;

public class ChaseAprilTagCommand extends CommandBase {
    
    private final SwerveSubsystem swerveSubsystem;
    private final ShuffleboardLayout visionLayout;

    private final PIDController pidControllerX = new PIDController(AutoConstants.X_kP, AutoConstants.X_kI, AutoConstants.X_kD);
    private final PIDController pidControllerY = new PIDController(AutoConstants.Y_kP, AutoConstants.Y_kI, AutoConstants.Y_kD);
    private final PIDController pidControllerOmega = new PIDController(AutoConstants.THETA_kP, AutoConstants.PATH_THETA_kI, AutoConstants.THETA_kD);

    public ChaseAprilTagCommand(
        SwerveSubsystem swerveSubsystem,
        ShuffleboardLayout visionLayout) 
    {
        this.swerveSubsystem = swerveSubsystem;
        this.visionLayout = visionLayout;

        addRequirements(swerveSubsystem);
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
    try{
      ChassisSpeeds speeds;
      // If the target is visible, get the new translation. If the target isn't visible we'll use the last known translation.
      LimelightHelpers.LimelightResults results = LimelightHelpers.getLatestResults((LimelightConstants.limeLightName));
     //System.out.println("I am inside april tag command execute");
      if(results.targetingResults.targets_Fiducials.length > 0){
        Pose3d pose = results.targetingResults.targets_Fiducials[0].getRobotPose_TargetSpace();
        //visionLayout.add("Target X", pose.getX());
        //visionLayout.add("Target Y", pose.getY());
        //visionLayout.add("Target Z", pose.getZ());
        
        var xSpeed = pidControllerX.calculate(pose.getX());
        if (pidControllerX.atSetpoint()) {
          xSpeed = 0;
        }
  
           // Handle alignment side-to-side
        var ySpeed = pidControllerY.calculate(pose.getY());
        if (pidControllerY.atSetpoint()) {
           ySpeed = 0;
        }
  
        // Handle rotation using target Yaw/Z rotation
        var omegaSpeed = pidControllerOmega.calculate(pose.getRotation().getZ());
        if (pidControllerOmega.atSetpoint()) {
           omegaSpeed = 0;
        }
  
          speeds = new ChassisSpeeds(xSpeed, ySpeed, -omegaSpeed);
        
        SwerveModuleState[] calculatedModuleStates = DriveConstants.KINEMATICS.toSwerveModuleStates(speeds);
        swerveSubsystem.setModules(calculatedModuleStates);
    }
    }
    catch(Exception e){
      //visionLayout.add("April tag error", e.getMessage());
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
    swerveSubsystem.stopDrive();
  }
  
}
