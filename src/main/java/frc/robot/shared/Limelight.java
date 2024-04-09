package frc.robot.shared;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.body.Arm;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.shared.LimelightHelpers;

public class Limelight {
    private static final String m_primary = "limelight-april";

    public static shootCalculation calculation = new shootCalculation(70, 20);
    
    public static void periodic() {
        // If a tag is seen, calculate our shot.
        LimelightHelpers.setPipelineIndex(m_primary, 2);
        if(LimelightHelpers.getTV(m_primary)) {
            calculation = calculateShot();
        }
    }

    public static double getDistance() {
        var ty = LimelightHelpers.getTY(m_primary);

        double limelightMountAngleDegrees = 24.8; 

        double limelightLensHeightInches = 7.5; 

        double goalHeightInches = 57.0; 

        double angleToGoalDegrees = limelightMountAngleDegrees + ty;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        SmartDashboard.putNumber("Auto Limelight Distance", (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians));

        //calculate distance
        return (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
    }

    public static shootCalculation calculateShot() {
        double rps = 70.0;
        double distance = getDistance();
        double angle;
        if(distance < 135.0){
            angle = -0.355 + 0.613 * distance - 2.05E-3 * Math.pow(distance, 2);
        }
        else{
            angle = 31.3 + 0.136 * distance - 2.21E-4 * Math.pow(distance, 2);
        }

        SmartDashboard.putNumber("Auto Limelight Angle", angle);

        return new shootCalculation(rps, angle);
    }

    public static double calculateCentering() {
        double kP = Arm.getInstance().m_temporaryRemove;
        return LimelightHelpers.getTX("limelight-april") 
                * kP 
                * SwerveConstants.kMaxAngularRate 
                * -1.0;
    }

    public record shootCalculation(double rps, double angle) {};
}
