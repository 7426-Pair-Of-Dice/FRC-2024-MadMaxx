package frc.robot.shared;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.SwerveConstants;

public class Limelight {
    private static final String m_primary = "limelight-april";

    public static shootCalculation calculation = new shootCalculation(70, 20);
    
    public static void periodic() {
        // If a tag is seen, calculate our shot.
        LimelightHelpers.setPipelineIndex(m_primary, 0);
        if(LimelightHelpers.getTV(m_primary)) {
            calculation = calculateShot();
        }
    }

    public static double getDistance() {
        var ty = LimelightHelpers.getTY(m_primary);

        double limelightMountAngleDegrees = 34.8; 

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
        if(distance < 112.0){
            angle = -0.0864 + (0.622 * distance) + (-2.22E-03 * Math.pow(distance, 2));
        }
        else{
            angle = 10.7 + (0.378 * distance) - (8.64E-4 * Math.pow(distance, 2));
        }

        SmartDashboard.putNumber("Auto Limelight Angle", angle);

        return new shootCalculation(rps, angle);
    }

    public static double calculateCentering() {
        double kP = 0.015;
        return LimelightHelpers.getTX("limelight-april") 
                * kP 
                * SwerveConstants.kMaxAngularRate 
                * -1.0;
    }

    public record shootCalculation(double rps, double angle) {};
}
