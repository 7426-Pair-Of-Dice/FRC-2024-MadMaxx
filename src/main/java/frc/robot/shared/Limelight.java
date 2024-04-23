package frc.robot.shared;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.SwerveConstants;

public class Limelight {
    private static final String m_primary = "limelight-april"; // The primary and only limelight on the final robot.
    private static final physicalConfig m_primaryConfig = new physicalConfig(34.8, 7.5, 57.0);

    public static shootCalculation calculation = new shootCalculation(70, 20);
    
    public static void periodic() {
        // Ensure we are running on the right pipeline
        LimelightHelpers.setPipelineIndex(m_primary, 0);

        // Only calculate a shot if a tag is seen.
        // Helpful if the limelight is loosely calibrated
        // or the limelight is blocked during a shot for any reason.
        if(LimelightHelpers.getTV(m_primary)) calculation = calculateShot();
        
        if(Constants.Dashboard.kSendDebug) {
            SmartDashboard.putNumber("Primary Limelight Distance", getDistance());
            SmartDashboard.putNumber("Primary Limelight Angle", calculation.angle());
        }
    }

    /**
     * Calculates the distance from the primary limelight and an april tag target.
     * This code is a modified version of the examples found in the 
     * <a href="https://docs.limelightvision.io/docs/docs-limelight/tutorials/tutorial-estimating-distance">
     * official Limelight Documentation
     * </a>.
     * @return Distance in inches.
     */
    public static double getDistance() {
        double ty = LimelightHelpers.getTY(m_primary);

        double angleToGoalDegrees = m_primaryConfig.angle() + ty;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        // Return calculated Distance
        return (m_primaryConfig.tagHeight() - m_primaryConfig.lensHeight()) / Math.tan(angleToGoalRadians);
    }

    /**
     * Calculates the arm angle by plugging the calculated distance into a piecewise
     * function. This function was created by manually measuring the required arm angle 
     * at different distances and plotting the line of best fit.
     * @return The calculated shooter RPS and arm position in degrees.
     */
    public static shootCalculation calculateShot() {
        double rps = 70.0; // Was planned to calculate rps, didn't seem necessary.
        double distance = getDistance();
        double angle;

        if(distance < 112.0) { // Calculating for distances past 112 inches
            angle = -0.0864 + (0.622 * distance) + (-2.22E-03 * Math.pow(distance, 2));
        } else { // Calculating for distances less than 112 inches
            angle = 10.7 + (0.378 * distance) - (8.64E-4 * Math.pow(distance, 2));
        }

        return new shootCalculation(rps, angle);
    }

    /**
     * Calculates the required rate of rotation needed to center onto
     * an april tag target. Needs to be further tuned on future robots.
     * @return Angular rate to rotate at, in radians per second.
     */
    public static double calculateCentering() {
        double kP = 0.015;
        return LimelightHelpers.getTX(m_primary) 
                * kP * -SwerveConstants.kMaxAngularRate;
    }

    /**
     * The calculated shooter RPS and arm angle required to make a shot into the speaker.
     * @param rps Target shooter velocity in rotations per second.
     * @param angle Target arm position in degrees.
     */
    public record shootCalculation(double rps, double angle) {};

    /**
     * The physical configuration of a limelight's placement on a robot.
     * @param angle The angle that the limelight is mounted at, in degrees.
     * @param lensHeight The distance from the floor to the center of the limelight lens, in inches.
     * @param tagHeight The height of the april tags we are looking to detect, in inches.
     */
    public record physicalConfig(double angle, double lensHeight, double tagHeight) {};
}
