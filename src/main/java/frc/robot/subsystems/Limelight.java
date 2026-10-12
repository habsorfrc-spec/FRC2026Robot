
package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import frc.robot.LimelightHelpers;

public class Limelight {

    private static final String LIMELIGHT_NAME = "limelight";

    // Example calibration values. Replace these with tested values.
    private static final double[] DISTANCES = {
        1.5, 2.5, 3.5, 4.5
    };

    private static final double[] RPMS = {
        2500, 3000, 3500, 4000
    };

    /**
     * Returns the straight-line distance from the camera
     * to the target in meters.
     *
     * Returns -1 if no target is detected or the measurement
     * is invalid.
     */
    public double getDistanceMeters() {

        if (!LimelightHelpers.getTV(LIMELIGHT_NAME)) {
            return -1.0;
        }

        Pose3d targetPose =
            LimelightHelpers.getTargetPose3d_CameraSpace(
                LIMELIGHT_NAME
            );

        if (targetPose == null) {
            return -1.0;
        }

        double x = targetPose.getX();
        double y = targetPose.getY();
        double z = targetPose.getZ();

        double distance = Math.sqrt(
            x * x + y * y + z * z
        );

        if (!Double.isFinite(distance) || distance <= 0.0) {
            return -1.0;
        }

        return distance;
    }

    /**
     * Returns the interpolated shooter RPM for a distance.
     * Returns -1 if the distance is invalid.
     */
    public double getShooterRPM(double distanceMeters) {

        if (!Double.isFinite(distanceMeters)
                || distanceMeters <= 0.0) {
            return -1.0;
        }

        // Clamp to the tested range
        if (distanceMeters <= DISTANCES[0]) {
            return RPMS[0];
        }

        int last = DISTANCES.length - 1;

        if (distanceMeters >= DISTANCES[last]) {
            return RPMS[last];
        }

        // Interpolate between the two nearest points
        for (int i = 0; i < last; i++) {

            if (distanceMeters <= DISTANCES[i + 1]) {

                double ratio =
                    (distanceMeters - DISTANCES[i])
                    / (DISTANCES[i + 1] - DISTANCES[i]);

                return RPMS[i]
                    + ratio * (RPMS[i + 1] - RPMS[i]);
            }
        }

        return -1.0;
    }

    /**
     * Convenience method: get the current distance and
     * calculate the corresponding RPM.
     */
    public double getCalculatedShooterRPM() {
        double distance = getDistanceMeters();

        if (distance <= 0.0) {
            return -1.0;
        }

        return getShooterRPM(distance);
    }
}
