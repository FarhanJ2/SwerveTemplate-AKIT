package org.steelhawks.subsystems.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import org.steelhawks.Constants;

import java.util.Queue;

/** IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {

    private final Pigeon2 pigeon = new Pigeon2(KSwerve.PIGEON_ID, Constants.CANIVORE_NAME);
    private final StatusSignal<Double> yaw = pigeon.getYaw();
    private final Queue<Double> yawPositionQueue;
    private final Queue<Double> yawTimestampQueue;
    private final StatusSignal<Double> yawVelocity = pigeon.getAngularVelocityZWorld();

    public GyroIOPigeon2(boolean phoenixDrive) {
        pigeon.getConfigurator().apply(new Pigeon2Configuration());
        pigeon.getConfigurator().setYaw(0.0);
        yaw.setUpdateFrequency(SwerveModule.ODOMETRY_FREQUENCY);
        yawVelocity.setUpdateFrequency(100.0);
        pigeon.optimizeBusUtilization();
        if (phoenixDrive) {
            yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
            yawPositionQueue =
                PhoenixOdometryThread.getInstance().registerSignal(pigeon, pigeon.getYaw());
        } else {
            yawTimestampQueue = null;
            yawPositionQueue = null;
        }
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
        inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

        inputs.odometryYawTimestamps =
            yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
        inputs.odometryYawPositions =
            yawPositionQueue.stream()
                .map(Rotation2d::fromDegrees)
                .toArray(Rotation2d[]::new);
        yawTimestampQueue.clear();
        yawPositionQueue.clear();
    }
}
