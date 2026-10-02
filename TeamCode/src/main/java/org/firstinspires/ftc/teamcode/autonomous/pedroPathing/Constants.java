package org.firstinspires.ftc.teamcode.autonomous.pedroPathing;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.subsystems.mecanum.MecanumConstants;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.tuning.Tuning;
import org.firstinspires.ftc.teamcode.utils.autonomous.PedroPathing;
import org.firstinspires.ftc.teamcode.utils.units.Distance;
import org.firstinspires.ftc.teamcode.utils.units.LinearVelocity;

import java.util.Optional;

/**
 * Pedro Pathing 3.0 constants. The values below are placeholders: run the AutoTune procedures in
 * {@link Tuning} (see https://pedropathing.com/docs/pathing/tuning) and paste the generated configs here.
 * If you use a different drivetrain or localizer, swap the config classes and {@link #createFollower} to match.
 */
public class Constants {

    public static MecanumConfig drivetrainConfig = PedroPathing.INSTANCE.createMecanumConfig(
            Optional.of(DcMotorSimple.Direction.FORWARD),
            Optional.of(DcMotorSimple.Direction.REVERSE),
            Optional.of(DcMotorSimple.Direction.FORWARD),
            Optional.of(DcMotorSimple.Direction.REVERSE),
            Optional.of(MecanumConstants.Control.IS_BRAKE_MODE),
            Optional.of(0.01)
    );

    public static PinpointConfig localizerConfig = PedroPathing.INSTANCE.createPinpointConfig(
            Optional.of(GoBildaPinpointDriver.EncoderDirection.FORWARD),
            Optional.of(GoBildaPinpointDriver.EncoderDirection.FORWARD),
            Optional.of(Distance.fromInches(3.225225613811823)),
            Optional.of(Distance.fromInches(-6.087658499169537))
    );


    public static ForesightConfig foresightConfig = PedroPathing.INSTANCE.createForesightConfig(
            Optional.of(LinearVelocity.fromInps(76.35614907285614)),
            Optional.of(LinearVelocity.fromInps(56.70816675683559)),
            Optional.of(LinearVelocity.fromInps(31.695582816741506)),
            Optional.of(LinearVelocity.fromInps(70.7024960555782)),
            Optional.of(Vector2D.cartesian(0.04122564769086424, 0.012900349474236858)),
            Optional.of(Controller.proportional(4.476963220918328)),
            Optional.of(Matrix.diag(0.09080938797590168, 0.04076852583434593)),
            Optional.of(Matrix.diag(0.0012044698822213874, 0.0022763626170223357)),
            Optional.of(Controller.proportionalFeedforward(0.0132022240880816)),
            Optional.of(Controller.proportionalFeedforward(0.011221890474869359)),
            Optional.of(Controller.proportional(0.19681124258833685)),
            Optional.of(Controller.proportional(0.07271649524313407)),
            Optional.of(Controller.proportional(0.42481893385615904)),
            Optional.of(Controller.proportional(0.15695924468887693))
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
