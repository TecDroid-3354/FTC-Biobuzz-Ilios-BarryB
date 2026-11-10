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
            Optional.of(GoBildaPinpointDriver.EncoderDirection.REVERSED),
            Optional.of(GoBildaPinpointDriver.EncoderDirection.REVERSED),
            Optional.of(Distance.fromInches(7.095777256282296)),
            Optional.of(Distance.fromInches(-0.9259914788674183))
    );

    public static ForesightConfig foresightConfig = PedroPathing.INSTANCE.createForesightConfig(
            Optional.of(LinearVelocity.fromInps(73.70766256965302)),
            Optional.of(LinearVelocity.fromInps(50.95451912606664)),
            Optional.of(LinearVelocity.fromInps(66.2330040361729)),
            Optional.of(LinearVelocity.fromInps(98.1109111501078)),
            Optional.of(Vector2D.cartesian(0.052388621977635726, 0.007584251199523778)),
            Optional.of(Controller.proportional(4.899468912226798)),
            Optional.of(Matrix.diag(0.10488295618353324, 0.06963286700854306)),
            Optional.of(Matrix.diag(0.0015608973032158766, 0.0017147221775882472)),
            Optional.of(Controller.proportionalFeedforward(0.013414203480727744)),
            Optional.of(Controller.proportionalFeedforward(0.011402072958618581)),
            Optional.of(Controller.proportional(0.30204688451928496)),
            Optional.of(Controller.proportional(0.11159825298847863)),
            Optional.of(Controller.proportional(0.7294004144110938)),
            Optional.of(Controller.proportional(0.26949396318687474))
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new Follower(
                new PinpointLocalizer(hardwareMap, localizerConfig),
                new Mecanum(hardwareMap, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}
