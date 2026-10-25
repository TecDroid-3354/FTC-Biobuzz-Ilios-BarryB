package org.firstinspires.ftc.teamcode.autonomous.pedroPathing.tuning;

import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.procedures.MecanumTuner;
import org.firstinspires.ftc.teamcode.autonomous.pedroPathing.procedures.PinpointTuner;
import org.firstinspires.ftc.teamcode.utils.autonomous.PedroPathing;

/**
 * Pedro Pathing 3.0 AutoTune procedures.
 * <p>
 * Deploy this code, connect to the robot's Wi-Fi and open http://192.168.43.1:10158 in a browser to run them
 * (see https://pedropathing.com/docs/pathing/tuning). Paste the configs that AutoTune generates into
 * {@link Constants}.
 * <p>
 * The tuners below assume a mecanum drivetrain with a goBILDA Pinpoint. If your robot differs, swap in the
 * matching tuner and localizer/drivetrain from the procedures package (e.g. {@code OTOSTuner},
 * {@code ThreeWheelTuner}, {@code TwoWheelTuner}, {@code OctoQuadTuner}).
 */
public class Tuning {
    @Tuner
    public static Procedure mecanumTuner() {
        return new MecanumTuner();
    }

    @Tuner
    public static Procedure pinpointTuner() {
        return new PinpointTuner();
    }

    @Tuner
    public static Procedure foresightTuner() {
        return PedroPathing.INSTANCE.createForesightTunerWithPinpoint(Constants.drivetrainConfig, Constants.localizerConfig);
    }

    @Tuner
    public static Procedure tests() {
        return PedroPathing.INSTANCE.createFullMecanumTestWithPinpoint(Constants.drivetrainConfig, Constants.localizerConfig, Constants.foresightConfig);
    }
}
