package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    // From the Mecanum tuner, matches subsystems/Drive.java
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    // No odometry pods, so position comes from the drive motor encoders.
    // Measured with the "Drive Encoder Test" OpMode
    public static double forwardTicksPerInch = 29.95;
    public static double strafeTicksPerInch = 26.48;

    // From the Foresight tuner
    public static ForesightConfig foresightConfig = new ForesightConfig(c -> {
        Controller primaryTranslationalForward = Controller.proportional(0.3532247808118508);
        Controller secondaryTranslationalForward = Controller.proportional(0.13050711817000718);
        Controller primaryTranslationalLateral = Controller.proportional(0.6427858607286897);
        Controller secondaryTranslationalLateral = Controller.proportional(0.23749219998472543);

        c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
        c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

        c.coast.set(Controller.proportionalFeedforward(0.013369799290572826));
        c.brake.set(Controller.proportionalFeedforward(0.011364329396986901));

        c.headingFeedback.set(Controller.proportional(5.754073560133086));
        c.headingBrakeCoefficients.set(Vector2D.cartesian(0.08123928909331031, 0.0016808237330948163));

        c.linearBrakeCoefficients.set(Matrix.diag(0.06237012791477002, 0.011977613930804336));
        c.quadraticBrakeCoefficients.set(Matrix.diag(3.114465368679837E-4, 6.986120030080262E-4));

        c.maxAchievableForwardVelocity.set(78.93028777566191);
        c.maxAchievableStrafeVelocity.set(76.11490891275665);
        c.naturalForwardDeceleration.set(106.6363590436436);
        c.naturalStrafeDeceleration.set(427.4586251925163);
    });

    public static Localizer localizer(HardwareMap h) {
        return new DriveEncoderLocalizer(h, drivetrainConfig, "expander", forwardTicksPerInch, strafeTicksPerInch);
    }

    public static Drivetrain drivetrain(HardwareMap h) {
        return new Mecanum(h, drivetrainConfig);
    }

    public static Follower create(HardwareMap h) {
        return new Follower(localizer(h), drivetrain(h), new Foresight(foresightConfig));
    }
}
