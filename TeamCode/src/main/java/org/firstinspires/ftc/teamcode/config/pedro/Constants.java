package org.firstinspires.ftc.teamcode.config.pedro;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.localizers.OTOSLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.qualcomm.hardware.dfrobot.HuskyLens;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.2 )
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.3, 0.1211,0.001701+1))
            .forwardZeroPowerAcceleration(-56.232924651374766)
            .lateralZeroPowerAcceleration(-93.23257871730974);

    public static MecanumConstants mecanumConstants = new MecanumConstants()
            .useBrakeModeInTeleOp(true)
            .xVelocity(55.04269817682702)
            .yVelocity(36.01743058332076)


            .leftFrontMotorName("lF")
            .leftRearMotorName("lR")
            .rightFrontMotorName("rF")
            .rightRearMotorName("rR")

            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static PinpointConstants pinpointConstants = new PinpointConstants()
            .hardwareMapName("pinpoint")
            .forwardPodY(4.9)
            .strafePodX(4.9)
            .distanceUnit(DistanceUnit.INCH)
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(0.95,
            100,
            1,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(mecanumConstants)
                .pinpointLocalizer(pinpointConstants)
                .build();
    }
}