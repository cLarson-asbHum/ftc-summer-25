package org.firstinspires.ftc.teamcode.gobilda;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.acmerobotics.roadrunner.ftc.Encoder;
import com.acmerobotics.roadrunner.ftc.FlightRecorder;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.PositionVelocityPair;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.messages.TwoDeadWheelInputsMessage;
import org.firstinspires.ftc.teamcode.messages.RRGoBildaInputsMessage;

@Config
public final class RRGobildaLocalizer implements Localizer {
    public static class Params {

        // 30 = 2004.5
        public double parYTicks = 2004.5; // y position of the parallel encoder (in tick units)

        // 30 = 130.1
        public double perpXTicks = 130.1; // x position of the perpendicular encoder (in tick units)

    }

    public static Params PARAMS = new Params();

    public final Encoder par, perp;
    // public final IMU imu;
    public final GoBildaPinpointDriver bildaDriver;
    public boolean hasReturnedNaN = false;

    private int lastParPos, lastPerpPos;
    private Rotation2d lastHeading;

    private PoseVelocity2d lastPoseVel;

    private final double inPerTick;

    private double lastRawHeadingVel, headingVelOffset;
    private boolean initialized;
    private Pose2d pose;

    public RRGobildaLocalizer(HardwareMap hardwareMap, GoBildaPinpointDriver bildaDriver, double inPerTick, Pose2d initialPose) {
        // TODO: make sure your config has **motors** with these names (or change them)
        //   the encoders should be plugged into the slot matching the named motor
        //   see https://ftc-docs.firstinspires.org/en/latest/hardware_and_software_configuration/configuring/index.html
        par = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "backRight")));
        perp = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "frontRight")));

        // TODO: reverse encoder directions if needed
        //   par.setDirection(DcMotorSimple.Direction.REVERSE);

        // this.imu = imu;

        this.bildaDriver = bildaDriver;

        this.bildaDriver.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD);

        this.bildaDriver.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        this.bildaDriver.recalibrateIMU(); // should let robot sit still for 0.25s

        this.inPerTick = inPerTick;

        FlightRecorder.write("RR_GOBILDA_PARAMS", PARAMS);

        pose = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        this.pose = pose;
    }

    @Override
    public Pose2d getPose() {
        return pose;
    }

    @Override
    public PoseVelocity2d update() {
        PositionVelocityPair parPosVel = par.getPositionAndVelocity();
        PositionVelocityPair perpPosVel = perp.getPositionAndVelocity();

        // YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        // // Use degrees here to work around https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/1070
        // AngularVelocity angularVelocityDegrees = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        // AngularVelocity angularVelocity = new AngularVelocity(
        //         UnnormalizedAngleUnit.RADIANS,
        //         (float) Math.toRadians(angularVelocityDegrees.xRotationRate),
        //         (float) Math.toRadians(angularVelocityDegrees.yRotationRate),
        //         (float) Math.toRadians(angularVelocityDegrees.zRotationRate),
        //         angularVelocityDegrees.acquisitionTime
        // );

        bildaDriver.update();
        Pose2D bildaPos = bildaDriver.getPosition();
        Pose2D bildaVel = bildaDriver.getVelocity();

        FlightRecorder.write("RR_GOBILDA_INPUTS", new RRGoBildaInputsMessage(parPosVel, perpPosVel,
                bildaPos.getHeading(AngleUnit.RADIANS), bildaVel.getHeading(AngleUnit.RADIANS)));

        // Rotation2d heading = Rotation2d.exp(angles.getYaw(AngleUnit.RADIANS));
        Rotation2d heading = Rotation2d.exp(bildaPos.getHeading(AngleUnit.RADIANS));

        // see https://github.com/FIRST-Tech-Challenge/FtcRobotController/issues/617
        // TODO: uncomment if feedforward for heading vel not working as expected
        // double rawHeadingVel = bildaVel.getHeading(AngleUnit.RADIANS);
        // if (Math.abs(rawHeadingVel - lastRawHeadingVel) > Math.PI) {
        //     headingVelOffset -= Math.signum(rawHeadingVel) * 2 * Math.PI;
        // }
        // lastRawHeadingVel = rawHeadingVel;
        // double headingVel = headingVelOffset + rawHeadingVel;

        double headingVel = (float) bildaVel.getHeading(AngleUnit.RADIANS);

        if (!initialized) {
            initialized = true;

            lastParPos = parPosVel.position;
            lastPerpPos = perpPosVel.position;
            lastHeading = heading;

            return lastPoseVel = new PoseVelocity2d(new Vector2d(0.0, 0.0), 0.0);
        }

        int parPosDelta = parPosVel.position - lastParPos;
        int perpPosDelta = perpPosVel.position - lastPerpPos;
        double headingDelta = heading.minus(lastHeading);

        // Protecting against the NaN spikes the pinpoint produces
        if (Double.isNaN(headingDelta) || Double.isNaN(headingVel)) {
            hasReturnedNaN = true;
            return lastPoseVel;
        }

        Twist2dDual<Time> twist = new Twist2dDual<>(
                new Vector2dDual<>(
                        new DualNum<Time>(new double[] {
                                parPosDelta - PARAMS.parYTicks * headingDelta,
                                parPosVel.velocity - PARAMS.parYTicks * headingVel,
                        }).times(inPerTick),
                        new DualNum<Time>(new double[] {
                                perpPosDelta - PARAMS.perpXTicks * headingDelta,
                                perpPosVel.velocity - PARAMS.perpXTicks * headingVel,
                        }).times(inPerTick)
                ),
                new DualNum<>(new double[] {
                        headingDelta,
                        headingVel,
                })
        );

        lastParPos = parPosVel.position;
        lastPerpPos = perpPosVel.position;
        lastHeading = heading;

        pose = pose.plus(twist.value());
        return lastPoseVel = twist.velocity().value();
    }
}
