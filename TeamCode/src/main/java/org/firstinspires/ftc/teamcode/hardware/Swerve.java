package org.firstinspires.ftc.teamcode.hardware;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.constraints.SwerveVelocityConstraint;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.IMUNavSource2;
import org.ftc9974.thorcore.control.navigation.roadrunner.HolonomicDrivetrainRR;
import org.ftc9974.thorcore.control.navigation.roadrunner.Odometry;
import org.ftc9974.thorcore.robot.drivetrains.swerve2.SwerveDrive2;
import org.ftc9974.thorcore.robot.drivetrains.swerve2.SwerveModule2;
import org.ftc9974.thorcore.util.MathUtilities;

public class Swerve implements HolonomicDrivetrainRR {

    private static final double WHEEL_RADIUS = 76.2 / 2.0;
    private static final double GEAR_RATIO = 3.7 * (44.0/36) * 2;
    private static final double MOTOR_FREE_SPEED = MathUtilities.rpmToRadPerSec(6000);
    private static final double TRACK_WIDTH = 2 * 132.8;
    private static final double WHEELBASE = 2 * 132.8;

    private static final PIDFCoefficients COEFS = new PIDFCoefficients(0.17, 0, 0, 0);
    private static final SwerveModule2.GainSchedule GAIN_SCHEDULE = SwerveModule2.constantGains(COEFS);

    public static final double FULL_LINEAR_SPEED = WHEEL_RADIUS * MOTOR_FREE_SPEED / GEAR_RATIO;
    public static final double FULL_ANGULAR_SPEED = FULL_LINEAR_SPEED / Math.hypot(0.5 * TRACK_WIDTH, 0.5 * WHEELBASE);
    public static final SwerveVelocityConstraint VELOCITY_CONSTRAINT = new SwerveVelocityConstraint(
            FULL_LINEAR_SPEED, TRACK_WIDTH
    );

    public final SwerveDrive2 rb;
    public final SwerveModule2 fl, fr, bl, br;

    public final Odometry odometry;
    public final IMUNavSource2 imu;

    public Swerve(HardwareMap hardwareMap) {
        fl = new SwerveModule2("frontLeft", hardwareMap, new Vector2(132.8, 132.8), 3.027,
                WHEEL_RADIUS, GEAR_RATIO, MOTOR_FREE_SPEED, GAIN_SCHEDULE);
        fr = new SwerveModule2("frontRight", hardwareMap, new Vector2(132.8, -132.8), 4.025 - PI,
                WHEEL_RADIUS, GEAR_RATIO, MOTOR_FREE_SPEED, GAIN_SCHEDULE);
        bl = new SwerveModule2("backLeft", hardwareMap, new Vector2(-132.8, 132.8), 1.549,
                WHEEL_RADIUS, GEAR_RATIO, MOTOR_FREE_SPEED, GAIN_SCHEDULE);
        br = new SwerveModule2("backRight", hardwareMap, new Vector2(-132.8, -132.8), 4.15 - PI,
                WHEEL_RADIUS, GEAR_RATIO, MOTOR_FREE_SPEED, GAIN_SCHEDULE);
        rb = new SwerveDrive2(fl, fr, bl, br);

        odometry = new SwerveOdometry(hardwareMap);
        imu = new IMUNavSource2(hardwareMap, new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
    }

    public void drive(Vector2 linearVelocity, double angularVelocity) {
        rb.drive(linearVelocity, angularVelocity);
    }

    @Override
    public void drive(Pose2d velocity, Pose2d acceleration) {
        rb.drive(new Vector2(velocity.getX(), velocity.getY()), velocity.getHeading());
    }

    public void update() {
        rb.update();
    }

    public void preAlignWheels(Vector2 linearVelocity, double angularVelocity) {
        rb.preAlignWheels(linearVelocity, angularVelocity);
    }

    public void preAlignWheels(Pose2d velocity) {
        preAlignWheels(new Vector2(velocity.getX(), velocity.getY()), velocity.getHeading());
    }
}
