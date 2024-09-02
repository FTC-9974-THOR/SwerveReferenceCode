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

public class Swerve implements HolonomicDrivetrainRR {

    private static final double wheelRadius = 76.0 / 2.0;
    private static final double fullSpeed = calculateFullSpeed(wheelRadius);
    private static final double powerPerMMPerSecond = 1 / fullSpeed;
    private static final double mmPerTick = wheelRadius * 2 * PI / (4 * 7 * 3.7 * 2);
    private static final PIDFCoefficients coefs = new PIDFCoefficients(0.17, 0, 0, 0);
    private static final SwerveModule2.GainSchedule gainSchedule = SwerveModule2.constantGains(coefs);

    public static final SwerveVelocityConstraint VELOCITY_CONSTRAINT = new SwerveVelocityConstraint(
            fullSpeed, 2 * 132.8
    );
    public static final double FULL_LINEAR_SPEED = fullSpeed;
    public static final double FULL_ANGULAR_SPEED = fullSpeed / Math.hypot(132.8, 132.8);

    public final SwerveDrive2 rb;
    public final SwerveModule2 fl, fr, bl, br;

    public final Odometry odometry;
    public final IMUNavSource2 imu;

    public Swerve(HardwareMap hardwareMap) {
        // if the pods are moved around, the positions here need to be changed. the position should
        // be the center of the contact patch of the wheel, measured relative to the center of the
        // robot. +x is forward, and +y is left. units are millimeters.
        // the servo offset is the angle the servo reads when the module is pointing forwards (with
        // forwards defined as "the module pulls the robot forwards when you apply positive power to
        // the motor). servo offset is used under the hood to account for servo phasing. the valid
        // range for servo offset is 0 to 2 * pi. units are radians.
        fl = new SwerveModule2("frontLeft", hardwareMap, new Vector2(132.8, 132.8), 3.027,
                powerPerMMPerSecond, mmPerTick, gainSchedule);
        fr = new SwerveModule2("frontRight", hardwareMap, new Vector2(132.8, -132.8), 4.025 - PI,
                powerPerMMPerSecond, mmPerTick, gainSchedule);
        bl = new SwerveModule2("backLeft", hardwareMap, new Vector2(-132.8, 132.8), 1.549,
                powerPerMMPerSecond, mmPerTick, gainSchedule);
        br = new SwerveModule2("backRight", hardwareMap, new Vector2(-132.8, -132.8), 4.15 - PI,
                powerPerMMPerSecond, mmPerTick, gainSchedule);
        rb = new SwerveDrive2(fl, fr, bl, br);

        odometry = new SwerveOdometry(hardwareMap);
        imu = new IMUNavSource2(hardwareMap, new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
    }

    /** @noinspection SameParameterValue*/
    private static double calculateFullSpeed(double wheelRadius) {
        // if the gear ratio or motors are changed, these constants need to be updated.
        final double mmPerWheelRevolution = 2 * PI * wheelRadius;
        final double wheelRevolutionsPerMotorRevolution = 0.092138;
        final double ticksPerMotorRevolution = 28;
        final double wheelRevolutionsPerTick = wheelRevolutionsPerMotorRevolution / ticksPerMotorRevolution;
        final double mmPerTick = mmPerWheelRevolution * wheelRevolutionsPerTick;
        //final double ticksPerMM = 1 / mmPerTick;

        return 6000 /* rpm */ * ticksPerMotorRevolution / 60 /* s/min */ * mmPerTick; // mm/s
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
