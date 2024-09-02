package org.firstinspires.ftc.teamcode.opmodes.auto;

import static java.lang.Math.PI;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.followers.HolonomicPIDVAFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Swerve;
import org.ftc9974.thorcore.control.math.Vector2;
import org.ftc9974.thorcore.control.navigation.roadrunner.OdometryLocalizer;
import org.ftc9974.thorcore.control.navigation.roadrunner.TrajectoryExecutor;
import org.ftc9974.thorcore.util.TimingUtilities;

import java.util.Arrays;

@Autonomous(name = "Swerve Auto")
public class SwerveAuto extends LinearOpMode {

    private Swerve swerve;
    private OdometryLocalizer localizer;
    private TrajectoryExecutor executor;

    private void update() {
        swerve.update();
        localizer.update();

        telemetry.addData("Pose", localizer.pose);
        telemetry.update();
    }

    private void updateWithExecutor() {
        update();
        executor.update(telemetry);
    }

    private void stopRobot() {
        swerve.rb.stop();
    }

    private static TrajectoryBuilder createBuilder(Pose2d startPose, double startTangent) {
        return new TrajectoryBuilder(
                startPose, startTangent,
                new MinVelocityConstraint(Arrays.asList(
                        Swerve.VELOCITY_CONSTRAINT,
                        new TranslationalVelocityConstraint(400)
                )),
                new ProfileAccelerationConstraint(200)
        );
    }

    @Override
    public void runOpMode() throws InterruptedException {
        swerve = new Swerve(hardwareMap);
        localizer = new OdometryLocalizer(swerve.odometry, new Pose2d());

        TrajectoryFollower follower = new HolonomicPIDVAFollower(
                new PIDCoefficients(5, 0, 0),
                new PIDCoefficients(5, 0, 0),
                new PIDCoefficients(5, 0, 0)
        );
        executor = new TrajectoryExecutor(swerve, follower, localizer);

        Trajectory trajectory = createBuilder(new Pose2d(), 0)
                .lineToSplineHeading(new Pose2d(3000, 0, PI))
                .build();
        Trajectory trajectory2 = createBuilder(trajectory.end(), -PI)
                .splineToSplineHeading(new Pose2d(), -PI)
                .build();

        // turn the wheels to be ready for the first trajectory. preAlignWheels sets the heading
        // setpoints for the swerve modules but doesn't move any of the motors. it's important to
        // align the wheels so the robot doesn't have to wait for them to turn when the trajectory
        // starts.
        swerve.preAlignWheels(trajectory.velocity(0));

        // wait for start
        while (!isStopRequested() && !isStarted()) {
            telemetry.addLine("Ready.");
            update();
        }
        if (isStopRequested()) return;

        // start the path
        executor.startPath(trajectory);
        // drive the path
        TimingUtilities.blockWhile(this, executor::isFollowing, this::updateWithExecutor, this::stopRobot);
        if (isStopRequested()) return;

        // start turning the wheels to be ready for the next trajectory
        swerve.preAlignWheels(trajectory2.velocity(0));
        // give the wheels time to turn
        TimingUtilities.sleep(this, 0.5, this::update, this::stopRobot);
        if (isStopRequested()) return;

        stopRobot();

        // start the path
        executor.startPath(trajectory2);
        // drive the path
        TimingUtilities.blockWhile(this, executor::isFollowing, this::updateWithExecutor, this::stopRobot);
        if (isStopRequested()) return;

        stopRobot();
    }
}
