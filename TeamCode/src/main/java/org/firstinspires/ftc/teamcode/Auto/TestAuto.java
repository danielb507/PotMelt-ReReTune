package org.firstinspires.ftc.teamcode.Auto;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.Actions.ArmActions;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "TestAuto", group = "Autonomous")
public class TestAuto extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(0, 38, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        if (hardwareMap == null) {
            telemetry.addData("Error", "hardwareMap is not initialized");
            telemetry.update();
            return;
        }


        ArmActions armActions = new ArmActions(hardwareMap);
        //ArmActions  Arm = new ArmActions(hardwareMap);

        TrajectoryActionBuilder traj_8 = drive.actionBuilder(new Pose2d(0, 38, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-48, 55, Math.toRadians(270)), Math.toRadians(90));//Park

        //

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }
        telemetry.update();
        waitForStart();

        //Actions.runBlocking(armActions.raiseClaw());
        //Actions.runBlocking(armActions.closeClaw());

        if (isStopRequested()) return;


        Action trajectory_8;

        trajectory_8 = traj_8.build();

        Actions.runBlocking(
                new SequentialAction(
                        trajectory_8
                )
        );
    }
}
