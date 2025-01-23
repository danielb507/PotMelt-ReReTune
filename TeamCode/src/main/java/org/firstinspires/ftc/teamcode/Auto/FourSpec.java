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
@Autonomous(name = "Four_Spec", group = "Autonomous")
public class FourSpec extends LinearOpMode {

    @Override
    public void runOpMode() {

        Pose2d startPose = new Pose2d(-10, 63, Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        if (hardwareMap == null) {
            telemetry.addData("Error", "hardwareMap is not initialized");
            telemetry.update();
            return;
        }


        ArmActions armActions = new ArmActions(hardwareMap);
        //ArmActions  Arm = new ArmActions(hardwareMap);



        TrajectoryActionBuilder traj_1 = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(-10, 39.5)); //Scores first spec

        TrajectoryActionBuilder traj_2 = drive.actionBuilder(new Pose2d(-10, 39.5, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-33, 39, Math.toRadians(270)), Math.toRadians(270))
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-38, 11, Math.toRadians(270)), Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-42, 53), Math.toRadians(270)) //push 1st sample in
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-49,11), Math.toRadians(180))
                .setTangent(Math.toRadians(180))
                .splineToConstantHeading(new Vector2d(-51, 52), Math.toRadians(270)) //push 2nd sample in
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-48, 44), Math.toRadians(0))
                .setTangent(Math.toRadians(0))
                .splineToConstantHeading(new Vector2d(-44,59.5), Math.toRadians(90)); //goes to pick up spec

        TrajectoryActionBuilder traj_25 = drive.actionBuilder(new Pose2d(-44, 59.5, Math.toRadians(270)))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-44,59.2), Math.toRadians(270));

        TrajectoryActionBuilder traj_3 = drive.actionBuilder(new Pose2d(-44, 59.2, Math.toRadians(270)))
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(0,36, Math.toRadians(90)), Math.toRadians(270)); //scores 2nd spec

        TrajectoryActionBuilder traj_4 = drive.actionBuilder(new Pose2d(0, 36, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-44,61, Math.toRadians(270)), Math.toRadians(90)); //pick up 3rd spec

        TrajectoryActionBuilder traj_45 = drive.actionBuilder(new Pose2d(-44, 61, Math.toRadians(270)))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-44,60.5), Math.toRadians(270));

        TrajectoryActionBuilder traj_5 = drive.actionBuilder(new Pose2d(-44, 60.5, Math.toRadians(270)))
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-8,37, Math.toRadians(90)), Math.toRadians(270)); //score 3rd spec

        TrajectoryActionBuilder traj_6 = drive.actionBuilder(new Pose2d(-8, 37, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-44, 61, Math.toRadians(270)), Math.toRadians(90)); //pick up 4th spec

        TrajectoryActionBuilder traj_65 = drive.actionBuilder(new Pose2d(-44, 61, Math.toRadians(270)))
                .setTangent(Math.toRadians(270))
                .splineToConstantHeading(new Vector2d(-44,60), Math.toRadians(270));

        TrajectoryActionBuilder traj_7 = drive.actionBuilder(new Pose2d(-44, 60, Math.toRadians(270)))
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-4, 37, Math.toRadians(90)), Math.toRadians(270)); //score 4th spec

        TrajectoryActionBuilder traj_8 = drive.actionBuilder(new Pose2d(-4, 37, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-44, 57, Math.toRadians(270)), Math.toRadians(90)); //Park

        //

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.update();
        }
        telemetry.update();
        waitForStart();

        //Actions.runBlocking(armActions.raiseClaw());
        //Actions.runBlocking(armActions.closeClaw());

        if (isStopRequested()) return;

        Action trajectory_1;
        Action trajectory_2;
        Action trajectory_25;
        Action trajectory_3;
        Action trajectory_4;
        Action trajectory_45;
        Action trajectory_5;
        Action trajectory_6;
        Action trajectory_65;
        Action trajectory_7;
        Action trajectory_8;

        trajectory_1 = traj_1.build();
        trajectory_2 = traj_2.build();
        trajectory_25 = traj_25.build();
        trajectory_3 = traj_3.build();
        trajectory_4 = traj_4.build();
        trajectory_45 = traj_45.build();
        trajectory_5 = traj_5.build();
        trajectory_6 = traj_6.build();
        trajectory_65 = traj_65.build();
        trajectory_7 = traj_7.build();
        trajectory_8 = traj_8.build();

        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                armActions.clawDown(),
                                armActions.closeClaw(),
                                armActions.raiseArmParm(1900),
                                trajectory_1
                        ),
                        armActions.halfLowerArm(),
                        armActions.openClaw(),
                        new ParallelAction(
                                trajectory_2, //pushes samples
                                armActions.lowerArm(),
                                armActions.halfOpenClaw(),
                                armActions.raiseClaw()
                        ),
                        trajectory_25,
                        armActions.raiseArmParm(40),
                        armActions.closeClaw(), //pick up spec two
                        new ParallelAction(
                                armActions.raiseArmParm(2150),
                                armActions.clawDown(),
                                trajectory_3 //score spec 2
                        ),
                        armActions.halfLowerArm(),
                        armActions.openClaw(),
                        new ParallelAction(
                                armActions.lowerArm(),
                                armActions.halfOpenClaw(),
                                armActions.raiseClaw(),
                                trajectory_4 //go to pick up spec 3
                        ),
                        trajectory_45,
                        armActions.raiseArmParm(30),
                        armActions.closeClaw(),
                        new ParallelAction(
                                armActions.raiseArmParm(2200),
                                armActions.clawDown(),
                                trajectory_5
                        ),
                        armActions.halfLowerArm(), //Score Spec three
                        armActions.openClaw(),
                        new ParallelAction(
                                armActions.lowerArm(),
                                armActions.halfOpenClaw(),
                                armActions.raiseClaw(),
                                trajectory_6
                        ),
                        trajectory_65,
                        armActions.raiseArmParm(30),
                        armActions.closeClaw(),
                        new ParallelAction(
                                armActions.raiseArmParm(2200),
                                armActions.clawDown(),
                                trajectory_7
                        ),
                        armActions.halfLowerArm(),
                        armActions.openClaw(),
                        new ParallelAction(
                                armActions.lowerArm(),
                                trajectory_8
                        )
                )
        );
    }
}
