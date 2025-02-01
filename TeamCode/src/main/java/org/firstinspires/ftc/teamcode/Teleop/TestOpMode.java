package org.firstinspires.ftc.teamcode.Teleop;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Teleop.Actions.ArmActions;
import org.firstinspires.ftc.teamcode.MecanumDrive;


import java.util.ArrayList;
import java.util.List;
@TeleOp(name="Macro OpMode", group="TeleOp")
public class TestOpMode extends OpMode {


    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();




    @Override
    public void init() {
        Pose2d startPose = new Pose2d(0, 38, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);

        DcMotor leftSlide;
        DcMotor rightSlide;
        leftSlide = hardwareMap.get(DcMotor.class, "leftSlide");
        rightSlide = hardwareMap.get(DcMotor.class, "rightSlide");

        leftSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightSlide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        drive.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        leftSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightSlide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    boolean macroIsRunning = false;

    public void setMacroIsRunning(boolean value){
        macroIsRunning = value;
    }



    @Override
    public void loop() {
        Pose2d startPose = new Pose2d(0, 38, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose);
        ArmActions armActions = new ArmActions(hardwareMap);

        IMU imu = drive.lazyImu.get();


        TelemetryPacket packet = new TelemetryPacket();
        double SLOW_DOWN_FACTOR = 1;
        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        double rx = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.5);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;


        telemetry.addData("Running TeleOp for:", "15344");


        if(!armActions.driveMacroIsRunning){
        drive.leftBack.setPower(backLeftPower);
        drive.leftFront.setPower(frontLeftPower);
        drive.rightBack.setPower(backRightPower);
        drive.rightFront.setPower(frontRightPower);
        }
        drive.updatePoseEstimate();
        // updated based on gamepads

        // update running actions
        List<Action> newActions = new ArrayList<>();
        for (Action action : runningActions) {
            action.preview(packet.fieldOverlay());
            if (action.run(packet)) {
                newActions.add(action);
            }
        }
        runningActions = newActions;


        if (gamepad1.left_bumper && !armActions.armMacroIsRunning) {
            armActions.leftSlide.setPower(-0.5);
            armActions.rightSlide.setPower(-0.5);
        } else if (gamepad1.right_bumper && !armActions.armMacroIsRunning) {
            armActions.leftSlide.setPower(1);
            armActions.rightSlide.setPower(1);
        }
        else if(gamepad1.dpad_left && !armActions.armMacroIsRunning){
            armActions.armMacroIsRunning = true;
        }
        else if (!armActions.armMacroIsRunning) {
            armActions.leftSlide.setPower(0);
            armActions.rightSlide.setPower(0);
        }

        telemetry.addData("ArmMacroVariable", armActions.armMacroIsRunning);



        if (gamepad1.a) {
            runningActions.add(
                    armActions.openClaw(.47)
            );
            telemetry.addData("runningactions", runningActions);
        }
        else if (gamepad1.x) {
            runningActions.add(
                    armActions.closeClaw()
            );
            telemetry.addData("runningactions", runningActions);
        }


        //gamepad 2

        if (gamepad2.left_bumper) {
            runningActions.add(
                    new ParallelAction(
                        new InstantAction(() -> armActions.leftHang.setPower(-1)),
                        new InstantAction(() -> armActions.rightHang.setPower(1))
                    ));
        } else if(gamepad2.right_bumper) {
            runningActions.add(
                    new ParallelAction(
                            new InstantAction(() -> armActions.leftHang.setPower(1)),
                            new InstantAction(() -> armActions.rightHang.setPower(-1))
                    ));
        } else {
            runningActions.add(
                    new ParallelAction(
                            new InstantAction(() -> armActions.leftHang.setPower(0)),
                            new InstantAction(() -> armActions.rightHang.setPower(0))
                    ));
        }


        //Intake Stuff

        if (gamepad2.dpad_left) {
            runningActions.add(armActions.runSlide());
        } else if (gamepad2.dpad_right) {
            runningActions.add(armActions.reverseSlide());
        } else {
            runningActions.add(armActions.stopSlide());
        }

        if(gamepad2.dpad_up){
            runningActions.add(armActions.raiseIntake());
        }

        else if(gamepad2.dpad_down){
            runningActions.add(armActions.lowerIntake());
        }

        if(gamepad2.a){
            runningActions.add(armActions.reverseIntake());
        }
        else if(gamepad2.b){
            runningActions.add(armActions.runIntake());
        }
        else{
            runningActions.add(armActions.stopIntake());
        }

        TrajectoryActionBuilder high_chamber_to_pickup = drive.actionBuilder(new Pose2d(-7, 34, Math.toRadians(90)))
                .setTangent(Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-48, 59, Math.toRadians(270)), Math.toRadians(90));

        TrajectoryActionBuilder pickup_to_high_chamber = drive.actionBuilder(new Pose2d(-48, 60, Math.toRadians(270)))
                .setTangent(Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-10, 34, Math.toRadians(90)), Math.toRadians(270));

        if(gamepad1.b){
            macroIsRunning = true;
            drive.localizer.setPose(new Pose2d(-10, 34, Math.toRadians(90)));
            drive.updatePoseEstimate();
            Actions.runBlocking(new ParallelAction(high_chamber_to_pickup.build(), armActions.openClaw(.47), armActions.raiseClaw(), armActions.lowerArmAuto(300)));
        }

        if(gamepad1.y){
            macroIsRunning = true;
            drive.localizer.setPose(new Pose2d(-48, 60, Math.toRadians(270)));
            drive.updatePoseEstimate();
            Actions.runBlocking(new ParallelAction(pickup_to_high_chamber.build(), armActions.clawDown(), armActions.raiseArm()));
        }

        if(gamepad1.dpad_down && !armActions.armMacroIsRunning){
            Actions.runBlocking(new ParallelAction(armActions.clawDown(), armActions.setDriveMacro(false)));
        }

        if(gamepad1.dpad_up && !armActions.armMacroIsRunning){
            Actions.runBlocking(new ParallelAction(armActions.raiseClaw(), armActions.setDriveMacro(false)));
        }


        telemetry.addData("E", runningActions.size());
        dash.sendTelemetryPacket(packet);

    }
}
