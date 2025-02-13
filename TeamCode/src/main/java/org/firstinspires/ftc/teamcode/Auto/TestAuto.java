package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@Autonomous(name="PID", group="Auto")
public class TestAuto extends LinearOpMode {
    DcMotorEx left;
    DcMotorEx right;

    // target in encoder ticks for each motor
    public static double reference = 10000;
    double targetLeft = reference;
    double targetRight = 1000;


    // initialize error
    double leftError = 1000;
    double rightError = 1000;

    // proportional gain
    public static double Kp = 0.001;
    public static double Ki = 0.0000001;
    public static double Kd = 0.00001;

    // pid controllers set with arbitrary values, use at your own risk
    PIDController leftController = new PIDController(Kp,Ki,Kd);
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();


    @Override
    public void runOpMode() throws InterruptedException {


        // configure your motors and other hardware stuff here
        // make sure the strings match the names that you have set on your robot controller configuration
        left = hardwareMap.get(DcMotorEx.class,"armMotor");


        left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // wait until the start button is pressed
        waitForStart();


        /*
         * use proportional feedback to move drive train to reference
         */
        while (opModeIsActive()) {
            packet.put("x", left.getCurrentPosition());
            packet.put("status", "alive");
            left.setPower(leftController.output(targetLeft,left.getCurrentPosition()));
            packet.fieldOverlay()
                    .setFill("blue")
                    .fillRect(-20, -20, 40, 40);
            dashboard.sendTelemetryPacket(packet);


        }


    }

}