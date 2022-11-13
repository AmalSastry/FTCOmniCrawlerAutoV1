

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.PID_V1;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;



@TeleOp(name="PID_Run", group="Linear Opmode")
public class bPID_Run extends LinearOpMode {

    DcMotor Arm = null;

    public void armRunTooPosition(int position) {
        Arm.setTargetPosition(position);
        Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Arm.setPower(1);
    }
    public void servoRunTooPosition(Servo servo, double position) {
        servo.setPosition(position);
    }

    @Override
    public void runOpMode() {
        DcMotor LeftDrive = hardwareMap.get(DcMotor.class, "LeftDrive");
        DcMotor LeftRoll = hardwareMap.get(DcMotor.class, "LeftRoll");
        DcMotor RightDrive = hardwareMap.get(DcMotor.class, "RightDrive");
        DcMotor RightRoll = hardwareMap.get(DcMotor.class, "RightRoll");

        Arm = hardwareMap.get(DcMotor.class, "Arm");
        Servo Claw = hardwareMap.get(Servo.class, "Claw");
        Servo liftClaw = hardwareMap.get(Servo.class, "LClaw");

        PID_V1 PID = new PID_V1(telemetry);

        PID.setMotor(LeftDrive, LeftRoll, RightDrive, RightRoll);

        waitForStart();
        PID.start();



        while (opModeIsActive()) {
            if (gamepad1.left_trigger >= 0.5) {
                PID.setRollSpeed(gamepad1.left_stick_x/3.5);
            } else {
                PID.setRollSpeed(gamepad1.left_stick_x);
            }
            PID.setDriveSpeed(gamepad1.left_stick_y, gamepad1.right_stick_y);

            if(gamepad2.dpad_up==true) {
                armRunTooPosition(-3250);
            }
            if(gamepad2.dpad_left==true) {
                armRunTooPosition(-1930);
            }
            if(gamepad2.dpad_right==true) {
                armRunTooPosition(-900);
            }
            if(gamepad2.dpad_down==true) {
                armRunTooPosition(0);
            }


            if (gamepad2.right_stick_y >= 0.9) {
                servoRunTooPosition(liftClaw, -0.6);
            } else if (gamepad2.right_stick_y <= -0.9) {
                servoRunTooPosition(liftClaw, 0.6);
            }

            if (gamepad2.right_stick_x >= 0.9) {
                servoRunTooPosition(Claw, 0.8);
            } else {
                servoRunTooPosition(Claw, -1);
            }
        }
        PID.end();
    }}
