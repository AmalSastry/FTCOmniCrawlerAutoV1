package org.firstinspires.ftc.teamcode.CIA;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous (name="RED", group="Autonomous")
public class AutoV1 extends LinearOpMode {

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
        PID.setRollSpeed(3.5);
        sleep(4000)

    }
}
