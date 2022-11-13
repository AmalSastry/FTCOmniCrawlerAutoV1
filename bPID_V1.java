package org.openftc.easyopencv;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID_V1 extends Thread {
    public Telemetry telemetry;
    private final ElapsedTime runtime = new ElapsedTime();
    public DcMotor LeftDrive = null;
    public DcMotor LeftRoll = null;
    public DcMotor RightDrive = null;
    public DcMotor RightRoll = null;
    double rollGoal;
    double leftDriveSpeed;
    double rightDriveSpeed;
    double leftRollError;
    double rightRollError;
    double integralLeftRoll = 0;
    double integralRightRoll = 0;
    double oldLeftRollPower = 0;
    double oldRightRollPower = 0;
    double leftARollGoal = 0;
    double rightARollGoal = 0;
    double leftADrivePower = 0;
    double rightADrivePower = 0;
    double rollEndGoal = 0;
    double currentLeftDrive;
    double currentRightDrive;
    double currentLeftRoll = 0;
    double currentRightRoll = 0;
    boolean end = false;

    public PID_V1(Telemetry telemetry) {
        this.telemetry = telemetry;
    }
    public void setMotor(DcMotor leftDrive, DcMotor leftRoll,DcMotor rightDrive,DcMotor rightRoll){
        this.LeftDrive = leftDrive;
        this.LeftRoll = leftRoll;
        this.RightDrive = rightDrive;
        this.RightRoll = rightRoll;
    }

    public void setRollSpeed(double rollSpeed) {
        //Encoder ticks per 10 milliseconds:
        this.rollGoal = rollSpeed * 50;
        //this.rollGoal = 0;
    }
    public void setDriveSpeed(double leftDriveSpeed, double rightDriveSpeed) {
        this.leftDriveSpeed = leftDriveSpeed * 20;
        this.rightDriveSpeed = rightDriveSpeed * 20;
    }
    public double Acceleration(double Goal, double aGoal, double Acceleration) {
        if (aGoal + Acceleration < Goal) {
            aGoal = aGoal + Acceleration;
        } else if (aGoal - Acceleration > Goal) {
            aGoal = aGoal - Acceleration;
        } else {
            aGoal = Goal;
        }
        return(aGoal);
    }
    public double maxMin(double input, double max, double min) {
        if(input > max) {
            input = max;
        } else if(input < min) {
            input = min;
        }
        return(input);
    }
    public void end() {
        end = true;
    }
    public void run() {

        LeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftRoll.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightRoll.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        LeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LeftRoll.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RightRoll.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        runtime.reset();
        double newTime = runtime.time();
        while(!end) {
            double oldTime = newTime;
            double fullRotation = 537.43333333333333;
            newTime = runtime.time();
            double passedTime = newTime - oldTime;
            double oldLeftRoll = currentLeftRoll;
            double oldRightRoll = currentRightRoll;
            currentLeftRoll = LeftRoll.getCurrentPosition();
            currentRightRoll = RightRoll.getCurrentPosition();
            double leftRollSpeed = ((currentLeftRoll - oldLeftRoll) / passedTime);
            double rightRollSpeed = ((currentRightRoll - oldRightRoll) / passedTime);

            double oldLeftDrive = currentLeftDrive;
            double oldRightDrive = currentRightDrive;
            currentLeftDrive = LeftDrive.getCurrentPosition();
            currentRightDrive = RightDrive.getCurrentPosition();

            double currentLeftDriveSpeed = ((currentLeftDrive - oldLeftDrive) / passedTime);
            double currentRightDriveSpeed = ((currentRightDrive - oldRightDrive) / passedTime);

            double rollStartPos = (currentRightRoll + ((currentLeftRoll - currentRightRoll)/2));
            double newLeftRollPower;
            double newRightRollPower;

            double KpRoll = 0.025;
            double KiRoll = 0.06;
            double KpDrive = 0.08;
            double KpSwitch = 0.07;

            double leftHalfCount = currentLeftRoll / (fullRotation/2);
            double leftDistance = leftHalfCount - Math.floor(leftHalfCount);
            double leftDistanceToClosestSwitch = (leftDistance - (Math.signum(leftDistance)/2)) * (fullRotation/2);
            double leftSwitchEffect = (Math.signum(leftDistanceToClosestSwitch)*(fullRotation/4) - leftDistanceToClosestSwitch) * KpSwitch;
            double rightHalfCount = currentLeftRoll / (fullRotation/2);
            double rightDistance = rightHalfCount - Math.floor(rightHalfCount);
            double rightDistanceToClosestSwitch = (rightDistance - (Math.signum(rightDistance)/2)) * (fullRotation/2);
            double rightSwitchEffect = (Math.signum(rightDistanceToClosestSwitch)*(fullRotation/4) - rightDistanceToClosestSwitch) * KpSwitch;

            double leftRollGoal = rollStartPos - currentLeftRoll+ rollGoal + leftSwitchEffect;
            double rightRollGoal = rollStartPos - currentRightRoll+ rollGoal + rightSwitchEffect;
            leftARollGoal = Acceleration(leftRollGoal, leftARollGoal, 2);
            rightARollGoal = Acceleration(rightRollGoal, rightARollGoal, 2);

            leftRollError = leftARollGoal;
            rightRollError = rightARollGoal;

            integralLeftRoll = integralLeftRoll + (leftRollError * passedTime);
            integralRightRoll = integralRightRoll + (rightRollError * passedTime);

            double integralCap = 1;
            if (integralLeftRoll > integralCap) {
                integralLeftRoll = integralCap;
            } else if (integralLeftRoll < -integralCap) {
                integralLeftRoll = -integralCap;
            }
            if (integralRightRoll > integralCap) {
                integralRightRoll = integralCap;
            } else if (integralRightRoll < -integralCap) {
                integralRightRoll = -integralCap;
            }

            newLeftRollPower = (KpRoll * leftRollError) + (KiRoll * integralLeftRoll);
            newRightRollPower = (KpRoll * rightRollError) + (KiRoll * integralRightRoll);

            LeftRoll.setPower(newLeftRollPower);
            RightRoll.setPower(newRightRollPower);





            double leftDriveDir;
            double leftHalfAmountPlusQuarter = (currentLeftRoll + (fullRotation/4)) / (fullRotation/2);
            if ((int)Math.floor(leftHalfAmountPlusQuarter)% 2 == 0) {
                leftDriveDir = 1;
            }
            else {
                leftDriveDir = -1;
            }

            double rightDriveDir;
            double rightHalfAmountPlusQuarter = (currentRightRoll + (fullRotation/4)) / (fullRotation/2);
            if ((int)Math.floor(rightHalfAmountPlusQuarter)% 2 == 0) {
                rightDriveDir = 1;
            }
            else {
                rightDriveDir = -1;
            }

            double leftDriveSpeedGoal = (leftDriveSpeed * leftDriveDir) - (leftRollSpeed / 100);
            double rightDriveSpeedGoal = (rightDriveSpeed * rightDriveDir) - (rightRollSpeed / 100);

            double leftDriveError = (leftDriveSpeedGoal - (currentLeftDriveSpeed / 60));
            double rightDriveError = (rightDriveSpeedGoal - (currentRightDriveSpeed / 60));

            double leftDriveGoalPower = leftADrivePower + (leftDriveError * KpDrive);
            double rightDriveGoalPower = rightADrivePower + (rightDriveError * KpDrive);

            leftADrivePower = Acceleration(leftDriveGoalPower, leftADrivePower, 0.04);
            leftADrivePower = maxMin(leftADrivePower, 1, -1);

            rightADrivePower = Acceleration(rightDriveGoalPower, rightADrivePower, 0.04);
            rightADrivePower = maxMin(rightADrivePower, 1, -1);

            LeftDrive.setPower(leftADrivePower);
            RightDrive.setPower(rightADrivePower);

            oldLeftRollPower = newLeftRollPower;
            oldRightRollPower = newRightRollPower;







            telemetry.addData("Roll End Goal", rollEndGoal);
            telemetry.addData("Integral Left Roll", integralLeftRoll);
            telemetry.addData("Integral Right Roll", integralRightRoll);
            telemetry.addData("Left Roll Error", leftRollError);
            telemetry.addData("Right Roll Error", rightRollError);
            telemetry.addData("Left Roll Power:", newLeftRollPower);
            telemetry.addData("Right Roll Power:", newRightRollPower);
            telemetry.addData("Left Drive Power:", leftADrivePower);
            telemetry.addData("Left Drive Goal:", leftDriveSpeedGoal);
            telemetry.addData("Right Drive Power:", rightADrivePower);
            telemetry.addData("Right Drive Goal:", rightDriveSpeedGoal);

            telemetry.addData("Left Switch Effect:", leftSwitchEffect);
            telemetry.addData("Right Switch Effect:", rightSwitchEffect);


            telemetry.update();

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }
}