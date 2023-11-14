package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Autonomous Basic")

public class AutonomousFirstVersion extends LinearOpMode {

    DcMotor FrontR;
    DcMotor BackR;
    DcMotor FrontL;
    DcMotor BackL;
    DcMotor ArmLift;
    Servo Claw;

        @Override
        public void runOpMode() throws InterruptedException {

            FrontR = hardwareMap.get(DcMotor.class, "right_front_drive");
            BackR = hardwareMap.get(DcMotor.class, "right_back_drive");
            FrontL = hardwareMap.get(DcMotor.class, "left_front_drive");
            BackL = hardwareMap.get(DcMotor.class, "left_back_drive");
            BackL = hardwareMap.get(DcMotor.class, "left_back_drive");
            ArmLift = hardwareMap.get(DcMotor.class, "arm_lift");
            Claw = hardwareMap.get(Servo.class, "Claw");
            DistanceSensor leftSensor = hardwareMap.get(DistanceSensor.class, "left");
            DistanceSensor rightSensor = hardwareMap.get(DistanceSensor.class, "right");

            telemetry.addData("ftc", "first");
            telemetry.addData("Init", "is a success");
            telemetry.update();

            BackR.setDirection(DcMotorSimple.Direction.REVERSE);

            waitForStart();


            resetRuntime();
            EncoderForward(1000, .75);

            double left;
            double right;
            int direction = 3;
            while (direction == 3) {
                left = leftSensor.getDistance(DistanceUnit.MM);
                right = rightSensor.getDistance(DistanceUnit.MM);
                telemetry.addData("Runtime", getRuntime());
                telemetry.addData("Left", left);
                telemetry.addData("Right", right);
                if (right <= 200) {
                    direction = 1;
                    telemetry.addData("Direction", "Right");
                } else if (left <= 200) {
                    direction = -1;
                    telemetry.addData("Direction", "Left");
                } else {
                    direction = 0;
                    telemetry.addData("Direction", "Forward");
                }
                telemetry.update();
            }


            if(direction == 1){

                EncoderForward(500, .75);
                RightTurn(1200, .75);
                EncoderForward(200, .75);

            }
            else if(direction == -1){

                EncoderForward(500, .75);
                LeftTurn(1200, .75);
                EncoderForward(200, .75);

            }
            else if(direction == 0){

                EncoderForward(500, .5);

            }

        }

        private void EncoderForward(double target, double speed){

            FrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontL.setTargetPosition((int) target);
            FrontR.setTargetPosition((int) target);
            BackL.setTargetPosition((int) target);
            BackR.setTargetPosition((int) target);

            FrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FrontL.setPower(speed);
            FrontR.setPower(speed);
            BackL.setPower(speed);
            BackR.setPower(speed);

            while (opModeIsActive() && isBusy()) {
                idle();
            }
        }

        private void RightTurn(double target, double speed){

            FrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontL.setTargetPosition((int) target);
            FrontR.setTargetPosition((int) -target);
            BackL.setTargetPosition((int)  target);
            BackR.setTargetPosition((int) -target);

            FrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FrontL.setPower(speed);
            FrontR.setPower(speed);
            BackL.setPower(speed);
            BackR.setPower(speed);

            while (opModeIsActive() && isBusy()) {
                idle();
            }
        }

        private void LeftTurn(double target, double speed){

            FrontL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            FrontR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            BackR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            FrontL.setTargetPosition((int) -target);
            FrontR.setTargetPosition((int) target);
            BackL.setTargetPosition((int)  -target);
            BackR.setTargetPosition((int)  target);

            FrontL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            FrontR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            BackR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            FrontL.setPower(speed);
            FrontR.setPower(speed);
            BackL.setPower(speed);
            BackR.setPower(speed);

            while (opModeIsActive() && isBusy()) {
                idle();
            }
        }

    private boolean isBusy() {
        return FrontL.isBusy() && FrontR.isBusy() && BackL.isBusy() && BackR.isBusy();
    }

}
