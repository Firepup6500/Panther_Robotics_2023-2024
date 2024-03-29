package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "TylerControlCode")
public class TylerControlCode extends OpMode {

    DcMotor FrontR;
    DcMotor BackR;
    DcMotor FrontL;
    DcMotor BackL;
    DcMotor ArmLift;
    
    DcMotor Hanging;
    Servo Claw;
    Servo LiftServo;
    Servo Drone;

    @Override
    public void init(){

        FrontR = hardwareMap.get(DcMotor.class,"right_front_drive");
        BackR = hardwareMap.get(DcMotor.class,"right_back_drive");
        FrontL = hardwareMap.get(DcMotor.class,"left_front_drive");
        BackL = hardwareMap.get(DcMotor.class,"left_back_drive");
        BackL = hardwareMap.get(DcMotor.class,"left_back_drive");
        ArmLift = hardwareMap.get(DcMotor.class,"arm_lift");
        Hanging = hardwareMap.get(DcMotor.class,"hanging_motor");
        Claw = hardwareMap.get(Servo.class,"Claw");
        Drone = hardwareMap.get(Servo.class,"Drone");

        telemetry.addData("ftc", "first");
        telemetry.addData("Init", "is a success");
        telemetry.update();
    }

    @Override
    public void loop() {
        if (gamepad1.right_trigger > .1) {
            forward(.5);
        }
        if (gamepad1.right_trigger > .75) {
            forward(1);
        }
        if (gamepad1.left_trigger > .1) {
            backward(.5);
        }
        if (gamepad1.left_trigger > .75) {
            backward(1);
        }
        if (gamepad1.right_bumper) {
            TurnRight(.5);
        }
        if (gamepad1.left_bumper) {
            TurnLeft(.5);
        }
        if (gamepad1.x) {
            StrafeRight(.5);
        }
        if (gamepad1.b) {
            StrafeLeft(.5);
        }
        if (gamepad2.right_bumper) {
            ArmLift.setPower(-5);
        }
        if (gamepad2.left_bumper) {
            ArmLift.setPower(5);
        }
        if (gamepad2.x) {
            Claw.setPosition(1);
        }
        if (gamepad2.b) {
            Claw.setPosition(0);
        }
        if (gamepad2.y) {
            Drone.setPosition(0);
        }
        if (gamepad2.a) {
            Drone.setPosition(1);
        }
        if (gamepad2.dpad_up ) {
            Hanging.setPower(-5);
        }
        if (gamepad2.dpad_down) {
            Hanging.setPower(-5);
        }
        FrontR.setPower(0);
        BackR.setPower(0);
        FrontL.setPower(0);
        BackL.setPower(0);
        ArmLift.setPower(0);
        Hanging.setPower(0);
    }

        public void forward(double power){
            FrontR.setPower(power);
            BackR.setPower(-power);
            FrontL.setPower(power);
            BackL.setPower(power);
        }

        public void backward(double power){
            FrontR.setPower(-power);
            BackR.setPower(power);
            FrontL.setPower(-power);
            BackL.setPower(-power);
        }

        public void TurnRight(double power){
            FrontR.setPower(-power);
            BackR.setPower(power);
            FrontL.setPower(power);
            BackL.setPower(power);
        }

        public void TurnLeft(double power){
            FrontR.setPower(power);
            BackR.setPower(-power);
            FrontL.setPower(-power);
            BackL.setPower(-power);
        }

        public void StrafeRight(double power){
            FrontR.setPower(power);
            BackR.setPower(power);
            FrontL.setPower(-power);
            BackL.setPower(power);
        }

        public void StrafeLeft(double power){
            FrontR.setPower(-power);
            BackR.setPower(-power);
            FrontL.setPower(power);
            BackL.setPower(-power);
        }


}
