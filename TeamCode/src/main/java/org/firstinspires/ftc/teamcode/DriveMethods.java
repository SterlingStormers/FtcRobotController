package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public abstract class DriveMethods extends OpMode {
    Devices robot = new Devices();

    /**
     * Given the desired movement, sets the power to each wheel to match that as well as it can.
     *
     * @param axial   describes the backward and forward movement
     * @param lateral describes left and right movement (strafe)
     * @param yaw     describes a spinning motion
     */
    public void omniDrive(double axial, double lateral, double yaw) {
        // code copied from BasicOmniOpMode_Linear.java
        double max;
        double leftFrontPower = axial + lateral + yaw;
        double rightFrontPower = axial - lateral - yaw;
        double leftBackPower = axial - lateral + yaw;
        double rightBackPower = axial + lateral - yaw;
        double wormGearPower = -gamepad2.left_stick_y;
        boolean chainMotorUp = gamepad1.dpad_up;
        boolean chainMotorDown = gamepad1.dpad_up;
        max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));
        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

//        robot.leftFrontDrive.setPower(leftFrontPower);
//        robot.rightFrontDrive.setPower(rightFrontPower);
//        robot.leftBackDrive.setPower(leftBackPower);
//        robot.rightBackDrive.setPower(rightBackPower);

        boolean slowMode = gamepad1.right_bumper;
        double SLOW_MODE_SPEED = .15;
        if (slowMode) {
            robot.leftFrontDrive.setPower(SLOW_MODE_SPEED * leftFrontPower);
            robot.rightFrontDrive.setPower(SLOW_MODE_SPEED * rightFrontPower);
            robot.leftBackDrive.setPower(SLOW_MODE_SPEED * leftBackPower);
            robot.rightBackDrive.setPower(SLOW_MODE_SPEED * rightBackPower);
        } else {
            robot.leftFrontDrive.setPower(leftFrontPower);
            robot.rightFrontDrive.setPower(rightFrontPower);
            robot.leftBackDrive.setPower(leftBackPower);
            robot.rightBackDrive.setPower(rightBackPower);
        }
        boolean slowMode2 = gamepad2.left_bumper;
        double SLOW_2_MODE_SPEED = .25;
        if (slowMode2) {
            robot.wormGear.setPower(SLOW_2_MODE_SPEED * wormGearPower);

        } else {
            robot.wormGear.setPower(wormGearPower);
        }



        if (chainMotorUp) {
            robot.chainMotor.setPower(0.75);
        } else if (chainMotorDown) {
            robot.chainMotor.setPower(0.75);
        } else {
            robot.chainMotor.setPower(0);
        }
    }

    /**
     * Changes the slider's target position.
     * This method will make sure the robot slider length stays safe and within the legal box limit.
     *
     * @param targetPosition the position in encoder ticks
     * @return The target position, but within safe legal limits
     */
    double setSliderAndReturnConstraint(double targetPosition) {
        double position = targetPosition;

        position = Math.min(position, robot.upperMaxLegalSliderLength());
        position = Math.min(position, robot.MAX_SAFE_SLIDER_TICKS);
        position = Math.max(position, robot.MIN_SLIDER_TICKS);

//        if (robot.wormGearAngle() < 0) {
//            position = Math.min(position, robot.lowerMaxSliderLength());

        robot.sliderMotor.setTargetPosition((int) position);

        return position;
    }

}

//Yaw=turn, Lateral=SideToSide,  Axial is Forward