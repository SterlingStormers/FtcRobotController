package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Autonomous
public class AutoBasket2024Encoder extends DriveMethods{

    left = hardwareMap.get(DcMotor.class, "leftMotor")

    int leftPos = 0;
    int rightPos = 0;

    left.SetMode(DcMotor.RunMode.RUN_TO_POSITION)

    double stateStartTime = -1;

    enum State {
        Finished,
        Unstarted,
        TightenClaw,
        StrafeRight,
        RaiseArm,
        ExtendSlider,
        MoveForward,
        OpenClaw,
        MoveBack
    }

    AutoBasket2024Encoder.State currentState = AutoBasket2024Encoder.State.Unstarted;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("code", "running");
        telemetry.addData("time", "%.1f", getRuntime());
        telemetry.addData("sliderLength", "%.1f", (double) robot.sliderMotor.getCurrentPosition());
        telemetry.addData("imu", "%.1f", robot.imu.getRobotYawPitchRollAngles().getYaw());

        telemetry.addData("state", currentState);
        switch (currentState) {
            case Unstarted:
                changeState(AutoBasket2024Encoder.State.TightenClaw);
                break;
            case TightenClaw:
                robot.clawServo.setPosition(1.20);
                changeState(AutoBasket2024Encoder.State.StrafeRight);
                break;
            case StrafeRight:
                omniDrive(0, 0.25, 0);

                if (getStateTime() >= 1.2) {
                    omniDrive(0, 0, 0);

                    changeState(AutoBasket2024Encoder.State.RaiseArm);
                }
                break;
            case RaiseArm:
                robot.wormGear.setPower(0.5);

                if (robot.wormGearAngle() >= 70) {
                    robot.wormGear.setPower(0);

                    changeState(AutoBasket2024Encoder.State.MoveForward);
                }
                break;
            case MoveForward:
                omniDrive(0.15, 0, 0);

                if (getStateTime() >= 1.1) {
                    omniDrive(0, 0, 0);
                    changeState(AutoBasket2024Encoder.State.ExtendSlider);
                }
                break;
            case ExtendSlider:
                robot.sliderMotor.setPower(0.5);
                robot.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setSliderAndReturnConstraint(robot.MAX_HORIZONTAL_SLIDER_TICKS);

                if (robot.sliderMotor.getCurrentPosition() >= 1450) {
                    robot.sliderMotor.setPower(0);
                    changeState(AutoBasket2024Encoder.State.OpenClaw);
                }
                break;
            case OpenClaw:
                robot.clawServo.setPosition(robot.CLAW_OPEN);
                changeState(AutoBasket2024Encoder.State.MoveBack);
                break;
            case MoveBack:
                omniDrive(-0.5, 0, 0);
                if (getStateTime() >= 1) {
                    omniDrive(0, 0, 0);
                    changeState(AutoBasket2024Encoder.State.Finished);
                }
            case Finished:
                omniDrive(0, 0, 0);
                break;
        }
//spider
    }

    void changeState(AutoBasket2024Encoder.State nextState) {
        currentState = nextState;
        stateStartTime = getRuntime();
    }

    double getStateTime() {
        return getRuntime() - stateStartTime;
    }


//        if (getRuntime() < 2) {
//            omniDrive(0, 1, 0);
//        } else if (getRuntime() < 4) {
//            omniDrive(0, 0, 1);
//        }
//        else {
//                omniDrive(0, 0, 0);
//            }
//        }

    //            case MoveForward:
//                omniDrive(0.5, 0, 0);
//
//                if (getStateTime() >= 0.7) {
//                    omniDrive(0, 0, 0);
//
//                    changeState(State.RaiseArm);
//                }
//                break;

}
