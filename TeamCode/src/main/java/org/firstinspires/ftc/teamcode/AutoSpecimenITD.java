package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous
public class AutoSpecimenITD extends DriveMethods {

    double stateStartTime = -1;
    double stateStartPos = 0;
    double stateStartAngle = 0;


    enum State {
        Unstarted,
        TightenClaw,
        MoveForward,
        StrafeLeft,
        RaiseArm,
        ExtendSlider,
        ExtraMove,
        LowerArm,
        OpenClaw,
        SafeRaise,
        MoveBackward,
        Wait,
        ExtraRetractSlider,
        SecondBack,
        SecondStrafe,
        SecondForward,
        SecondRight,
        SecondPush,
        Finished,

    }

    State currentState = State.Unstarted;

    @Override
    public void init() {
        robot.init(hardwareMap);
    }

    @Override
    public void loop() {
        telemetry.addData("code", "running");
        telemetry.addData("time", "%.1f", getRuntime());
        telemetry.addData("encoder", "%.1f", (double) robot.leftFrontDrive.getCurrentPosition());
        telemetry.addData("imu", "%.1f", robot.imu.getRobotYawPitchRollAngles().getYaw());
        telemetry.addData("state", currentState);

        switch (currentState) {

            case Unstarted:
                changeState(State.TightenClaw);
                break;

            case TightenClaw:
                robot.clawServo.setPosition(1.20);
                changeState(State.MoveForward);
                break;

            case MoveForward:
                double remainingPos = moveStraightTo(0.5334);

                if (Math.abs(remainingPos) <= .01) {
                    omniDrive(0, 0, 0);
                    changeState(State.StrafeLeft);
                }
                break;


            case StrafeLeft:
                double remainingDistance = strafeTo(-0.25);

                if (Math.abs(remainingDistance) <= .01) {
                    changeState(State.RaiseArm);
                }
                break;


            case RaiseArm:

                // make sure to stay still
                moveStraightTo(0);

                robot.wormGear.setPower(1);

                if (robot.wormGearAngle() >= 65) {
                    robot.wormGear.setPower(0);

                    changeState(State.ExtendSlider);
                }
                break;

            case ExtendSlider:
                robot.sliderMotor.setPower(0.8);
                robot.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setSliderAndReturnConstraint(800);
                if (robot.sliderMotor.getCurrentPosition() >= 800) {
                    changeState(State.ExtraMove);
                }
                break;


            case ExtraMove:
                remainingPos = moveStraightTo(0.265);

                if (Math.abs(remainingPos) <= .01) {
                    omniDrive(0, 0, 0);
                    changeState(State.Wait);
                }
                break;


            case Wait:
            if (getStateTime() >= 1) {
                omniDrive(0, 0, 0);
                changeState(State.ExtraRetractSlider);
            }
            break;


            case ExtraRetractSlider:
                robot.sliderMotor.setPower(0.8);
                robot.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setSliderAndReturnConstraint(177.669773);

                if (robot.sliderMotor.getCurrentPosition() <= 177.669773) {
                    changeState(State.LowerArm);
                }
                break;


            case LowerArm:
                robot.wormGear.setPower(-0.7);

                if (robot.wormGearAngle() <= 50) {
                    robot.wormGear.setPower(0);

                    changeState(State.OpenClaw);
                }
                break;

            case OpenClaw:
                robot.clawServo.setPosition(robot.CLAW_OPEN);
                changeState(State.SafeRaise);
                break;


            case SafeRaise:
                robot.wormGear.setPower(0.7);

                if (robot.wormGearAngle() >= 60) {
                    robot.wormGear.setPower(0);

                    changeState(State.SecondBack);
                }
                break;


            case SecondBack:
                remainingPos = moveStraightTo(-0.3);

                if (Math.abs(remainingPos) <= .01) {
                    omniDrive(0, 0, 0);
                    changeState(State.SecondStrafe);
                }
                break;


            case SecondStrafe:
                remainingDistance = strafeTo(1.1);

                if (Math.abs(remainingDistance) <= .01) {
                    changeState(State.SecondForward);
                }
                break;


            case SecondForward:
                remainingPos = moveStraightTo(0.8);

                if (Math.abs(remainingPos) <= .01) {
                    omniDrive(0, 0, 0);
                    changeState(State.SecondRight);
                }
                break;


            case SecondRight:
                remainingDistance = strafeTo(0.3);

                if (Math.abs(remainingDistance) <= .01) {
                    changeState(State.SecondPush);
                }
                break;


            case SecondPush:
                remainingPos = moveStraightTo(-1);

                if (Math.abs(remainingPos) <= .01) {
                    omniDrive(0, 0, 0);
                    changeState(State.Finished);
                }
                break;
            case Finished:
                robot.wormGear.setPower(0);
                strafeTo(0);
                break;
        }
    }

    void changeState(State nextState) {
        currentState = nextState;
        stateStartTime = getRuntime();
        stateStartPos = position();
    }

    double getStateTime() {
        return getRuntime() - stateStartTime;
    }

    double position() {
        double MM_PER_METER = 1000;
        return robot.leftFrontDrive.getCurrentPosition() / robot.TICKS_PER_MM / MM_PER_METER;
    }

    double angle() {
        return  robot.imu.getRobotYawPitchRollAngles().getYaw();
    }

    double strafeTo(double targetDistance) {
        double distanceTravelled = position();
        double targetPos = stateStartPos + targetDistance;
        double remainingDistance = targetPos - distanceTravelled;
        double MAX_POWER = .5;


        telemetry.addData("remainingDistance", "%.2f", remainingDistance);

        double power = 3 * remainingDistance;

        if (power < -MAX_POWER) {
            power = -MAX_POWER;
        }
        if (power > MAX_POWER) {
            power = MAX_POWER;
        }

        omniDrive(0, power, 0);

        return remainingDistance;
    }

    double moveStraightTo(double targetDistance) {
        double distanceTravelled = position();
        double targetPos = stateStartPos + targetDistance;
        double remainingPos = targetPos - distanceTravelled;
        double MAX_POWER = .5;

        telemetry.addData("remainingDistance", "%.2f", remainingPos);

        double power = 3 * remainingPos;

        if (power < -MAX_POWER) {
            power = -MAX_POWER;
        }
        if (power > MAX_POWER) {
            power = MAX_POWER;
        }


        omniDrive(power, 0, 0);

        return remainingPos;
    }

    double turnTo(double targetAngleOffset) {
        double targetAngle = stateStartAngle + targetAngleOffset;
        double remainingAngle = targetAngle - angle();
        double MAX_POWER = .5;


        telemetry.addData("remainingDistance", "%.2f", remainingAngle);

        double power = 0.0075 * remainingAngle;

        if (power < -MAX_POWER) {
            power = -MAX_POWER;
        }
        if (power > MAX_POWER) {
            power = MAX_POWER;
        }

        omniDrive(0, 0, power);

        return remainingAngle;
    }
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

//    double targetTotalAngle()
//
//    {
//        double turnTo(double targetTotalAngle)
//    }
