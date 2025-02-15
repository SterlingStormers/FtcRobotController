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
        MoveForward,
        StrafeLeft,
        RaiseArm,
        ExtendSlider,
        ExtraMove,
        LowerArm,
        OpenClaw,
        ExtraRaise,
        MoveBackward,
        ExtraLowerArm,
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
                double remainingDistance = strafeTo(-0);

                if (Math.abs(remainingDistance) <= .01) {
                    changeState(State.RaiseArm);
                }
                break;


            case RaiseArm:
                robot.wormGear.setPower(1);

                // make sure to stay still
                moveStraightTo(0);

                if (robot.wormGearAngle() >= 0) {
                    robot.wormGear.setPower(0);

                    changeState(State.ExtendSlider);
                }
                break;

            case ExtendSlider:
                robot.sliderMotor.setPower(0.8);
                robot.sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                setSliderAndReturnConstraint(2700);

                if (robot.sliderMotor.getCurrentPosition() >= 2700) {
                    changeState(State.ExtraMove);
                }
                break;


            case ExtraMove:
                remainingPos = moveStraightTo(0.0);

                if (Math.abs(remainingPos) <= .01) {
                    omniDrive(0, 0, 0);
                    changeState(State.LowerArm);
                }
                break;


            case LowerArm:
                robot.wormGear.setPower(0.0);

                if (robot.wormGearAngle() >= 78) {
                    robot.wormGear.setPower(0);

                    changeState(State.OpenClaw);
                }
                break;

            case OpenClaw:
                robot.clawServo.setPosition(robot.CLAW_OPEN);
                changeState(State.ExtraRaise);
                break;

            case ExtraRaise:
                robot.wormGear.setPower(0.0);

                if (robot.wormGearAngle() >= 0) {
                    robot.wormGear.setPower(0);

                    changeState(State.MoveBackward);
                }
                break;

            case MoveBackward:
                double remaining = moveStraightTo(-0.0);

                if (Math.abs(remaining) <= .01) {
                    omniDrive(0, 0, 0);
                    changeState(State.ExtraLowerArm);
                }
                break;

            case ExtraLowerArm:
                robot.wormGear.setPower(0.0);

                if (robot.wormGearAngle() >= 78) {
                    robot.wormGear.setPower(0);

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
