package org.firstinspires.ftc.teamcode.systems


import org.firstinspires.ftc.teamcode.Core.Controller

data class MecanumMotorState(var topLeftAngularSpeed: Double = 0.0, var topRightAngularSpeed: Double = 0.0, var downLeftAngularSpeed: Double = 0.0, var downRightAngularSpeed: Double = 0.0)

data class MecanumMotorValues(var topLeftSpeed: Double = 0.0, var topRightSpeed: Double = 0.0, var downLeftSpeed: Double = 0.0, var downRightSpeed: Double = 0.0)

data class Twist2D(var vx: Double = 0.0, var vy: Double = 0.0, var w: Double = 0.0)


class MecanumKinematics(var xDistanceFromWheelToCenter: Double, var yDistanceFromWheelToCenter: Double, var wheelRadius: Double) {

    fun calcInverseKinematics(vx: Double, vy: Double, w: Double): MecanumMotorValues {
        val values = MecanumMotorValues()
        values.topLeftSpeed = (vx - vy - xDistanceFromWheelToCenter * w - yDistanceFromWheelToCenter * w) * (1.0/wheelRadius)
        values.topRightSpeed = (vx + vy + xDistanceFromWheelToCenter * w + yDistanceFromWheelToCenter * w) * (1.0/wheelRadius)
        values.downLeftSpeed = (vx + vy - xDistanceFromWheelToCenter * w - yDistanceFromWheelToCenter * w) * (1.0/wheelRadius)
        values.downRightSpeed = (vx - vy + xDistanceFromWheelToCenter * w + yDistanceFromWheelToCenter * w) * (1.0/wheelRadius)
        return values

    }


    fun calcForwardKinematics(state: MecanumMotorState): Twist2D {
        val result = Twist2D()

        result.vx = (state.topLeftAngularSpeed + state.topRightAngularSpeed + state.downLeftAngularSpeed + state.downRightAngularSpeed) * wheelRadius / 4.0
        result.vy = (-state.topLeftAngularSpeed + state.topRightAngularSpeed + state.downLeftAngularSpeed - state.downRightAngularSpeed) * wheelRadius / 4.0
        result.w = (-state.topLeftAngularSpeed + state.topRightAngularSpeed - state.downLeftAngularSpeed + state.downRightAngularSpeed) * wheelRadius / 4.0 * (xDistanceFromWheelToCenter + yDistanceFromWheelToCenter)
        return result
    }
}

/**
 * moveBy(vx, vy, w)
 *
 */


class Chassis : Controller() {
    fun move(vx: Double, vy: Double, w: Double) {
        val values = kinematics.calcInverseKinematics(vx, vy, w)
        writeMotors(values)
    }

    fun writeMotors(values: MecanumMotorValues) {

    }

    var kinematics = MecanumKinematics(0.5, 0.5, 1.0)
}
