package org.firstinspires.ftc.teamcode.controllers.chassis

import org.firstinspires.ftc.teamcode.core.Twist2D

data class MecanumMotorValues(var topLeftSpeed: Double = 0.0, var topRightSpeed: Double = 0.0, var downLeftSpeed: Double = 0.0, var downRightSpeed: Double = 0.0) {
    operator fun plus(other: MecanumMotorValues): MecanumMotorValues {
        val result = MecanumMotorValues()
        result.topLeftSpeed = this.topLeftSpeed + other.topLeftSpeed
        result.topRightSpeed = this.topRightSpeed + other.topRightSpeed
        result.downLeftSpeed = this.downLeftSpeed + other.downLeftSpeed
        result.downRightSpeed = this.downRightSpeed + other.downRightSpeed
        return result
    }

    operator fun minus(other: MecanumMotorValues): MecanumMotorValues {
        val result = MecanumMotorValues()
        result.topLeftSpeed = this.topLeftSpeed - other.topLeftSpeed
        result.topRightSpeed = this.topRightSpeed - other.topRightSpeed
        result.downLeftSpeed = this.downLeftSpeed - other.downLeftSpeed
        result.downRightSpeed = this.downRightSpeed - other.downRightSpeed
        return result
    }
}

data class MecanumMoveCommand(var vx: Double = 0.0, var vy: Double = 0.0, var theta: Double = 0.0)

class MecanumKinematics(var xDistanceFromWheelToCenter: Double, var yDistanceFromWheelToCenter: Double, var wheelRadius: Double) {

    fun calcInverseKinematics(vx: Double, vy: Double, w: Double): MecanumMotorValues {
        val values = MecanumMotorValues()
        values.topLeftSpeed = (vx - vy - xDistanceFromWheelToCenter * w - yDistanceFromWheelToCenter * w) * (1.0 / wheelRadius)
        values.topRightSpeed = (vx + vy + xDistanceFromWheelToCenter * w + yDistanceFromWheelToCenter * w) * (1.0 / wheelRadius)
        values.downLeftSpeed = (vx + vy - xDistanceFromWheelToCenter * w - yDistanceFromWheelToCenter * w) * (1.0 / wheelRadius)
        values.downRightSpeed = (vx - vy + xDistanceFromWheelToCenter * w + yDistanceFromWheelToCenter * w) * (1.0 / wheelRadius)
        return values

    }


    fun calcForwardKinematics(state: MecanumMotorValues): Twist2D {
        val result = Twist2D()

        result.vx = (state.topLeftSpeed + state.topRightSpeed + state.downLeftSpeed + state.downRightSpeed) * wheelRadius / 4.0
        result.vy = (-state.topLeftSpeed + state.topRightSpeed + state.downLeftSpeed - state.downRightSpeed) * wheelRadius / 4.0
        result.w = (-state.topLeftSpeed + state.topRightSpeed - state.downLeftSpeed + state.downRightSpeed) * wheelRadius / 4.0 * (xDistanceFromWheelToCenter + yDistanceFromWheelToCenter)
        return result
    }
}