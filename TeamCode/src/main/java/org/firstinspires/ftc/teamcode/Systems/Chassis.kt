package org.firstinspires.ftc.teamcode.Systems

import org.firstinspires.ftc.teamcode.core.Controller

class MecanumMotorState {
    public var topLeftAngularSpeed = 0.0
    public var topRightAngularSpeed = 0.0
    public var downLeftAngularSpeed = 0.0
    public var downRightAngularSpeed = 0.0

}

class MecanumMotorValues {
    public var topLeftSpeed = 0.0
    public var topRightSpeed = 0.0
    public var downLeftSpeed = 0.0
    public var downRightSpeed = 0.0
}

class Twist2D {
    public var vx = 0.0
    public var vy = 0.0
    public var w = 0.0

}

class MecanumKinematics(var xDistanceFromWheelToCenter: Double, var yDistanceFromWheelToCenter: Double, var wheelRadius: Double) {

    public fun calcInverseKinematics(vx: Double, vy: Double, w: Double): MecanumMotorValues {
        val values = MecanumMotorValues()
        values.topLeftSpeed = vx - vy - xDistanceFromWheelToCenter * w - yDistanceFromWheelToCenter * w
        values.topRightSpeed = vx + vy + xDistanceFromWheelToCenter * w + yDistanceFromWheelToCenter * w
        values.downLeftSpeed = vx + vy - xDistanceFromWheelToCenter * w - yDistanceFromWheelToCenter * w
        values.downRightSpeed = vx - vy + xDistanceFromWheelToCenter * w + yDistanceFromWheelToCenter * w
        return values

    }

    public fun calcForwardKinematics(state: MecanumMotorState): Twist2D {
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
    public fun move(vx: Double, vy: Double, w: Double){
        val values= kinematics.calcInverseKinematics(vx, vy, w)
        writeMotors(values)
    }
    fun writeMotors(values:MecanumMotorValues){

    }
    var kinematics = MecanumKinematics(0.5, 0.5, 1.0)
}
