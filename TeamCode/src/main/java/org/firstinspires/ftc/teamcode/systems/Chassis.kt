package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit

class MecanumMotorValues {
    var topLeftAngularSpeed = 0.0
    var topRightAngularSpeed = 0.0
    var downLeftAngularSpeed = 0.0
    var downRightAngularSpeed = 0.0
}

class Twist2D {
    var vx = 0.0
    var vy = 0.0
    var w = 0.0

}

class MecanumKinematics(var xDistanceFromWheelToCenter: Double, var yDistanceFromWheelToCenter: Double, var wheelRadius: Double) {

    fun calcInverseKinematics(vx: Double, vy: Double, w: Double): MecanumMotorValues {
        val values = MecanumMotorValues()
        values.topLeftAngularSpeed = (vx - vy - xDistanceFromWheelToCenter * w - yDistanceFromWheelToCenter * w) * (1.0/wheelRadius)
        values.topRightAngularSpeed = (vx + vy + xDistanceFromWheelToCenter * w + yDistanceFromWheelToCenter * w) * (1.0/wheelRadius)
        values.downLeftAngularSpeed = (vx + vy - xDistanceFromWheelToCenter * w - yDistanceFromWheelToCenter * w) * (1.0/wheelRadius)
        values.downRightAngularSpeed = (vx - vy + xDistanceFromWheelToCenter * w + yDistanceFromWheelToCenter * w) * (1.0/wheelRadius)
        return values

    }

    fun calcForwardKinematics(state: MecanumMotorValues): Twist2D {
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

class Chassis (var hardwareMap: HardwareMap){
    fun move(vx: Double, vy: Double, w: Double){
        val values= kinematics.calcInverseKinematics(vx, vy, w)
        writeMotors(values)
    }
    private fun writeMotors(values:MecanumMotorValues){
        topLeftMotor.setVelocity(values.topLeftAngularSpeed, AngleUnit.RADIANS)
        topRightMotor.setVelocity(values.topRightAngularSpeed, AngleUnit.RADIANS)
        bottomLeftMotor.setVelocity(values.downLeftAngularSpeed, AngleUnit.RADIANS)
        bottomRightMotor.setVelocity(values.downRightAngularSpeed, AngleUnit.RADIANS)
    }

    private var topLeftMotor = hardwareMap.dcMotor.get("topLeft") as DcMotorEx
    private var topRightMotor = hardwareMap.dcMotor.get("topRight") as DcMotorEx
    private var bottomLeftMotor = hardwareMap.dcMotor.get("bottomLeft") as DcMotorEx
    private var bottomRightMotor = hardwareMap.dcMotor.get("bottomRight") as DcMotorEx

    private var kinematics = MecanumKinematics(0.5, 0.5, 1.0)
}
