package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.Core.Controller

data class MecanumMotorState(var topLeftAngularSpeed: Double = 0.0, var topRightAngularSpeed: Double = 0.0, var downLeftAngularSpeed: Double = 0.0, var downRightAngularSpeed: Double = 0.0)

data class MecanumMotorValues(var topLeftSpeed: Double = 0.0, var topRightSpeed: Double = 0.0, var downLeftSpeed: Double = 0.0, var downRightSpeed: Double = 0.0)

data class Twist2D(var vx: Double = 0.0, var vy: Double = 0.0, var w: Double = 0.0)


class MecanumKinematics(var xDistanceFromWheelToCenter: Double, var yDistanceFromWheelToCenter: Double, var wheelRadius: Double) {

    fun calcInverseKinematics(vx: Double, vy: Double, w: Double): MecanumMotorValues {
        val values = MecanumMotorValues()
        values.topLeftSpeed = (vx - vy - xDistanceFromWheelToCenter * w - yDistanceFromWheelToCenter * w) * (1.0 / wheelRadius)
        values.topRightSpeed = (vx + vy + xDistanceFromWheelToCenter * w + yDistanceFromWheelToCenter * w) * (1.0 / wheelRadius)
        values.downLeftSpeed = (vx + vy - xDistanceFromWheelToCenter * w - yDistanceFromWheelToCenter * w) * (1.0 / wheelRadius)
        values.downRightSpeed = (vx - vy + xDistanceFromWheelToCenter * w + yDistanceFromWheelToCenter * w) * (1.0 / wheelRadius)
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


class Chassis : Controller() {
    lateinit var topLeftMotor: DcMotor
    lateinit var topRightMotor: DcMotor
    lateinit var downRightMotor: DcMotor
    lateinit var downLeftMotor: DcMotor
    lateinit var imu: BNO055IMU

    var kinematics = MecanumKinematics(0.5, 0.5, 1.0)


    override fun init(hardwareMap: HardwareMap) {
        topLeftMotor = hardwareMap.get(DcMotor::class.java, "topLeftMotor")
        topRightMotor = hardwareMap.get(DcMotor::class.java, "topRightMotor")
        downLeftMotor = hardwareMap.get(DcMotor::class.java, "downLeftMotor")
        downRightMotor = hardwareMap.get(DcMotor::class.java, "downRightMotor")

        topLeftMotor.direction = DcMotorSimple.Direction.REVERSE
        downLeftMotor.direction = DcMotorSimple.Direction.REVERSE

        imu = hardwareMap.get(BNO055IMU::class.java, "imu")

        val parameters = BNO055IMU.Parameters()
        parameters.mode = BNO055IMU.SensorMode.NDOF
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES

        imu.initialize(parameters)
    }

    fun move(twist2D: Twist2D) {
        val values = kinematics.calcInverseKinematics(twist2D.vx, twist2D.vy, twist2D.w)
        writeMotors(values)
    }

    fun getHeading(): Double {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle.toDouble()
    }

    fun writeMotors(values: MecanumMotorValues) {
        topLeftMotor.power = values.topLeftSpeed
        topRightMotor.power = values.topRightSpeed
        downLeftMotor.power = values.downLeftSpeed
        downRightMotor.power = values.downRightSpeed
    }

}