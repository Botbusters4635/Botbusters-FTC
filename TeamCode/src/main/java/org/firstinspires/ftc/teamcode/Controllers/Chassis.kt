package org.firstinspires.ftc.teamcode.systems

import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.*
import kotlinx.coroutines.*
import kotlinx.coroutines.channels.ReceiveChannel
import kotlinx.coroutines.channels.SendChannel
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.Core.Twist2D

import org.firstinspires.ftc.teamcode.core.Controller
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings



data class MecanumMotorState(var topLeftAngularSpeed: Double = 0.0, var topRightAngularSpeed: Double = 0.0, var downLeftAngularSpeed: Double = 0.0, var downRightAngularSpeed: Double = 0.0)

data class MecanumMotorValues(var topLeftSpeed: Double = 0.0, var topRightSpeed: Double = 0.0, var downLeftSpeed: Double = 0.0, var downRightSpeed: Double = 0.0)


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
    private val scope = CoroutineScope(Job())
    private lateinit var topLeftMotor: DcMotorEx
    private lateinit var topRightMotor: DcMotorEx
    private lateinit var downRightMotor: DcMotorEx
    private lateinit var downLeftMotor: DcMotorEx

    private lateinit var angularPID: PID

    private lateinit var imu: BNO055IMU

    private var kinematics = MecanumKinematics(0.5, 0.5, 1.0)

    var movementTarget = Twist2D()
        set(value) {
            field = value
            angularPID.target = field.w
        }


    override fun init(hardwareMap: HardwareMap) {
        // Get motors and cast to DcMotorEx
        topLeftMotor = hardwareMap.get(DcMotor::class.java, "topLeftMotor") as DcMotorEx
        topRightMotor = hardwareMap.get(DcMotor::class.java, "topRightMotor") as DcMotorEx
        downLeftMotor = hardwareMap.get(DcMotor::class.java, "downLeftMotor") as DcMotorEx
        downRightMotor = hardwareMap.get(DcMotor::class.java, "downRightMotor") as DcMotorEx
//
////        val pidSettings: PIDFCoefficients = PIDFCoefficients(10.0, 1.0, 0.0, 0.0)
//
////        topLeftMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidSettings)
//
        // Change mode to run by velocity ( RUN_USING_ENCODER)
        topLeftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        topRightMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        downRightMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER
        downLeftMotor.mode = DcMotor.RunMode.RUN_USING_ENCODER

        // Reverse motors
        topLeftMotor.direction = DcMotorSimple.Direction.REVERSE
        downLeftMotor.direction = DcMotorSimple.Direction.REVERSE

        // Get the IMU, configure it and initialize it.
        imu = hardwareMap.get(BNO055IMU::class.java, "imu")
        val parameters = BNO055IMU.Parameters()
        parameters.mode = BNO055IMU.SensorMode.NDOF
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES
        imu.initialize(parameters)

        // Initialize the angularPID
        val pidSettings = PIDSettings(kP = 0.05, kI = 0.0, kD = 0.0035, continous = false, lowerBound = -180.0, upperBound = 180.0)
        angularPID = PID(pidSettings)
    }

    @ExperimentalCoroutinesApi
    override fun start() {
        scope.launch { headingProducer(angularPID.inputChannel) }
        scope.launch { angularPID.start() }
        scope.launch { motorReceiver(angularPID.outputChannel) }
    }

    override fun stop() {
        scope.cancel()
    }

    @ExperimentalCoroutinesApi
    suspend fun headingProducer(sendChannel: SendChannel<Double>) {
        while (scope.isActive) {
            sendChannel.send(getHeading())
        }
    }

    suspend fun motorReceiver(angularVelocityChannel: ReceiveChannel<Double>){
        while (scope.isActive) {
            telemetry.addData("PID Error", angularPID.error)
            telemetry.addData("PID Target", angularPID.target)
            telemetry.addData("targetW", movementTarget.w)
            val motorValues = kinematics.calcInverseKinematics(movementTarget.vx, movementTarget.vy, angularVelocityChannel.receive())
            writeMotors(motorValues)
        }

    }

    fun getHeading(): Double {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle.toDouble()
    }

    fun moveWithoutPID(vx: Double, vy: Double, w: Double) {
        val values = kinematics.calcInverseKinematics(vx, vy, w)
        writeMotors(values)
    }

    fun writeMotors(values: MecanumMotorValues) {
        topLeftMotor.power = values.topLeftSpeed
        topRightMotor.power = values.topRightSpeed
        downLeftMotor.power = values.downLeftSpeed
        downRightMotor.power = values.downRightSpeed
    }

}
