package org.firstinspires.ftc.teamcode.Controllers

import android.os.SystemClock
import com.qualcomm.hardware.bosch.BNO055IMU
import com.qualcomm.robotcore.hardware.*
import kotlinx.coroutines.*
import kotlinx.coroutines.channels.ReceiveChannel
import kotlinx.coroutines.channels.SendChannel
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference
import org.firstinspires.ftc.teamcode.core.Twist2D
import org.firstinspires.ftc.teamcode.core.Controller
import org.firstinspires.ftc.teamcode.core.PID
import org.firstinspires.ftc.teamcode.core.PIDSettings

data class MecanumMotorValues(var topLeftSpeed: Double = 0.0, var topRightSpeed: Double = 0.0, var downLeftSpeed: Double = 0.0, var downRightSpeed: Double = 0.0){
    operator fun plus(other : MecanumMotorValues) : MecanumMotorValues {
        val result = MecanumMotorValues()
        result.topLeftSpeed = this.topLeftSpeed + other.topLeftSpeed
        result.topRightSpeed = this.topRightSpeed + other.topRightSpeed
        result.downLeftSpeed = this.downLeftSpeed + other.downLeftSpeed
        result.downRightSpeed = this.downRightSpeed + other.downRightSpeed
        return result
    }
    operator fun minus(other : MecanumMotorValues) : MecanumMotorValues {
        val result = MecanumMotorValues()
        result.topLeftSpeed = this.topLeftSpeed - other.topLeftSpeed
        result.topRightSpeed = this.topRightSpeed - other.topRightSpeed
        result.downLeftSpeed = this.downLeftSpeed - other.downLeftSpeed
        result.downRightSpeed = this.downRightSpeed - other.downRightSpeed
        return result
    }
}


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


class Chassis : Controller() {
    private val scope = CoroutineScope(Job())
    private lateinit var topLeftMotor: DcMotorEx
    private lateinit var topRightMotor: DcMotorEx
    private lateinit var downRightMotor: DcMotorEx
    private lateinit var downLeftMotor: DcMotorEx

    private lateinit var angularPID: PID
    private lateinit var vyPID: PID

    private var lastMotorUpdate = SystemClock.elapsedRealtime() / 1000.0
    private val maxMotorOutputChangePerSecond = 0.1
    private  val maxMotorUpdateRate = 0.01
    private lateinit var imu: BNO055IMU


    private var currentMotorState = MecanumMotorValues()

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

        // Change mode to run by velocity ( RUN_USING_ENCODER)
        topLeftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        topRightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        downRightMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        downLeftMotor.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

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

        val pidSettings = PIDSettings(kP = 0.0175, kI = 0.0, kD = 0.0005, continous = true, lowerBound = -180.0, upperBound = 180.0)
        angularPID = PID(pidSettings)

    }

    @ExperimentalCoroutinesApi
    override fun start() {
        scope.launch { headingProducer(angularPID.inputChannel) }
        scope.launch { angularPID.start() }
        scope.launch { vyPID.start() }
        scope.launch { motorVelocityReceiver(angularPID.outputChannel) }
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


    suspend fun motorVelocityReceiver(angularVelocityChannel: ReceiveChannel<Double>) {
        while (scope.isActive) {
            val motorValues = kinematics.calcInverseKinematics(movementTarget.vx, movementTarget.vy, angularVelocityChannel.receive())
            writeMotors(motorValues)
        }

    }


    fun getHeading(): Double {
        return imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle.toDouble()
    }
//
//    fun moveWithoutPID(vx: Double, vy: Double, w: Double) {
//        val values = kinematics.calcInverseKinematics(vx, vy, w)
//        writeMotors(values)
//    }

    suspend fun writeMotors(values: MecanumMotorValues) {
        var currentTime = SystemClock.elapsedRealtime() / 1000.0
        var timeStep = currentTime - lastMotorUpdate

        if(timeStep  < maxMotorUpdateRate){
            return
        }

        var maxChange = maxMotorOutputChangePerSecond * timeStep

        var desiredChange = currentMotorState - values

        telemetry.addData("TimeStep", timeStep)
        telemetry.addData("Change", desiredChange.topLeftSpeed * timeStep)
        telemetry.addData("Max Allowed Change", maxChange)

        if(Math.abs(desiredChange.topLeftSpeed * timeStep) >  maxChange){
            desiredChange.topLeftSpeed = Math.copySign(maxChange, desiredChange.topLeftSpeed)
        }

        if(Math.abs(desiredChange.topRightSpeed * timeStep) >  maxChange){
            desiredChange.topRightSpeed = Math.copySign(maxChange, desiredChange.topRightSpeed)
        }

        if(Math.abs(desiredChange.downLeftSpeed * timeStep) >  maxChange){
            desiredChange.downLeftSpeed = Math.copySign(maxChange, desiredChange.downLeftSpeed)
        }

        if(Math.abs(desiredChange.downRightSpeed * timeStep) >  maxChange){
            desiredChange.downRightSpeed = Math.copySign(maxChange, desiredChange.downRightSpeed)
        }

        val currentMotorOutput = MecanumMotorValues()
        currentMotorOutput.topLeftSpeed = currentMotorState.topLeftSpeed + desiredChange.topLeftSpeed
        currentMotorOutput.topRightSpeed = currentMotorState.topRightSpeed + desiredChange.topRightSpeed
        currentMotorOutput.downLeftSpeed = currentMotorState.downLeftSpeed + desiredChange.downLeftSpeed
        currentMotorOutput.downRightSpeed = currentMotorState.downRightSpeed + desiredChange.downRightSpeed

        topLeftMotor.power = currentMotorOutput.topLeftSpeed
        topRightMotor.power = currentMotorOutput.topRightSpeed
        downLeftMotor.power = currentMotorOutput.topRightSpeed
        downRightMotor.power = currentMotorOutput.downRightSpeed


        currentMotorState = currentMotorOutput

        lastMotorUpdate = SystemClock.elapsedRealtime()  / 1000.0
    }
}
