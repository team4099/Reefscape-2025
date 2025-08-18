package com.team4099.robot2025.subsystems.intake

import com.team4099.robot2025.config.constants.IntakeConstants
import com.team4099.robot2025.subsystems.arm.Arm.Companion.ArmState
import com.team4099.robot2025.subsystems.intake.IntakeIOSim.setIntakeVoltage
import com.team4099.robot2025.subsystems.superstructure.Request
import edu.wpi.first.wpilibj.RobotBase
import edu.wpi.first.wpilibj2.command.SubsystemBase
import org.team4099.lib.units.derived.ElectricalPotential
import org.team4099.lib.units.derived.degrees
import org.team4099.lib.units.derived.volts

class Intake(private val io: IntakeIO): SubsystemBase() { //Class Intake, and every time you try creating an instance of this logic class, you pass in a parameter named IO of type IntakeIO (your blueprint interface)
    val inputs = IntakeIO.IntakeIOInputs() //Creates an instance of the inputs you created in IO (blueprint)

    var currentState: IntakeState = IntakeState.UNINITIALIZED //Sets the current state

    var intakeTargetVoltage: ElectricalPotential = 0.0.volts //sets the initial target voltage

    private var lastIntakePosition = -1337.0.degrees //has to be an unrealistic number, just sets up PID
    private var intakePositionTarget = 0.0.degrees // :000
//
    val isAtTargetedPosition: Boolean //checks if the current request is PID, and it is legal, and returns a boolean w the get()
        get() =
            (
                currentRequest is Request.IntakeRequest.ClosedLoop &&
                    (inputs.intakePosition - intakePositionTarget).absoluteValue <= IntakeConstants.INTAKE_TOLERANCE
                )

    var currentRequest: Request.IntakeRequest = Request.IntakeRequest.Zero() //this asks whether the currentRequest is an OpenLoop or ClosedLoop
        set(value) {
            when (value) {
                is Request.IntakeRequest.OpenLoop -> {
                    intakeTargetVoltage = value.voltage
                }

                is Request.IntakeRequest.ClosedLoop -> {
                    intakePositionTarget = value.position
                }

                else -> {}
            }
            field = value
        }

    init { //sets up our control system, open and closed loop
        if (RobotBase.isReal()) {
            IntakeTunableValues.kP.initDefault(IntakeConstants.PID.REAL_INTAKE_KP)
            IntakeTunableValues.kI.initDefault(IntakeConstants.PID.REAL_INTAKE_KI)
            IntakeTunableValues.kD.initDefault(IntakeConstants.PID.REAL_INTAKE_KD)
        } else {
            IntakeTunableValues.kP.initDefault(IntakeConstants.PID.SIM_PIVOT_KP)
            IntakeTunableValues.kI.initDefault(IntakeConstants.PID.SIM_PIVOT_KI)
            IntakeTunableValues.kD.initDefault(IntakeConstants.PID.SIM_PIVOT_KD)
        }
        IntakeTunableValues.kS.initDefault(IntakeConstants.PID.INTAKE_KS)
        IntakeTunableValues.kA.initDefault(IntakeConstants.PID.INTAKE_KA)
        IntakeTunableValues.kV.initDefault(IntakeConstants.PID.INTAKE_KV)
        IntakeTunableValues.kG.initDefault(IntakeConstants.PID.INTAKE_KG) //Default cs gravity
    } //sets up PID

    override fun periodic() {
        io.updateInputs(inputs) //this is where the motor's status signal values are updated into the actual vars

        //In the case your PID or FF values change, you can set them right back up, since periodic runs every 20 ms.
        val hasPIDChanged: Boolean =
            IntakeTunableValues.kP.hasChanged() ||
                IntakeTunableValues.kI.hasChanged() ||
                IntakeTunableValues.kD.hasChanged()
        val hasFFChanged: Boolean =
            IntakeTunableValues.kS.hasChanged() ||
                IntakeTunableValues.kA.hasChanged() ||
                IntakeTunableValues.kV.hasChanged() ||
                IntakeTunableValues.kG.hasChanged()

        if (hasPIDChanged) {
            io.configPID(
                IntakeTunableValues.kP.get(),
                IntakeTunableValues.kI.get(),
                IntakeTunableValues.kD.get()
            )
        }
        if (hasFFChanged) {
            io.configFF(
                IntakeTunableValues.kG.get(),
                IntakeTunableValues.kS.get(),
                IntakeTunableValues.kV.get(),
                IntakeTunableValues.kA.get()

            )
        }
        var nextState: IntakeState = currentState //updates the currentState

        when (currentState) {
            IntakeState.UNINITIALIZED -> {
                nextState = fromRequestToState(currentRequest)
            }
            IntakeState.ZERO -> {
                io.zeroEncoder()
                nextState = fromRequestToState(currentRequest)
            }
            IntakeState.OPEN_LOOP -> {
                io.setIntakeVoltage(intakeTargetVoltage)
                nextState = fromRequestToState(currentRequest)
            }
            IntakeState.CLOSED_LOOP -> {
                io.setIntakePosition(intakePositionTarget)
                nextState = fromRequestToState(currentRequest)
            }

        }
    }
    companion object { //creates an object within a class
        enum class IntakeState { //These are all the states of Intake, and it's like a drop-down menu
            UNINITIALIZED,
            OPEN_LOOP,
            CLOSED_LOOP,
            ZERO
        }

        fun fromRequestToState(request: Request.IntakeRequest): IntakeState { //Superstructures State machine will dictate what state the intake should be at in order to achieve the actionable
            return when (request) {
                // is Request.ArmRequest.TorqueControl -> ArmState.TORQUE_CURRENT
                is Request.IntakeRequest.OpenLoop -> IntakeState.OPEN_LOOP
                is Request.IntakeRequest.ClosedLoop -> IntakeState.CLOSED_LOOP
                is Request.IntakeRequest.Uninitialized -> IntakeState.UNINITIALIZED
                is Request.IntakeRequest.Zero -> IntakeState.ZERO
            }
        }
    }
}


//State Machine does 2 things. 1. Keeps track of the current state. (Uninitialized, preheating, staying at it's temp) 2. Handling requests, and switching between states accordingly.
//Actionable states are within Superstructure's State machine
//Basic States are within logic files