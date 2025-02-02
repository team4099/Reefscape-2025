package com.team4099.robot2025.commands.characterization

import com.team4099.robot2025.subsystems.elevator.Elevator
import com.team4099.robot2025.subsystems.elevator.ElevatorTunableValues
import com.team4099.robot2025.subsystems.superstructure.Request
import edu.wpi.first.wpilibj2.command.Command
import org.team4099.lib.units.derived.volts

class TestElevatorCommand(
    val elevator: Elevator
) : Command() {
    init {
        addRequirements(elevator)
    }

    override fun execute() {
        elevator.currentRequest = Request.ElevatorRequest.OpenLoop(ElevatorTunableValues.testingVoltage.get())
    }

    override fun end(interrupted: Boolean) {
        elevator.currentRequest = Request.ElevatorRequest.OpenLoop(0.volts)
    }

    override fun isFinished(): Boolean {
        return false
    }
}