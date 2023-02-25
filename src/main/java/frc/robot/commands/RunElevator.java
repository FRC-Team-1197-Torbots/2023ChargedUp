package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Elevator;

public class RunElevator extends CommandBase{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})

    private Arm arm;
    private Claw claw;
    private Elevator elevator;
    
    public RunElevator(Arm armSystem, Claw clawSystem, Elevator elevatorSystem){
        arm = armSystem;
        claw = clawSystem;
        elevator = elevatorSystem;
        addRequirements(armSystem, clawSystem, elevatorSystem);
    }

    
}
