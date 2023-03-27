package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

public class IntakeOutElevatorUp extends CommandBase{
    private Intake m_intake;
    private Elevator m_Elevator;

    public IntakeOutElevatorUp(Intake intake, Elevator elevator){
        m_intake = intake;
        m_Elevator = elevator;
    }

    @Override
    public void initialize(){

    }

    @Override
    public void execute(){
        System.out.println("Running Up");
        System.out.println("Elevator Pos: " + m_Elevator.GetElevatorPos());
        m_intake.SetSolenoid(true);

    }

    @Override
    public void end(boolean interrupted){
        System.out.println("Done top");
        m_intake.SetSolenoid(false);

    }

    @Override
    public boolean isFinished(){
        return m_Elevator.GetElevatorPos() > 500;
    }

    
}
