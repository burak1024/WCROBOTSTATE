package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volt;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import frc.robot.lib.Subsystem;

public class IntakeSubsystem extends Subsystem {
    private final TalonFX intakeMotorR = new TalonFX(0);
    private final TalonFX intakeMotorL = new TalonFX(0);
    private final TalonFX rollerMotorR = new TalonFX(0);
    private final TalonFX rollerMotorL = new TalonFX(0);
    private final MotionMagicVoltage Motion = new MotionMagicVoltage(0).withEnableFOC(true);
    private final VoltageOut Voltage = new VoltageOut(0).withEnableFOC(true);
    private boolean isRolling = false;

    private static IntakeSubsystem instance;

    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    public static IntakeSubsystem subsystem() {
        return getInstance();
    }

    private state currentState = state.idle;

    public enum state {
        idle(0),
        intaking(1),
        unjamming(2),
        shaking(3);

        public final int stateNum;

        state(int stateNum) {
            this.stateNum = stateNum;
        }
    }

    public Runnable[][] methods = {
            {
                    () -> emptyMethod(),
                    () -> emptyMethod(),
                    () -> emptyMethod()
            },
            {
                    () -> intakingIMethods(),
                    () -> intakingPMethods(),
                    () -> intakingEMethods()
            },
            {
                    () -> emptyMethod(),
                    () -> unjammingPMethods(),
                    () -> unjammingEMethods()
            },
            {
                    () -> emptyMethod(),
                    () -> shakingPMethods(),
                    () -> shakingEMethods(),
            }

    };

    public void changeState(state newState) {
        if (this.currentState == newState)
            return;
        methods[currentState.stateNum][2].run();
        currentState = newState;
        methods[currentState.stateNum][0].run();
        super.activeStatePeriodic = methods[newState.stateNum][1];
    }

    private void intakingIMethods() {
        intakeMotorL.setControl(Motion.withPosition(intakeConstants.INTAKE_POSES[4]));
        intakeMotorR.setControl(Motion.withPosition(intakeConstants.INTAKE_POSES[4]));
    }

    private void intakingPMethods() {
        roll();
        setIntakeState();
    }

    private void intakingEMethods() {
        stop();
    }

    private void unjammingPMethods() {
        rollerMotorL.setControl(Voltage.withOutput(0));
        rollerMotorR.setControl(Voltage.withOutput(0));
    }

    private void unjammingEMethods() {
        rollerMotorL.stopMotor();
        rollerMotorR.stopMotor();
    }
    private void shakingPMethods(){
        shakeIntake();
    }
    private void shakingEMethods(){
        stop();
    }

    private void roll() {
        if (isOnThePos()) {
            rollerMotorL.setControl(Voltage.withOutput(7.0));
            rollerMotorR.setControl(Voltage.withOutput(7.0));
        }
    }

    private boolean isJammed() {
        return rollerMotorL.getStatorCurrent().getValueAsDouble() > intakeConstants.JAMMED_AMPER_LIMIT
                || rollerMotorR.getStatorCurrent().getValueAsDouble() > intakeConstants.JAMMED_AMPER_LIMIT;
    }

    public void setIntakeState() {
        if (isJammed()) {
            changeState(state.unjamming);
        } else
            changeState(state.intaking);
    }

    public void shakeIntake() {
        if (getIntakePos() < intakeConstants.INTAKE_POSES[1] + 0.3) {
            intakeMotorL.setControl(Motion.withPosition(intakeConstants.INTAKE_POSES[4]));
            intakeMotorR.setControl(Motion.withPosition(intakeConstants.INTAKE_POSES[4]));
        } else if (getIntakePos() > intakeConstants.INTAKE_POSES[3] - 0.3) {
            intakeMotorR.setControl(Motion.withPosition(intakeConstants.INTAKE_POSES[0]));
            intakeMotorL.setControl(Motion.withPosition(intakeConstants.INTAKE_POSES[0]));
        }
    }

    private boolean isOnThePos() {
        return intakeConstants.INTAKE_POSES[4] - 0.3 <= getIntakePos();
    }

    private double getIntakePos() {
        return intakeMotorL.getPosition().getValueAsDouble();
    }

    @Override
    public void stop() {
        rollerMotorL.stopMotor();
        rollerMotorR.stopMotor();
        intakeMotorL.setPosition(0);
        intakeMotorR.setPosition(0);
    }
}
