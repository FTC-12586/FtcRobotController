package org.firstinspires.ftc.teamcode.src.utills;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.qualcomm.robotcore.hardware.Gamepad;

/**
This designates a class as able to be controlled through the gamepads. It is a parametrized interface where the return type MAY be representative of the state of the subsystem.
 For example, a sensor that can scan on a button press may return it's sensed value through the {@link Controllable#gamepadControl(Gamepad, Gamepad)} method.
 */
public interface Controllable<T> {

    /**
     * Allows control of subsystems through the gamepads. MAY return the state of the subsystem.
     *
     * @param gamepad1 The first gamepad
     * @param gamepad2 The second gamepad
     *
     * @throws InterruptedException Throws if the thread is interrupted during execution
     *
     * @return A Object representative of the state of the subsystem
     */
    @Nullable
    T gamepadControl(@NonNull Gamepad gamepad1, @NonNull Gamepad gamepad2) throws InterruptedException;


    /**
     * Stops the controllable object from moving.
     */
    void halt();
}
