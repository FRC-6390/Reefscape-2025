// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils.Experimental;

import java.util.List;
import frc.robot.Constants.EndEffector.WristState;
import frc.robot.Constants.EndEffector.ArmState;
import frc.robot.Constants.EndEffector.RollerState;


public enum SuperStructureStates
    {

        Intaking(WristState.Intaking, ArmState.Intaking, RollerState.Running),
        Aim(WristState.Aim, ArmState.Aim, RollerState.Stopped),

        Score(RollerState.Running),

        Home(WristState.Home, ArmState.Home, RollerState.Stopped);

        private List<Enum<?>> states;
        private SuperStructureStates(Enum<?>... states)
        {
            this.states = List.of(states);
        }

        public List<Enum<?>> getStates()
        {
            return states;
        }
    }
