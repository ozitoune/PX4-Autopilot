/****************************************************************************
 *
 *   Copyright (C) 2023 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include <gtest/gtest.h>
#include <ControlAllocationSequentialDesaturation.hpp>

TEST(ControlAllocationSequentialDesaturationTest, AllZeroCase)
{
	ControlAllocationSequentialDesaturation control_allocation;
	EXPECT_EQ(control_allocation.getActuatorSetpoint(), (matrix::Vector<float, ControlAllocation::NUM_ACTUATORS>()));
	control_allocation.allocate();
	EXPECT_EQ(control_allocation.getActuatorSetpoint(), (matrix::Vector<float, ControlAllocation::NUM_ACTUATORS>()));
}

TEST(ControlAllocationSequentialDesaturationTest, SetGatActuatorSetpoint)
{
	ControlAllocationSequentialDesaturation control_allocation;
	float actuator_setpoint_array[6] = {1.f, 2.f, 3.f, 4.f, 5.f, 6.f};
	matrix::Vector<float, ControlAllocation::NUM_ACTUATORS> actuator_setpoint(actuator_setpoint_array);
	control_allocation.setActuatorSetpoint(actuator_setpoint);
	EXPECT_EQ(control_allocation.getActuatorSetpoint(), actuator_setpoint);
}

TEST(ControlAllocationSequentialDesaturationTest, AllocateRoll)
{
	ControlAllocationSequentialDesaturation control_allocation;

	float control_setpoint_array[6] = {1.f, 2.f, 3.f, 4.f, 5.f, 6.f};
	matrix::Vector<float, ControlAllocation::NUM_AXES> control_setpoint(control_setpoint_array);
	control_allocation.setControlSetpoint(control_setpoint);
	control_allocation.allocate();
	control_allocation.getActuatorSetpoint().print();
	EXPECT_EQ((matrix::Vector<float, ControlAllocation::NUM_ACTUATORS>()), control_allocation.getActuatorSetpoint());
}
