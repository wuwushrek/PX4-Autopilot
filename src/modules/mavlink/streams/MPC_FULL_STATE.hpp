/****************************************************************************
 *
 *   Copyright (c) 2022 Franck Djeumou. All rights reserved.
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

#ifndef MPC_FULL_STATE_HPP
#define MPC_FULL_STATE_HPP

#include <uORB/topics/mpc_full_state.h>

class MavlinkStreamMPCFullState : public MavlinkStream
{
public:
	static MavlinkStream *new_instance(Mavlink *mavlink) { return new MavlinkStreamMPCFullState(mavlink); }

	static constexpr const char *get_name_static() { return "MPC_FULL_STATE"; }
	static constexpr uint16_t get_id_static() { return MAVLINK_MSG_ID_MPC_FULL_STATE; }

	const char *get_name() const override { return get_name_static(); }
	uint16_t get_id() override { return get_id_static(); }

	unsigned get_size() override
	{
		return _mpc_full_state_sub.advertised() ? MAVLINK_MSG_ID_MPC_FULL_STATE_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES : 0;
	}

private:
	explicit MavlinkStreamMPCFullState(Mavlink *mavlink) : MavlinkStream(mavlink) {}

	uORB::Subscription _mpc_full_state_sub{ORB_ID(mpc_full_state)};

	bool send() override
	{
		mpc_full_state_s mpc_state;

		if (_mpc_full_state_sub.update(&mpc_state)) {
			mavlink_mpc_full_state_t msg{};

			msg.time_usec = mpc_state.timestamp_sample;
			msg.x = mpc_state.x;
			msg.y = mpc_state.y;
			msg.z = mpc_state.z;
			msg.vx = mpc_state.vx;
			msg.vy = mpc_state.vy;
			msg.vz = mpc_state.vz;
			msg.qw = mpc_state.qw;
			msg.qx = mpc_state.qx;
			msg.qy = mpc_state.qy;
			msg.qz = mpc_state.qz;
			msg.wx = mpc_state.wx;
			msg.wy = mpc_state.wy;
			msg.wz = mpc_state.wz;
			// Motor inputs
			msg.m1 = mpc_state.m1;
			msg.m2 = mpc_state.m2;
			msg.m3 = mpc_state.m3;
			msg.m4 = mpc_state.m4;
			msg.m5 = mpc_state.m5;
			msg.m6 = mpc_state.m6;

			mavlink_msg_mpc_full_state_send_struct(_mavlink->get_channel(), &msg);

			return true;

		}

		return false;
	}
};

#endif // MPC_FULL_STATE_HPP
