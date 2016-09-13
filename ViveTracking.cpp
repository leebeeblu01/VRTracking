#include "ViveTracking.h"
#include <Windows.h>


ViveTracking::~ViveTracking()
{
	if (m_pHMD != NULL)
	{
		vr::VR_Shutdown();
		m_pHMD = NULL;
	}
}

ViveTracking::ViveTracking()
{
	vr::EVRInitError eError = vr::VRInitError_None;
	m_pHMD = vr::VR_Init(&eError, vr::VRApplication_Background);

	if (eError != vr::VRInitError_None)
	{
		m_pHMD = NULL;
		char buf[1024];
		sprintf_s(buf, sizeof(buf), "Unable to initialize VR runtime: %s", vr::VR_GetVRInitErrorAsEnglishDescription(eError));
		printf_s(buf);
		Sleep(5000);
		exit(EXIT_FAILURE);
	}
}

bool ViveTracking::RunProcedure(void)
{	
	// Output tracking data every 100ms or 10Hz
	// Change as needed
	ParseTrackingFrame();
	Sleep(100);	
	return true;
}



// Rotation in quaternions
vr::HmdQuaternion_t ViveTracking::GetRotation(vr::HmdMatrix34_t matrix)
{
	vr::HmdQuaternion_t q;
	q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
	q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
	q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
	return q;
}

// Rotation in Euler Angles
vr::HmdVector3_t ViveTracking::GetEuler(vr::HmdQuaternion_t matrix)
{
	vr::HmdQuaternion_t q;
	q.w = matrix.w;
	q.x = matrix.x;
	q.y = matrix.y;
	q.z = matrix.z;
	vr::HmdVector3_t euler;
	euler.v[0] = atan2(2 * (q.w*q.x + q.y*q.z), 1 - 2*((q.x)*(q.x) + (q.y)*q.y));
	euler.v[1] = asin(2 * (q.w*q.y - q.z*q.x));
	euler.v[2] = atan2(2 * (q.w*q.z + q.x*q.y), 1 - 2 * (q.y*q.y + q.z*q.z));
	return euler;
}

// Position matrix
vr::HmdVector3_t ViveTracking::GetPosition(vr::HmdMatrix34_t matrix)
{
	vr::HmdVector3_t vector;
	vector.v[0] = matrix.m[0][3];
	vector.v[1] = matrix.m[1][3];
	vector.v[2] = matrix.m[2][3];
	return vector;
}

// NOTE: Test to track velocity
vr::HmdVector3_t ViveTracking::GetVelocity(vr::HmdVector3_t matrix)
{
	vr::HmdVector3_t velocity;
	velocity.v[0] = matrix.v[0];
	velocity.v[1] = matrix.v[1];
	velocity.v[2] = matrix.v[2];
	return velocity;
}

// NOTE: Obtain Origin
vr::HmdVector3_t ViveTracking::GetOrigin(vr::HmdVector3_t matrix)
{
	vr::HmdVector3_t origin;
	origin.v[0] = matrix.v[0];
	origin.v[1] = matrix.v[1];
	origin.v[2] = matrix.v[2];
	return origin;

}


void ViveTracking::ParseTrackingFrame()
{
	// Process SteamVR device states
		
	for (vr::TrackedDeviceIndex_t unDevice = 0; unDevice < vr::k_unMaxTrackedDeviceCount; unDevice++)
	{
		if (!m_pHMD->IsTrackedDeviceConnected(unDevice))
			continue;
		
		vr::VRControllerState_t state;

		

		if (m_pHMD->GetControllerState(unDevice, &state))
		{
			vr::TrackedDevicePose_t trackedDevicePose;
			vr::TrackedDevicePose_t *devicePose = &trackedDevicePose;

			vr::TrackedDevicePose_t trackedControllerPose;
			vr::TrackedDevicePose_t *controllerPose = &trackedControllerPose;

			vr::VRControllerState_t controllerState;
			vr::VRControllerState_t *controllerState_ptr = &controllerState;

			vr::HmdVector3_t vector;
			vr::HmdQuaternion_t quaternion;
			// NOTE: Test for tracking Velocity
			vr::HmdVector3_t velocity;
			vr::HmdVector3_t euler;

			char buf[1024];

			vr::ETrackedDeviceClass trackedDeviceClass = vr::VRSystem()->GetTrackedDeviceClass(unDevice);
			switch (trackedDeviceClass)
			{
				// NOTE: May need to adjust TrackingUniverseRawAndUncalibrated to TrackingUniverseStanding
				case vr::ETrackedDeviceClass::TrackedDeviceClass_HMD:
					vr::VRSystem()->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0, &trackedDevicePose, 1);
					vector = GetPosition(devicePose->mDeviceToAbsoluteTracking);
					quaternion = GetRotation(devicePose->mDeviceToAbsoluteTracking);

					// NOTE: Output tracking information for HMD here
					//char buf[1024];
					/*
					sprintf_s(buf, sizeof(buf), "\nPosition: [%2.2f %2.2f %2.2f]\n", vector.v[0], vector.v[1], vector.v[2]);
					printf_s(buf);

					sprintf_s(buf, sizeof(buf), "Rotation: [%2.2f %2.2f %2.2f %2.2f]\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
					printf_s(buf);

					sprintf_s(buf, sizeof(buf), "Device ID: %d\n", unDevice);
					printf_s(buf);
					*/
					break;

				case vr::ETrackedDeviceClass::TrackedDeviceClass_Controller:
					vr::VRSystem()->GetControllerStateWithPose(vr::TrackingUniverseStanding, unDevice, &controllerState, &trackedControllerPose);
					vector = GetPosition(controllerPose->mDeviceToAbsoluteTracking);
					quaternion = GetRotation(controllerPose->mDeviceToAbsoluteTracking);
					velocity = GetVelocity(controllerPose->vVelocity);
					// NOTE: Get the Euler angles
					euler = GetEuler(quaternion);
					

					// NOTE: Use for testing purposes
					/*
					char buf[1024];
					sprintf_s(buf, sizeof(buf), "\nController\nx: %.2f y: %.2f z: %.2f\n", vector.v[0], vector.v[1], vector.v[2]);
					printf_s(buf);

					sprintf_s(buf, sizeof(buf), "qw: %.2f qx: %.2f qy: %.2f qz: %.2f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
					printf_s(buf);

					sprintf_s(buf, sizeof(buf), "vx: %.2f vy: %.2f vz: %.2f\n", velocity.v[0], velocity.v[1], velocity.v[2]);
					printf_s(buf);

					sprintf_s(buf, sizeof(buf), "Device ID: %d\n", unDevice);
					printf_s(buf);
					*/


					// NOTE: Used for tracking if the trigger button was pressed on the controller.
					// Probably need to move this somewhere else.
					vr::VREvent_t event;
					if (m_pHMD->PollNextEvent(&event, sizeof(event)))
					{
						switch (event.eventType)
						{
						case vr::VREvent_ButtonPress:
						{
							//uint32_t butNum = 33;
							if (event.data.controller.button == 33)
							{
								char buf[1024];
								sprintf_s(buf, sizeof(buf), "(OpenVR) Trigger Button was pressed\n");
								printf_s(buf);
								ORIGIN = GetOrigin(vector);
								ORIGIN_ANGLE = GetEuler(quaternion);
								sprintf_s(buf, sizeof(buf), "OpenVR) New Origin Set at [%2.2f %2.2f %2.2f]\n", ORIGIN.v[0], ORIGIN.v[1], ORIGIN.v[2]);
								printf_s(buf);
								sprintf_s(buf, sizeof(buf), "OpenVR) New Origin Angle Set at [%2.2f %2.2f %2.2f]\n", ORIGIN_ANGLE.v[0], ORIGIN_ANGLE.v[1], ORIGIN_ANGLE.v[2]);
								printf_s(buf);
								
								Sleep(1000);
							}
						}
						}
					}

					vector.v[0] = vector.v[0] - ORIGIN.v[0];
					vector.v[1] = vector.v[1] - ORIGIN.v[1];
					vector.v[2] = vector.v[2] - ORIGIN.v[2];

					euler.v[0] = euler.v[0] - ORIGIN_ANGLE.v[0];
					euler.v[1] = euler.v[1] - ORIGIN_ANGLE.v[1];
					euler.v[2] = euler.v[2] - ORIGIN_ANGLE.v[2];
					

					// NOTE: Use for deployment

					//char buf[1024];
					sprintf_s(buf, sizeof(buf), "\nPosition: [%2.2f %2.2f %2.2f]\n", vector.v[0], vector.v[1], vector.v[2]);
					printf_s(buf);

					/*sprintf_s(buf, sizeof(buf), "Rotation: [%2.2f %2.2f %2.2f %2.2f]\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
					printf_s(buf);*/

					sprintf_s(buf, sizeof(buf), "Velocity: [%2.2f %2.2f %2.2f]\n", velocity.v[0], velocity.v[1], velocity.v[2]);
					printf_s(buf);

					sprintf_s(buf, sizeof(buf), "Euler Angles: [%2.2f %2.2f %2.2f]\n", euler.v[0], euler.v[1], euler.v[2]);
					printf_s(buf);

					sprintf_s(buf, sizeof(buf), "Device ID: %d\n", unDevice);
					printf_s(buf);

					break;

					/*					
					switch (vr::VRSystem()->GetControllerRoleForTrackedDeviceIndex(unDevice))
					{
					case vr::TrackedControllerRole_Invalid:
						// Invalid hand
						break;
						
					case vr::TrackedControllerRole_LeftHand:
						char buf[1024];
						sprintf_s(buf, sizeof(buf), "\nLeft Controller\nx: %.2f y: %.2f z: %.2f\n", vector.v[0], vector.v[1], vector.v[2]);
						printf_s(buf);

						sprintf_s(buf, sizeof(buf), "qw: %.2f qx: %.2f qy: %.2f qz: %.2f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
						printf_s(buf);
						
						break;

					case vr::TrackedControllerRole_RightHand:
						char buf2[1024];
						sprintf_s(buf2, sizeof(buf2), "\nRight Controller\nx: %.2f y: %.2f z: %.2f\n", vector.v[0], vector.v[1], vector.v[2]);
						printf_s(buf2);

						sprintf_s(buf2, sizeof(buf2), "qw: %.2f qx: %.2f qy: %.2f qz: %.2f\n", quaternion.w, quaternion.x, quaternion.y, quaternion.z);
						printf_s(buf2);
						break;
					}
					break;
					*/
			}
		}

	}
}