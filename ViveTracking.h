// ViveTracking.h
#ifndef _VIVETRACKING_H_
#define _VIVETRACKING_H_	

#include <openvr.h>
#include "Matrices.h"

class ViveTracking
{
private:

	vr::IVRSystem *m_pHMD = NULL;

	// NOTE: TrackedDevicePose_t describes a single pose for a tracked device
	// NOTE: k_unMaxTrackedDeviceCount is used to pass device IDs to API calls
	vr::TrackedDevicePose_t m_rTrackedDevicePose[vr::k_unMaxTrackedDeviceCount];
	Matrix4 m_rmat4DevicePose[vr::k_unMaxTrackedDeviceCount];

	vr::HmdQuaternion_t ViveTracking::GetRotation(vr::HmdMatrix34_t matrix);
	vr::HmdVector3_t ViveTracking::GetPosition(vr::HmdMatrix34_t matrix);
	// NOTE: Test for Velocity data
	vr::HmdVector3_t ViveTracking::GetVelocity(vr::HmdVector3_t matrix);
	// NOTE: Test for storing origin data
	vr::HmdVector3_t ViveTracking::GetOrigin(vr::HmdVector3_t matrix);
	// NOTE: Test for Euler angles
	vr::HmdVector3_t ViveTracking::GetEuler(vr::HmdQuaternion_t matrix);

public:

	// Constructor declaration
	ViveTracking();
	// Deconstructor declaration
	~ViveTracking();

	// Main loop that calls process and parse routines, if false the service has quit.
	bool RunProcedure();

	// Process a VR event and output some general information.
	//bool ProcessVREvent(const vr::VREvent_t & event);

	// Parse a tracking frame and print its position / rotation
	void ParseTrackingFrame();

	vr::HmdVector3_t ORIGIN;
	vr::HmdVector3_t ORIGIN_ANGLE;

};

#endif _VIVETRACKING_H_
