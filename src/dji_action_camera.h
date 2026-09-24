// ============================================================================
// dji_action_camera.h — DUML-over-BLE backend for DJI Osmo Action cameras
// ============================================================================
// Target: DJI Osmo Action 2 (first Action-line camera on this fork's bench).
// Shares pairing / record opcodes / transport with the Nano backend via
// dji_duml_transport; differs in telemetry: recording state and battery
// are POLLED (02/70, 0D/02) instead of arriving as unsolicited pushes.
//
// STATUS: hardware-verified on an Action 2 (2026-09-24) — pairing, record
// control, record state, battery, remaining time, heartbeat and reconnect.
// Built from two independent Action 2 projects plus this repo's upstream
// Action 2 work (sources in dji_action_camera.cpp). Action 3/4/5 are untested here; the newer models also speak
// DJI's official 0xAA-framed camera-remote protocol (Osmo GPS Controller
// SDK), which may turn out to need a backend of its own.
// ============================================================================

#ifndef DJI_ACTION_CAMERA_H
#define DJI_ACTION_CAMERA_H

#include "camera_common.h"

void                djiActionInit();
void                djiActionUpdate();
bool                djiActionSendStartRecord();
bool                djiActionSendStopRecord();
BleConnectionState  djiActionGetState();
const CameraTelemetry& djiActionGetTelemetry();
bool                djiActionIsReady();
/// Stop scanning and connect directly to this MAC (user-approved pairing).
void                djiActionTargetMac(const char *mac);

/// User-initiated one-shot discovery scan (5 s window). Called by
/// camera_manager::camStartUserScan() from the /api/camera {scan:true}
/// endpoint — NEVER from the background reconnect loop.
void                djiActionStartScan();

/// Read-only access to the last connect-attempt error (empty = no error).
/// Surfaced via /api/status and shown as a toast in the Web UI.
const char*         djiActionGetLastError();

#endif // DJI_ACTION_CAMERA_H