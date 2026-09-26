// ============================================================================
// dji_rsdk_camera.h — DJI R SDK backend: Osmo Action 4 / 5 Pro / 6, Osmo 360
// ============================================================================
// These cameras speak DJI's official R SDK remote protocol (0xAA framing,
// documented in DJI's Osmo GPS Controller demo) over the same BLE GATT
// service (0xFFF0) the DUML cameras use. The BLE link itself (connect,
// reconnect, auth timeout, liveness watchdog, FC-UART bench guard) comes
// from the shared dji_duml_transport module via DjiLinkHooks; only the
// protocol on top is different.
//
// STATUS: written from DJI's protocol docs + unit-tested frame layer
// (dji_rsdk_protocol.cpp vs DJI's own test frames). NOT yet run against a
// real camera — first bench target is the Osmo Action 4.
// ============================================================================

#ifndef DJI_RSDK_CAMERA_H
#define DJI_RSDK_CAMERA_H

#include "camera_common.h"

void                djiRsdkInit();
void                djiRsdkUpdate();
bool                djiRsdkSendStartRecord();
bool                djiRsdkSendStopRecord();
BleConnectionState  djiRsdkGetState();
const CameraTelemetry& djiRsdkGetTelemetry();
bool                djiRsdkIsReady();
/// Stop scanning and connect directly to this MAC (user-approved pairing).
void                djiRsdkTargetMac(const char *mac);

/// User-initiated one-shot discovery scan (5 s window). Called by
/// camera_manager::camStartUserScan() only.
void                djiRsdkStartScan();

/// Last connect-attempt error (empty = no error), shown as a Web UI toast.
const char*         djiRsdkGetLastError();

#endif // DJI_RSDK_CAMERA_H
