#ifndef API_CORE_H
#define API_CORE_H

#include <Arduino.h>

/// Build the full status JSON (same shape /api/status returns) into buf.
/// Returns the length written (as snprintf would).
size_t apiBuildStatusJson(char *buf, size_t bufLen);

/// Apply a settings-update JSON body (same shape /api/settings accepts).
/// Returns true on success; on failure a short message is written into
/// errBuf.
///
/// apNeedsRestart is set true when the caller should call webInit() after
/// responding — Wi-Fi credentials changed, or the master wifiApEnabled
/// switch just turned ON (the AP wasn't running and needs to come up).
///
/// apShouldStop is set true when the caller should call webStop() after
/// responding instead — the master wifiApEnabled switch just turned OFF.
/// Deferred to the caller (same pattern as apNeedsRestart) so the "ok"
/// response can be sent over the AP before it's torn down.
///
/// The two are mutually exclusive; a caller only needs to check one, then
/// the other, in that order.
bool apiApplySettings(const String &body, bool &apNeedsRestart, bool &apShouldStop,
                       char *errBuf, size_t errBufLen);

/// Apply a camera-registry action (same shape /api/camera accepts:
/// {"scan":true} | {"select":i} | {"remove":i} | {"pair":true,"mac":...,"type":...}).
/// alreadyScanning is set true when a scan was requested but one was
/// already in progress (nothing new was started).
bool apiApplyCamera(const String &body, bool &alreadyScanning,
                     char *errBuf, size_t errBufLen);

/// Apply a {"cmd":"start"|"stop"|"reboot"} command.
/// shouldReboot is set true for "reboot" — the caller must send its
/// response BEFORE acting on it, then delay briefly and call ESP.restart().
bool apiApplyCommand(const String &cmd, bool &shouldReboot,
                      char *errBuf, size_t errBufLen);

#endif // API_CORE_H