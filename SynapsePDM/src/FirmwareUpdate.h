/*  FirmwareUpdate.h Firmware update definitions and functions.
    Copyright (c) 2026 Joe Mann.  All right reserved.

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in
    all copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
    THE SOFTWARE.

*/

#ifndef FirmwareUpdate_H
#define FirmwareUpdate_H

#include <Arduino.h>

#define UPDATE_CHECK_INTERVAL_MS 5000
#define SHA256_DIGEST_SIZE 32
#define SHA256_HEX_SIZE (SHA256_DIGEST_SIZE * 2)
#define ED25519_SIGNATURE_SIZE 64
#define FILE_COPY_BUFFER_SIZE 512
#define FILE_UPLOAD_WRITE_BLOCK_SIZE 512
#define FIRMWARE_UPDATE_DIAGNOSTIC_SIZE 96

#define STAGED_FIRMWARE_PATH "/update.bin"
#define STAGED_HASH_PATH "/update.sha256"
#define STAGED_SIGNATURE_PATH "/update.sig"
#define STAGED_READY_PATH "/update.ready"
#define BOOTLOADER_FIRMWARE_PATH "/synapse.bin"

enum FirmwareUpdateAssetType : uint8_t
{
    FirmwareUpdateAssetBinary = 0,
    FirmwareUpdateAssetHash = 1,
    FirmwareUpdateAssetSignature = 2,
};

/// @brief Sets the diagnostic message for the firmware update process.
/// @param message The diagnostic message to set.
void SetFirmwareUpdateDiagnostic(const char *message);

/// @brief Gets the current diagnostic message for the firmware update process.
/// @return The current diagnostic message.
const char *GetFirmwareUpdateDiagnostic();

/// @brief Clears the firmware update diagnostic message.
void ClearFirmwareUpdateDiagnostic();

/// @brief Checks if a firmware asset upload is currently in progress.
/// @return True if an upload is in progress, false otherwise.
bool IsFirmwareAssetUploadInProgress();

/// @brief Services the firmware update process. Should be called regularly in the main loop.
void ServiceFirmwareUpdate();

/// @brief Begins the upload of a firmware asset.
/// @param assetType The type of firmware asset to upload.
/// @return True if the upload was successfully started, false otherwise.
bool BeginFirmwareAssetUpload(uint8_t assetType);

/// @brief Writes a chunk of data to the currently active firmware asset upload.
/// @param data Pointer to the data to write.
/// @param length Length of the data to write.
/// @return True if the chunk was successfully written, false otherwise.
bool WriteFirmwareAssetChunk(const uint8_t *data, size_t length);

/// @brief Finishes the current firmware asset upload, finalizing the file on disk.
/// @return True if the upload was successfully finalized, false otherwise.
bool FinishFirmwareAssetUpload();

/// @brief Cancels the current firmware asset upload, discarding any uploaded data.
void CancelFirmwareAssetUpload();

/// @brief Checks if a staged firmware update is present and valid.
/// @return True if a valid staged firmware update is present, false otherwise.
bool InstallStagedFirmware();

#endif