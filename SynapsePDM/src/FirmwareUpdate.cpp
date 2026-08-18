/*  FirmwareUpdate.cpp Firmware update variables, functions and data handling.
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

#include "FirmwareUpdate.h"

#include <Arduino.h>
#include <string.h>
#include <SHA256.h>
#include <Ed25519.h>
#include <IWatchdog.h>

#include "FirmwareSigningKey.h"
#include "OutputHandler.h"
#include "Storage.h"
#include "System.h"

static File uploadFile;
static bool uploadInProgress = false;
static bool updateSessionOutputsInhibited = false;
static bool uploadResumeLogging = false;
static char uploadPath[16] = {0};
static char firmwareUpdateDiagnostic[FIRMWARE_UPDATE_DIAGNOSTIC_SIZE] = "idle";

static void SetDiagnosticMessage(const char *message)
{
    if (message == nullptr)
    {
        message = "unknown";
    }

    memset(firmwareUpdateDiagnostic, 0, sizeof(firmwareUpdateDiagnostic));
    strncpy(firmwareUpdateDiagnostic, message, sizeof(firmwareUpdateDiagnostic) - 1);
}

static const char *GetAssetPath(uint8_t assetType)
{
    switch (assetType)
    {
    case FirmwareUpdateAssetBinary:
        return STAGED_FIRMWARE_PATH;
    case FirmwareUpdateAssetHash:
        return STAGED_HASH_PATH;
    case FirmwareUpdateAssetSignature:
        return STAGED_SIGNATURE_PATH;
    default:
        return nullptr;
    }
}

static bool DeleteIfExists(const char *path)
{
    if (SD.exists(path))
    {
        return SD.remove(path);
    }

    return true;
}

static bool AcquireUpdateStorage(bool *resumeLogging)
{
    if (resumeLogging == nullptr)
    {
        return false;
    }

    *resumeLogging = false;

    if (IsLogTransferActive())
    {
        return false;
    }

    if (SDFileOpen)
    {
        dataFile.flush();
        dataFile.close();
        SDFileOpen = false;
        *resumeLogging = true;
    }

    if (!SDCardOK)
    {
        ResumeSD();
    }

    return SDCardOK;
}

static void ReleaseUpdateStorage(bool resumeLogging)
{
    if (resumeLogging)
    {
        ResumeSD();
    }
}

static bool ReadSignatureFile(uint8_t signature[ED25519_SIGNATURE_SIZE])
{
    File signatureFile = SD.open(STAGED_SIGNATURE_PATH, FILE_READ);
    if (!signatureFile)
    {
        return false;
    }

    size_t bytesRead = signatureFile.read(signature, ED25519_SIGNATURE_SIZE);
    signatureFile.close();
    return bytesRead == ED25519_SIGNATURE_SIZE;
}

static int HexNibble(char value)
{
    if (value >= '0' && value <= '9')
    {
        return value - '0';
    }
    if (value >= 'a' && value <= 'f')
    {
        return value - 'a' + 10;
    }
    if (value >= 'A' && value <= 'F')
    {
        return value - 'A' + 10;
    }

    return -1;
}

static bool ReadHashFile(uint8_t expectedDigest[SHA256_DIGEST_SIZE])
{
    File hashFile = SD.open(STAGED_HASH_PATH, FILE_READ);
    if (!hashFile)
    {
        return false;
    }

    char hexBuffer[SHA256_HEX_SIZE] = {0};
    size_t hexIndex = 0;
    while (hashFile.available() && hexIndex < sizeof(hexBuffer))
    {
        char next = (char)hashFile.read();
        if (next == '\r' || next == '\n' || next == ' ' || next == '\t')
        {
            continue;
        }

        hexBuffer[hexIndex++] = next;
    }
    hashFile.close();

    if (hexIndex != sizeof(hexBuffer))
    {
        return false;
    }

    for (size_t index = 0; index < SHA256_DIGEST_SIZE; ++index)
    {
        int high = HexNibble(hexBuffer[index * 2]);
        int low = HexNibble(hexBuffer[(index * 2) + 1]);
        if (high < 0 || low < 0)
        {
            return false;
        }

        expectedDigest[index] = (uint8_t)((high << 4) | low);
    }

    return true;
}

static bool ComputeFirmwareDigest(uint8_t digest[SHA256_DIGEST_SIZE])
{
    File firmwareFile = SD.open(STAGED_FIRMWARE_PATH, FILE_READ);
    if (!firmwareFile)
    {
        return false;
    }

    SHA256 sha256;
    sha256.reset();

    uint8_t buffer[FILE_COPY_BUFFER_SIZE];
    while (firmwareFile.available())
    {
        int bytesRead = firmwareFile.read(buffer, sizeof(buffer));
        if (bytesRead < 0)
        {
            firmwareFile.close();
            return false;
        }
        if (bytesRead == 0)
        {
            break;
        }

        sha256.update(buffer, (size_t)bytesRead);
        IWatchdog.reload();
    }

    firmwareFile.close();
    sha256.finalize(digest, SHA256_DIGEST_SIZE);
    return true;
}

static bool CopyFile(const char *sourcePath, const char *destinationPath)
{
    File sourceFile = SD.open(sourcePath, FILE_READ);
    if (!sourceFile)
    {
        return false;
    }

    File destinationFile = SD.open(destinationPath, FILE_WRITE);
    if (!destinationFile)
    {
        sourceFile.close();
        return false;
    }

    uint8_t buffer[FILE_COPY_BUFFER_SIZE];
    bool success = true;

    while (sourceFile.available())
    {
        int bytesRead = sourceFile.read(buffer, sizeof(buffer));
        if (bytesRead < 0)
        {
            success = false;
            break;
        }

        if (bytesRead == 0)
        {
            break;
        }

        size_t bytesWritten = destinationFile.write(buffer, (size_t)bytesRead);
        if (bytesWritten != (size_t)bytesRead)
        {
            success = false;
            break;
        }

        IWatchdog.reload();
    }

    destinationFile.flush();
    destinationFile.close();
    sourceFile.close();

    if (!success)
    {
        DeleteIfExists(destinationPath);
    }

    return success;
}

static bool StageFirmwareForBootloader()
{
    if (!DeleteIfExists(BOOTLOADER_FIRMWARE_PATH))
    {
        return false;
    }

    return CopyFile(STAGED_FIRMWARE_PATH, BOOTLOADER_FIRMWARE_PATH);
}

static void CleanupStagedUpdateFiles()
{
    DeleteIfExists(STAGED_FIRMWARE_PATH);
    DeleteIfExists(STAGED_HASH_PATH);
    DeleteIfExists(STAGED_SIGNATURE_PATH);
}

static bool StagedUpdateAssetsPresent()
{
    return SD.exists(STAGED_FIRMWARE_PATH) &&
           SD.exists(STAGED_HASH_PATH) &&
           SD.exists(STAGED_SIGNATURE_PATH);
}

static bool VerifyStagedUpdate()
{
    uint8_t expectedDigest[SHA256_DIGEST_SIZE] = {0};
    uint8_t computedDigest[SHA256_DIGEST_SIZE] = {0};
    uint8_t signature[ED25519_SIGNATURE_SIZE] = {0};

    if (!ReadHashFile(expectedDigest))
    {
        SetDiagnosticMessage("install hash read failed");
        return false;
    }

    if (!ReadSignatureFile(signature))
    {
        SetDiagnosticMessage("install signature read failed");
        return false;
    }

    if (!ComputeFirmwareDigest(computedDigest))
    {
        SetDiagnosticMessage("install digest compute failed");
        return false;
    }

    if (memcmp(expectedDigest, computedDigest, SHA256_DIGEST_SIZE) != 0)
    {
        SetDiagnosticMessage("install digest mismatch");
        return false;
    }

    if (!Ed25519::verify(signature, FIRMWARE_UPDATE_PUBLIC_KEY, computedDigest, SHA256_DIGEST_SIZE))
    {
        SetDiagnosticMessage("install signature mismatch");
        return false;
    }

    return true;
}

void SetFirmwareUpdateDiagnostic(const char *message)
{
    SetDiagnosticMessage(message);
}

const char *GetFirmwareUpdateDiagnostic()
{
    return firmwareUpdateDiagnostic;
}

void ClearFirmwareUpdateDiagnostic()
{
    SetDiagnosticMessage("idle");
}

bool IsFirmwareAssetUploadInProgress()
{
    return uploadInProgress;
}

bool BeginFirmwareAssetUpload(uint8_t assetType)
{
    if (uploadInProgress)
    {
        CancelFirmwareAssetUpload();
    }

    const char *assetPath = GetAssetPath(assetType);
    if (assetPath == nullptr)
    {
        SetDiagnosticMessage("invalid upload asset type");
        return false;
    }

    if (!AcquireUpdateStorage(&uploadResumeLogging))
    {
        SetDiagnosticMessage("update storage unavailable");
        return false;
    }

    if (!updateSessionOutputsInhibited)
    {
        SetOutputsInhibited(true);
        updateSessionOutputsInhibited = true;
    }

    if (!DeleteIfExists(assetPath))
    {
        SetDiagnosticMessage("failed to clear update asset");
        ReleaseUpdateStorage(uploadResumeLogging);
        uploadResumeLogging = false;
        if (updateSessionOutputsInhibited)
        {
            SetOutputsInhibited(false);
            updateSessionOutputsInhibited = false;
        }
        return false;
    }

    uploadFile = SD.open(assetPath, FILE_WRITE);
    if (!uploadFile)
    {
        SDCardOK = false;
        ResumeSD();
        uploadFile = SD.open(assetPath, FILE_WRITE);
    }

    if (!uploadFile)
    {
        SetDiagnosticMessage("failed to open update asset file");
        ReleaseUpdateStorage(uploadResumeLogging);
        uploadResumeLogging = false;
        if (updateSessionOutputsInhibited)
        {
            SetOutputsInhibited(false);
            updateSessionOutputsInhibited = false;
        }
        return false;
    }

    memset(uploadPath, 0, sizeof(uploadPath));
    strncpy(uploadPath, assetPath, sizeof(uploadPath) - 1);
    uploadInProgress = true;
    SetDiagnosticMessage("upload begin accepted");
    return true;
}

bool WriteFirmwareAssetChunk(const uint8_t *data, size_t length)
{
    if (!uploadInProgress || !uploadFile || data == nullptr)
    {
        SetDiagnosticMessage("upload chunk invalid state");
        return false;
    }

    if (length == 0)
    {
        SetDiagnosticMessage("upload chunk empty");
        return true;
    }

    size_t totalWritten = 0;
    while (totalWritten < length)
    {
        size_t blockLength = length - totalWritten;
        if (blockLength > FILE_UPLOAD_WRITE_BLOCK_SIZE)
        {
            blockLength = FILE_UPLOAD_WRITE_BLOCK_SIZE;
        }

        size_t bytesWritten = uploadFile.write(&data[totalWritten], blockLength);
        if (bytesWritten != blockLength)
        {
            char diagnostic[48] = {0};
            snprintf(diagnostic, sizeof(diagnostic), "upload sd write fail %u/%u",
                     (unsigned int)bytesWritten,
                     (unsigned int)blockLength);
            SetDiagnosticMessage(diagnostic);
            return false;
        }

        totalWritten += bytesWritten;
        IWatchdog.reload();
    }

    SetDiagnosticMessage("upload chunk accepted");
    return true;
}

bool FinishFirmwareAssetUpload()
{
    if (!uploadInProgress || !uploadFile)
    {
        SetDiagnosticMessage("upload end invalid state");
        return false;
    }

    uploadFile.flush();
    uploadFile.close();
    uploadInProgress = false;
    memset(uploadPath, 0, sizeof(uploadPath));
    ReleaseUpdateStorage(uploadResumeLogging);
    uploadResumeLogging = false;
    SetDiagnosticMessage("upload asset finalized");
    return true;
}

void CancelFirmwareAssetUpload()
{
    if (uploadFile)
    {
        uploadFile.close();
    }

    if (uploadPath[0] != '\0')
    {
        DeleteIfExists(uploadPath);
    }

    uploadInProgress = false;
    memset(uploadPath, 0, sizeof(uploadPath));
    ReleaseUpdateStorage(uploadResumeLogging);
    uploadResumeLogging = false;
    if (updateSessionOutputsInhibited)
    {
        SetOutputsInhibited(false);
        updateSessionOutputsInhibited = false;
    }
    SetDiagnosticMessage("upload cancelled");
}

bool InstallStagedFirmware()
{
    bool resumeLogging = false;
    if (!AcquireUpdateStorage(&resumeLogging))
    {
        SetDiagnosticMessage("install storage unavailable");
        if (updateSessionOutputsInhibited)
        {
            SetOutputsInhibited(false);
            updateSessionOutputsInhibited = false;
        }
        return false;
    }

    if (!StagedUpdateAssetsPresent())
    {
        SetDiagnosticMessage("install staged assets missing");
        if (updateSessionOutputsInhibited)
        {
            SetOutputsInhibited(false);
            updateSessionOutputsInhibited = false;
        }
        ReleaseUpdateStorage(resumeLogging);
        return false;
    }

    if (!VerifyStagedUpdate())
    {
        if (updateSessionOutputsInhibited)
        {
            SetOutputsInhibited(false);
            updateSessionOutputsInhibited = false;
        }
        ReleaseUpdateStorage(resumeLogging);
        return false;
    }

    if (!StageFirmwareForBootloader())
    {
        SetDiagnosticMessage("install stage copy failed");
        if (updateSessionOutputsInhibited)
        {
            SetOutputsInhibited(false);
            updateSessionOutputsInhibited = false;
        }
        ReleaseUpdateStorage(resumeLogging);
        return false;
    }

    CleanupStagedUpdateFiles();
    SetDiagnosticMessage("install staged firmware ready");
    ReleaseUpdateStorage(resumeLogging);
    return true;
}
