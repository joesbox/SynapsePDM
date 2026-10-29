#ifndef MathHandler_H
#define MathHandler_H

#include <Globals.h>

/// @brief Reset all MATH configurations and cached evaluation state to safe defaults.
void InitialiseMathChannelData();

/// @brief Evaluate MATH channels in deterministic dependency order.
void EvaluateMathChannels();

/// @brief Validate one expression without changing its persisted bytes.
MathRuntimeError ValidateMathChannelConfig(uint8_t channelIndex);

/// @brief Read one cached MATH result and its current error.
float GetMathChannelValue(uint8_t channelIndex);

/// @brief Get the current error state of a MATH channel.
/// @param channelIndex The index of the MATH channel to query.
/// @return The current error code of the specified MATH channel.
uint8_t GetMathChannelError(uint8_t channelIndex);

/// @brief Read one channel's output using MATH channel-operand semantics: MATH channels
/// return their evaluated root value, PWM types their duty while enabled, others 1/0.
/// @return False when the channel is disabled, unevaluated, or in error.
bool ReadChannelOutputValue(uint8_t channelIndex, float *value);

#endif