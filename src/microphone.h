#pragma once
#include "Arduino.h"
int i2s_init(uint32_t sampling_rate);
bool ei_microphone_inference_start(uint32_t n_samples, float interval_ms);
int ei_microphone_inference_get_data(size_t offset, size_t length, float *out_ptr);
bool ei_microphone_inference_is_recording(void);