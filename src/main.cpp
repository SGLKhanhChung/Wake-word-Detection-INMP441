#include <Arduino.h>
#include <driver/i2s.h>
#include "microphone.h"
#include "a19127644-wake-word_inferencing.h"
// don't mess around with this
#define NOT !
constexpr bool debug_mode = false;
static unsigned long long lastEnable;
static void display_results(ei_impulse_result_t* result)
{
  if(debug_mode){
    return;
  }
    ei_printf("Predictions (DSP: %d ms., Classification: %d ms., Anomaly: %d ms.): \n",
        result->timing.dsp, result->timing.classification, result->timing.anomaly);
    for (size_t ix = 0; ix < EI_CLASSIFIER_LABEL_COUNT; ix++) {            
        ei_printf("    %s: \t%f\r\n", result->classification[ix].label,result->classification[ix].value);
    }
#if EI_CLASSIFIER_HAS_ANOMALY == 1
    ei_printf("    anomaly score: %f\r\n", result->anomaly);
#endif
}


void setup()
{
  // we need serial output for the plotter
  Serial.begin(115200);
  // start up the I2S peripheral
  // i2s_init(16000);
  run_classifier_init();
  ei_microphone_inference_start(EI_CLASSIFIER_SLICE_SIZE,EI_CLASSIFIER_INTERVAL_MS);
  pinMode(GPIO_NUM_2,OUTPUT);
  lastEnable = millis();
}

static int print_results;
// int32_t raw_samples[512];
void toggleLED(float percent);
void loop()
{
    
    if (ei_microphone_inference_is_recording()) {
        return;
    }

    signal_t signal;

    signal.total_length = EI_CLASSIFIER_SLICE_SIZE;
    signal.get_data = &ei_microphone_inference_get_data;

    // run the impulse: DSP, neural network and the Anomaly algorithm
    ei_impulse_result_t result = { 0 };
    EI_IMPULSE_ERROR ei_error;
      ei_error = run_classifier_continuous(&signal, &result, debug_mode);

    if (ei_error != EI_IMPULSE_OK) {
        ei_printf("Failed to run impulse (%d)", ei_error);
        return;
    }

    if(++print_results >= EI_CLASSIFIER_SLICES_PER_MODEL_WINDOW) {
        display_results(&result);
        toggleLED(result.classification[0].value);
        print_results = 0;
    }

}

void toggleLED(float percent){
  constexpr uint16_t interval = 3000;
  if(lastEnable + interval > millis()){
    if(percent > 0.65f)
    {
      digitalWrite(GPIO_NUM_2,HIGH); //wake word
    }
    else{
      digitalWrite(GPIO_NUM_2,LOW); //None
    }
    lastEnable = millis();
  }
}
