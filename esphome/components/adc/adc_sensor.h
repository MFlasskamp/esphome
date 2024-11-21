#pragma once

#include "esphome/components/sensor/sensor.h"
#include "esphome/components/voltage_sampler/voltage_sampler.h"
#include "esphome/core/component.h"
#include "esphome/core/defines.h"
#include "esphome/core/hal.h"

const static char *TAG = "ADC";

#ifdef USE_ESP32
#include <esp_adc/adc_oneshot.h>
#include <esp_adc/adc_cali.h>
#include <esp_adc/adc_cali_scheme.h>
#endif

namespace esphome {
namespace adc {

class ADCSensor : public sensor::Sensor, public PollingComponent, public voltage_sampler::VoltageSampler {
 public:
#ifdef USE_ESP32
  static adc_oneshot_unit_handle_t adc_handle;
  //-------------ADC Init---------------//
  adc_oneshot_unit_init_cfg_t init_config = {
      .unit_id = ADC_UNIT_1, .clk_src = ADC_DIGI_CLK_SRC_DEFAULT, .ulp_mode = ADC_ULP_MODE_DISABLE};

  //-------------ADC Config---------------//
  adc_oneshot_chan_cfg_t config = {
      .atten = ADC_ATTEN_DB_0,
      .bitwidth = ADC_BITWIDTH_DEFAULT,
  };

  //-------------ADC Calibration Init---------------//
  adc_cali_handle_t adc_cali_handle = NULL;
  bool do_calibration = false;

  /// Set the attenuation for this pin. Only available on the ESP32.
  void set_attenuation(adc_atten_t attenuation) { this->config.atten = attenuation; }
  void set_channel(adc_channel_t channel) { this->channel_ = channel; }
  void set_autorange(bool autorange) { this->autorange_ = autorange; }
#endif

  /// Update ADC values
  void update() override;
  /// Setup ADC
  void setup() override;
  void dump_config() override;
  /// `HARDWARE_LATE` setup priority
  float get_setup_priority() const override;
  void set_pin(InternalGPIOPin *pin) { this->pin_ = pin; }
  void set_output_raw(bool output_raw) { this->output_raw_ = output_raw; }
  void set_sample_count(uint8_t sample_count);
  float sample() override;

#ifdef USE_ESP8266
  std::string unique_id() override;
#endif

#ifdef USE_RP2040
  void set_is_temperature() { this->is_temperature_ = true; }
#endif

 protected:
  InternalGPIOPin *pin_;
  bool output_raw_{false};
  uint8_t sample_count_{1};

#ifdef USE_RP2040
  bool is_temperature_{false};
#endif

#ifdef USE_ESP32
  adc_channel_t channel_;
  bool autorange_{false};
#endif

#ifdef USE_ESP32
  /*---------------------------------------------------------------
          ADC Calibration
  ---------------------------------------------------------------*/
  static bool example_adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten,
                                           adc_cali_handle_t *out_handle) {
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
      // ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
      adc_cali_curve_fitting_config_t cali_config = {
          .unit_id = unit,
          .chan = channel,
          .atten = atten,
          .bitwidth = ADC_BITWIDTH_DEFAULT,
      };
      ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
      if (ret == ESP_OK) {
        calibrated = true;
      }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
      // ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
      adc_cali_line_fitting_config_t cali_config = {
          .unit_id = unit,
          .atten = atten,
          .bitwidth = ADC_BITWIDTH_DEFAULT,
      };
      ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
      if (ret == ESP_OK) {
        calibrated = true;
      }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
      // ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
      ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
      ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
  }

  static void example_adc_calibration_deinit(adc_cali_handle_t handle) {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
  }
#endif
};

}  // namespace adc
}  // namespace esphome
