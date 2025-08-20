/**
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/sys/printk.h>
#include <zephyr/logging/log.h>


#include <vl53l0x_api.h>
#include <vl53l0x_platform.h>

void print_pal_error(VL53L0X_Error Status){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    printk("API Status: %i : %s\n", Status, buf);
}

void print_range_status(VL53L0X_RangingMeasurementData_t* pRangingMeasurementData){
    char buf[VL53L0X_MAX_STRING_LENGTH];
    uint8_t RangeStatus;

    /*
     * New Range Status: data is valid when pRangingMeasurementData->RangeStatus = 0
     */

    RangeStatus = pRangingMeasurementData->RangeStatus;

    VL53L0X_GetRangeStatusString(RangeStatus, buf);
    printk("Range Status: %i : %s\n", RangeStatus, buf);

}


VL53L0X_Error rangingTest(VL53L0X_Dev_t *pMyDevice)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    VL53L0X_RangingMeasurementData_t    RangingMeasurementData;
    int i;
    FixPoint1616_t LimitCheckCurrent;
    uint32_t refSpadCount;
    uint8_t isApertureSpads;
    uint8_t VhvSettings;
    uint8_t PhaseCal;

    if(Status == VL53L0X_ERROR_NONE)
    {
        printk ("Call of VL53L0X_StaticInit\n");
        Status = VL53L0X_StaticInit(pMyDevice); // Device Initialization
        print_pal_error(Status);
    }
    
    if(Status == VL53L0X_ERROR_NONE)
    {
        printk ("Call of VL53L0X_PerformRefCalibration\n");
        Status = VL53L0X_PerformRefCalibration(pMyDevice,
        		&VhvSettings, &PhaseCal); // Device Initialization
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {
        printk ("Call of VL53L0X_PerformRefSpadManagement\n");
        Status = VL53L0X_PerformRefSpadManagement(pMyDevice,
        		&refSpadCount, &isApertureSpads); // Device Initialization
        printk ("refSpadCount = %d, isApertureSpads = %d\n", refSpadCount, isApertureSpads);
        print_pal_error(Status);
    }

    if(Status == VL53L0X_ERROR_NONE)
    {

        // no need to do this when we use VL53L0X_PerformSingleRangingMeasurement
        printk ("Call of VL53L0X_SetDeviceMode\n");
        Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING); // Setup in single ranging mode
        print_pal_error(Status);
    }

    // Enable/Disable Sigma and Signal check
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    }
    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
        		VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckEnable(pMyDevice,
        		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
    }

    if (Status == VL53L0X_ERROR_NONE) {
        Status = VL53L0X_SetLimitCheckValue(pMyDevice,
        		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
        		(FixPoint1616_t)(1.5*0.023*65536));
    }


    /*
     *  Step  4 : Test ranging mode
     */

    if(Status == VL53L0X_ERROR_NONE)
    {
        for(i=0;i<10;i++){
            printk ("Call of VL53L0X_PerformSingleRangingMeasurement\n");
            Status = VL53L0X_PerformSingleRangingMeasurement(pMyDevice,
            		&RangingMeasurementData);

            print_pal_error(Status);
            print_range_status(&RangingMeasurementData);

            VL53L0X_GetLimitCheckCurrent(pMyDevice,
            		VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, &LimitCheckCurrent);

            printk("RANGE IGNORE THRESHOLD: %f\n\n", (float)LimitCheckCurrent/(float)65536.0);


            if (Status != VL53L0X_ERROR_NONE) break;

            printk("Measured distance: %i\n\n", RangingMeasurementData.RangeMilliMeter);


        }
    }
    return Status;
}

// Enhanced Calibration Diagnostic Function
VL53L0X_Error diagnose_calibration_failure(VL53L0X_Dev_t *pMyDevice) {
    // Detailed Interrupt and Range Status Diagnostics
    uint8_t interrupt_status = 0, range_status = 0;
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    // Read Interrupt and Range Status
    Status = VL53L0X_RdByte(pMyDevice, VL53L0X_REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
    if (Status != VL53L0X_ERROR_NONE) {
        printk("Failed to read Interrupt Status\n");
        return Status;
    }

    Status = VL53L0X_RdByte(pMyDevice, VL53L0X_REG_RESULT_RANGE_STATUS, &range_status);
    if (Status != VL53L0X_ERROR_NONE) {
        printk("Failed to read Range Status\n");
        return Status;
    }

    // Detailed Diagnostic Logging
    printk("Calibration Diagnostic Details:\n");
    printk("  Interrupt Status: 0x%02X\n", interrupt_status);
    printk("  Range Status: 0x%02X\n", range_status);

    // Interpret Specific Status Codes
    switch (interrupt_status) {
        case 0x50:
            printk("  Possible Issues:\n");
            printk("  - Incorrect interrupt configuration\n");
            printk("  - Signal quality problems\n");
            break;
        default:
            printk("  Unknown interrupt configuration\n");
    }

    switch (range_status) {
        case 0x18:
            printk("  Range Status Indicates:\n");
            printk("  - Possible calibration signal weakness\n");
            printk("  - Sensor alignment issues\n");
            break;
        default:
            printk("  Unrecognized range status\n");
    }

    return Status;
}

// Modified Initialization Function with Advanced Error Handling
VL53L0X_Error _standard_vl53l0x_init(VL53L0X_Dev_t *pMyDevice) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t VhvSettings = 0, PhaseCal = 0;

    // Validate Device Pointer
    if (pMyDevice == NULL) {
        printk("Error: NULL device pointer\n");
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    // 1. Device Initialization
    Status = VL53L0X_DataInit(pMyDevice);
    if (Status != VL53L0X_ERROR_NONE) {
        printk("DataInit Failed. Status: %d\n", Status);
        return Status;
    }

    // 2. Static Initialization
    Status = VL53L0X_StaticInit(pMyDevice);
    if (Status != VL53L0X_ERROR_NONE) {
        printk("StaticInit Failed. Status: %d\n", Status);
        return Status;
    }

    // 3. Reference Calibration with Comprehensive Error Handling
    int max_attempts = 3;
    for (int attempt = 1; attempt <= max_attempts; attempt++) {
        printk("Reference Calibration Attempt %d\n", attempt);
        
        // Perform Calibration with Detailed Logging
        Status = VL53L0X_PerformRefCalibration(pMyDevice, &VhvSettings, &PhaseCal);
        
        if (Status == VL53L0X_ERROR_NONE) {
            printk("Calibration Successful\n");
            printk("  VhvSettings: %d\n", VhvSettings);
            printk("  PhaseCal: %d\n", PhaseCal);
            break;
        }

        // Diagnostic Analysis of Failure
        printk("Calibration Attempt %d Failed. Status: %d\n", attempt, Status);
        diagnose_calibration_failure(pMyDevice);

        // Potential Recovery Strategies
        switch (attempt) {
            case 2:
                // Try relaxing signal rate limits
                VL53L0X_SetLimitCheckValue(
                    pMyDevice, 
                    VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 
                    (FixPoint1616_t)(0.1 * 65536)  // More lenient threshold
                );
                break;
            case 3:
                // Last resort: reset device
                VL53L0X_ResetDevice(pMyDevice);
                break;
        }

        // Delay between attempts
        k_msleep(50 * attempt);
    }

    // Final Calibration Check
    if (Status != VL53L0X_ERROR_NONE) {
        printk("Reference Calibration Failed after %d attempts\n", max_attempts);
        return Status;
    }

    // 4. Device Mode Configuration
    Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (Status != VL53L0X_ERROR_NONE) {
        printk("Device Mode Setting Failed. Status: %d\n", Status);
        return Status;
    }

    // 5. Measurement Timing Budget
    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice, 33000);
    if (Status != VL53L0X_ERROR_NONE) {
        printk("Timing Budget Setting Failed. Status: %d\n", Status);
        return Status;
    }

    // 6. Signal Rate Limits with Progressive Configuration
    Status = VL53L0X_SetLimitCheckValue(
        pMyDevice, 
        VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 
        (FixPoint1616_t)(0.25 * 65536)  // Moderate threshold
    );
    if (Status != VL53L0X_ERROR_NONE) {
        printk("Signal Rate Limit Setting Failed. Status: %d\n", Status);
        return Status;
    }

    printk("VL53L0X Initialization Successful\n");
    return Status;
}

// Advanced Error Handling Macro
#define VL53L0X_VERIFY_STATUS(func, error_msg) \
    do { \
        Status = func; \
        if (Status != VL53L0X_ERROR_NONE) { \
            printk(error_msg " Failed. Status: %d\n", Status); \
            diagnose_calibration_failure(pMyDevice); \
            return Status; \
        } \
    } while(0)


VL53L0X_Error standard_vl53l0x_init(VL53L0X_Dev_t *pMyDevice) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t VhvSettings = 0, PhaseCal = 0;

    // 1. Device Initialization
    Status = VL53L0X_DataInit(pMyDevice);
    if (Status != VL53L0X_ERROR_NONE) {
        printk("DataInit Failed. Status: %d\n", Status);
        return Status;
    }

    // 2. Static Initialization
    Status = VL53L0X_StaticInit(pMyDevice);
    if (Status != VL53L0X_ERROR_NONE) {
        printk("StaticInit Failed. Status: %d\n", Status);
        return Status;
    }

    // 3. Reference Calibration Diagnostic
    printk("Attempting Reference Calibration\n");
    Status = VL53L0X_PerformRefCalibration(pMyDevice, &VhvSettings, &PhaseCal);
    
    if (Status != VL53L0X_ERROR_NONE) {
        printk("Reference Calibration Failed. Status: %d\n", Status);
        
        // Detailed Diagnostic Logging
        uint8_t interrupt_status = 0, range_status = 0;
        VL53L0X_RdByte(pMyDevice, VL53L0X_REG_RESULT_INTERRUPT_STATUS, &interrupt_status);
        VL53L0X_RdByte(pMyDevice, VL53L0X_REG_RESULT_RANGE_STATUS, &range_status);
        
        printk("Interrupt Status: 0x%02X\n", interrupt_status);
        printk("Range Status: 0x%02X\n", range_status);

        return Status;
    }

    // 4. Device Mode Configuration
    Status = VL53L0X_SetDeviceMode(pMyDevice, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (Status != VL53L0X_ERROR_NONE) {
        printk("Device Mode Setting Failed. Status: %d\n", Status);
        return Status;
    }

    // 5. Measurement Timing Budget
    Status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(pMyDevice, 33000);
    if (Status != VL53L0X_ERROR_NONE) {
        printk("Timing Budget Setting Failed. Status: %d\n", Status);
        return Status;
    }

    // 6. Signal Rate Limits
    Status = VL53L0X_SetLimitCheckValue(
        pMyDevice, 
        VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 
        (FixPoint1616_t)(0.25 * 65536)  // 0.25 MCPS
    );
    if (Status != VL53L0X_ERROR_NONE) {
        printk("Signal Rate Limit Setting Failed. Status: %d\n", Status);
        return Status;
    }

    printk("VL53L0X Initialization Successful\n");
    return Status;
}

// Robust Measurement Function
VL53L0X_Error perform_measurement(VL53L0X_Dev_t *pMyDevice, VL53L0X_RangingMeasurementData_t *pRangingMeasurementData) {
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;

    // Start Single Measurement
    Status = VL53L0X_StartMeasurement(pMyDevice);
    if (Status != VL53L0X_ERROR_NONE) {
        printk("Start Measurement Failed. Status: %d\n", Status);
        return Status;
    }

    // Wait for Measurement Completion
    uint8_t measurement_ready = 0;
    uint32_t timeout = 100;  // 100ms timeout
    while (timeout > 0) {
        Status = VL53L0X_GetMeasurementDataReady(pMyDevice, &measurement_ready);
        if (Status != VL53L0X_ERROR_NONE) {
            printk("Measurement Ready Check Failed. Status: %d\n", Status);
            return Status;
        }

        if (measurement_ready) break;
        
        k_msleep(1);  // Zephyr sleep
        timeout--;
    }

    if (timeout == 0) {
        printk("Measurement Timeout\n");
        return VL53L0X_ERROR_TIME_OUT;
    }

    // Get Ranging Measurement
    Status = VL53L0X_GetRangingMeasurementData(pMyDevice, pRangingMeasurementData);
    if (Status != VL53L0X_ERROR_NONE) {
        printk("Get Measurement Data Failed. Status: %d\n", Status);
        return Status;
    }

    // Clear Interrupt
    Status = VL53L0X_ClearInterruptMask(pMyDevice, 0);
    if (Status != VL53L0X_ERROR_NONE) {
        printk("Clear Interrupt Mask Failed. Status: %d\n", Status);
    }

    return Status;
}

// Example Usage
void vl53l0x_task(VL53L0X_Dev_t * my_device) {
    // VL53L0X_Dev_t my_device = {0};
    VL53L0X_RangingMeasurementData_t ranging_data;

    // Initialize Device
    VL53L0X_Error init_status = standard_vl53l0x_init(my_device);
    if (init_status != VL53L0X_ERROR_NONE) {
        printk("Initialization Failed\n");
        return;
    }

    // Perform Measurement
    VL53L0X_Error measurement_status = perform_measurement(&my_device, &ranging_data);
    if (measurement_status == VL53L0X_ERROR_NONE) {
        printk("Distance: %d mm\n", ranging_data.RangeMilliMeter);
        printk("Signal Rate: %.2f MCPS\n", (float)ranging_data.SignalRateRtnMegaCps / 65536.0);
    }
}
