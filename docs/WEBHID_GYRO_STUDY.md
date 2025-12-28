# Betaflight WebHID & Gyro Data Analysis

## 1. Executive Summary
The Betaflight firmware currently supports WebHID communication, but it is strictly limited to **RC Controller (Joystick) inputs**. It does not transmit sensor data (Gyro, Accelerometer) by default. To enable Gyro data transmission via WebHID, specific modifications to the USB HID Report Descriptors and the data transmission logic are required.

## 2. Current WebHID Architecture

### 2.1 USB Composite Device
The controller presents itself to the host (computer/browser) as a Composite Device consisting of two interfaces:
1.  **CDC (Communication Device Class):** Used for the Betaflight Configurator (serial data).
2.  **HID (Human Interface Device):** Used to emulate a Joystick/Gamepad.

### 2.2 Key Files
*   **`src/main/io/usb_cdc_hid.c`**: Contains the high-level logic for mapping data to HID reports.
*   **`src/platform/STM32/vcp_hal/usbd_cdc_hid.c`** (and similar per platform): Defines the USB Descriptors and low-level transmission functions.
*   **`src/main/sensors/gyro.h`**: Defines the `gyro_t` structure containing the sensor data.

### 2.3 Data Flow
Currently, the system only sends RC stick positions:
1.  **Source:** Global `rcData` array (Raw RC inputs).
2.  **Processing:** `sendRcDataToHid()` maps these inputs to USB axes (X, Y, Z, Rx, Ry, Rz).
3.  **Transport:** `sendReport()` transmits the byte array to the USB host.

## 3. Why Gyro Data is Missing
1.  **Report Descriptor Limitation:** The USB Report Descriptor is hardcoded as a "Game Pad". It defines usages for Axes (0x30-0x35) and Buttons, but not for Motion/Gyro sensors.
2.  **Code Logic:** The `sendRcDataToHid` function explicitly reads from `rcData` and has no access or logic to read from the `gyro` structure.

## 4. Implementation Guide for Gyro over WebHID

To stream Gyro data to the browser, follow these steps:

### Step 1: Access Gyro Data
Include the gyro header in `usb_cdc_hid.c` to access the global data.
```c
#include "sensors/gyro.h"
// Access data via: gyro.gyroADCf[X], gyro.gyroADCf[Y], gyro.gyroADCf[Z]
```

### Step 2: Create Transmission Logic
Create a function to format and send the data. You may need to scale the float values to fit the report format (int8 or int16).

```c
void sendGyroDataToHid(void) {
    int8_t report[9]; // Adjust size based on descriptor
    
    // Example: Map Gyro X, Y, Z to first 3 bytes
    // Note: Proper scaling from float to int8 is required
    report[0] = (int8_t)(gyro.gyroADCf[X]); 
    report[1] = (int8_t)(gyro.gyroADCf[Y]);
    report[2] = (int8_t)(gyro.gyroADCf[Z]);
    
    // Send to host
    sendReport((uint8_t*)report, sizeof(report));
}
```

### Step 3: Modify USB Descriptors
You must edit the `HID_MOUSE_ReportDesc` array (located in platform-specific `usbd_hid.c` files) to accommodate the new data.
*   **Option A (Replace):** Change the meaning of existing axes (e.g., use "Slider" or "Dial" axes for Gyro).
*   **Option B (Extend):** Add a new Report ID specifically for sensor data, allowing you to send RC data (Report ID 1) and Gyro data (Report ID 2) separately.

### Step 4: Browser Side (WebHID)
In your web application:
```javascript
const device = await navigator.hid.requestDevice({ filters: [{ vendorId: 0x0483 }] });
await device[0].open();
device[0].addEventListener("inputreport", event => {
    const { data, reportId } = event;
    // Parse 'data' according to your new report structure
});
```
