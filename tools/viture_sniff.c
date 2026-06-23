/*
 * viture_sniff.c - Capture raw HID reports from Viture glasses
 *
 * Build: clang -framework IOKit -framework CoreFoundation -o viture_sniff viture_sniff.c
 * Run:   ./viture_sniff
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <IOKit/hid/IOHIDManager.h>
#include <CoreFoundation/CoreFoundation.h>

#define VITURE_VID 0x35CA  // 13770
#define VITURE_PID 0x1201  // 4609 - Beast XR Glasses

static int report_count = 0;

static void print_hex(const uint8_t *data, size_t len) {
    for (size_t i = 0; i < len; i++) {
        printf("%02x ", data[i]);
        if ((i + 1) % 16 == 0) printf("\n       ");
    }
    printf("\n");
}

static void input_report_callback(void *context, IOReturn result, void *sender,
                                   IOHIDReportType type, uint32_t reportID,
                                   uint8_t *report, CFIndex reportLength) {
    report_count++;

    // Print every 240th report (~1 per second at 240Hz)
    if (report_count % 240 == 1 || report_count <= 5) {
        printf("[%d] Report ID=%d, len=%ld, type=%d\n",
               report_count, reportID, (long)reportLength, type);
        printf("  raw: ");
        print_hex(report, reportLength > 64 ? 64 : reportLength);

        // Try to interpret as IMU data
        if (reportLength >= 40) {
            // Hypothesis 1: floats at offset 0
            float *f = (float *)report;
            printf("  as float[0-8]: %.4f %.4f %.4f | %.4f %.4f %.4f | %.4f %.4f %.4f\n",
                   f[0], f[1], f[2], f[3], f[4], f[5], f[6], f[7], f[8]);

            // Hypothesis 2: int16 scaled values
            int16_t *s = (int16_t *)report;
            printf("  as int16[0-8]: %d %d %d | %d %d %d | %d %d %d\n",
                   s[0], s[1], s[2], s[3], s[4], s[5], s[6], s[7], s[8]);

            // Hypothesis 3: header byte + data
            if (report[0] != 0) {
                printf("  header byte: 0x%02x, subtype: 0x%02x\n", report[0], report[1]);
                float *f2 = (float *)(report + 4);
                printf("  float@4: %.4f %.4f %.4f | %.4f %.4f %.4f\n",
                       f2[0], f2[1], f2[2], f2[3], f2[4], f2[5]);
            }
        }
        printf("\n");
    }
}

static void device_matched_callback(void *context, IOReturn result,
                                     void *sender, IOHIDDeviceRef device) {
    CFStringRef product = IOHIDDeviceGetProperty(device, CFSTR(kIOHIDProductKey));
    char name[256] = "Unknown";
    if (product) CFStringGetCString(product, name, sizeof(name), kCFStringEncodingUTF8);

    printf("Device matched: %s\n", name);

    // Register for input reports
    static uint8_t report_buffer[256];
    IOHIDDeviceRegisterInputReportCallback(device, report_buffer, sizeof(report_buffer),
                                            input_report_callback, NULL);

    // Open the device
    IOReturn ret = IOHIDDeviceOpen(device, kIOHIDOptionsTypeNone);
    if (ret != kIOReturnSuccess) {
        printf("Failed to open device: 0x%x\n", ret);
        // Try seize mode
        ret = IOHIDDeviceOpen(device, kIOHIDOptionsTypeSeizeDevice);
        if (ret != kIOReturnSuccess) {
            printf("Failed to seize device: 0x%x (SpaceWalker may be using it)\n", ret);
        }
    }

    printf("Listening for HID reports... (Ctrl+C to stop)\n\n");
}

static void device_removed_callback(void *context, IOReturn result,
                                     void *sender, IOHIDDeviceRef device) {
    printf("Device removed\n");
}

int main(int argc, char **argv) {
    printf("Viture HID Sniffer - Looking for VID=0x%04X PID=0x%04X\n\n", VITURE_VID, VITURE_PID);

    IOHIDManagerRef manager = IOHIDManagerCreate(kCFAllocatorDefault, kIOHIDOptionsTypeNone);
    if (!manager) {
        fprintf(stderr, "Failed to create HID manager\n");
        return 1;
    }

    // Match Viture glasses
    CFMutableDictionaryRef match = CFDictionaryCreateMutable(
        kCFAllocatorDefault, 0, &kCFTypeDictionaryKeyCallBacks, &kCFTypeDictionaryValueCallBacks);

    int vid = VITURE_VID, pid = VITURE_PID;
    CFNumberRef vidNum = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &vid);
    CFNumberRef pidNum = CFNumberCreate(kCFAllocatorDefault, kCFNumberIntType, &pid);
    CFDictionarySetValue(match, CFSTR(kIOHIDVendorIDKey), vidNum);
    CFDictionarySetValue(match, CFSTR(kIOHIDProductIDKey), pidNum);
    CFRelease(vidNum);
    CFRelease(pidNum);

    IOHIDManagerSetDeviceMatching(manager, match);
    CFRelease(match);

    IOHIDManagerRegisterDeviceMatchingCallback(manager, device_matched_callback, NULL);
    IOHIDManagerRegisterDeviceRemovalCallback(manager, device_removed_callback, NULL);

    IOHIDManagerScheduleWithRunLoop(manager, CFRunLoopGetCurrent(), kCFRunLoopDefaultMode);
    IOHIDManagerOpen(manager, kIOHIDOptionsTypeNone);

    printf("Waiting for device...\n");
    CFRunLoopRun();

    return 0;
}
