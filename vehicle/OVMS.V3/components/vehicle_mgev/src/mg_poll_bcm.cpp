/*
;    Project:       Open Vehicle Monitor System
;    Date:          3rd September 2020
;
;    Changes:
;    1.0  Initial release
;
;    (C) 2011       Michael Stegen / Stegen Electronics
;    (C) 2011-2017  Mark Webb-Johnson
;    (C) 2011        Sonny Chen @ EPRO/DX
;    (C) 2020       Chris Staite
;
; Permission is hereby granted, free of charge, to any person obtaining a copy
; of this software and associated documentation files (the "Software"), to deal
; in the Software without restriction, including without limitation the rights
; to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
; copies of the Software, and to permit persons to whom the Software is
; furnished to do so, subject to the following conditions:
;
; The above copyright notice and this permission notice shall be included in
; all copies or substantial portions of the Software.
;
; THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
; IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
; FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
; AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
; LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
; OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
; THE SOFTWARE.
*/

#include "ovms_log.h"
static const char *TAG = "v-mgev";

#include "vehicle_mgev.h"
#include "mg_obd_pids.h"
#include "metrics_standard.h"

namespace {

// The bitmasks for the doors being open on the BCM Door PID
enum DoorMasks : unsigned char {
    Driver = 1,
    Passenger = 2,
    RearLeft = 4,
    RearRight = 8,
    Bonnet = 16,
    Boot = 32,
    Locked = 128
};

uint32_t iterate(uint32_t seed, uint32_t count)
{
    while (count--)
    {
        seed = (seed << 1u) | ((((((((seed >> 6u) ^ seed) >> 12u) ^ seed) >> 10u) ^ seed) >> 2u) & 1u);
    }
    return seed;
}

uint32_t pass(uint32_t seed)
{
    uint32_t count = 0x2bu + (((seed >> 0x18u) & 0x17u) ^ 0x02u);
    return iterate(seed, count) ^ 0x594e348au;
}

}  // anon namespace

void OvmsVehicleMgEv::IncomingBcmPoll(uint16_t pid, uint8_t* data, uint8_t length)
{
    switch (pid)
    {
        case bcmDoorPid:
            StandardMetrics.ms_v_door_fl->SetValue(data[0] & Passenger);
            StandardMetrics.ms_v_door_fr->SetValue(data[0] & Driver);
            StandardMetrics.ms_v_door_rl->SetValue(data[0] & RearLeft);
            StandardMetrics.ms_v_door_rr->SetValue(data[0] & RearRight);
            StandardMetrics.ms_v_door_trunk->SetValue(data[0] & Boot);
            break;
        case bcmLightPid:
            StandardMetrics.ms_v_env_headlights->SetValue(data[0] > 1);
            break;
    }    
}

bool OvmsVehicleMgEv::StartBcmAuthentication(canbus* currentBus)
{
    CAN_frame_t authStart = {
        currentBus,
        nullptr,
        { .B = { 2, 0, CAN_no_RTR, CAN_frame_std, 0 } },
        bcmId,
        { .u8 = {
            (ISOTP_FT_FIRST<<4), 3, 0, 0, 0, 0, 0, 0
        } }
    };
    return currentBus->Write(&authStart) != ESP_FAIL;
}

void OvmsVehicleMgEv::BcmAuthentication(canbus* currentBus, uint8_t frameType, uint8_t* data)
{
    CAN_frame_t nextFrame = {
        currentBus,
        nullptr,
        { .B = { 0, 0, CAN_no_RTR, CAN_frame_std, 0 } },
        bcmId,
        { .u8 = {
            0, 0, 0, 0, 0, 0, 0, 0
        } }
    };

    bool known = false;

    if (frameType == ISOTP_FT_FIRST)
    {
        if (*data == 3u)
        {
            // Start authentication
            nextFrame.data.u8[0] = (ISOTP_FT_FLOWCTRL<<4) + 0xeu;
            nextFrame.data.u8[1] = 0x00u;
            nextFrame.FIR.B.DLC = 2u;
            known = true;
        }
    }
    else if (frameType == ISOTP_FT_FLOWCTRL)
    {
        // Request seed
        nextFrame.data.u8[0] = (ISOTP_FT_CONSECUTIVE<<4) + 7;
        nextFrame.data.u8[1] = 0x01u;
        nextFrame.FIR.B.DLC = 2u;
        known = true;
    }
    else if (frameType == ISOTP_FT_CONSECUTIVE)
    {
        if (*data == 0x01u)
        {
            // Seed response
            uint32_t seed = (data[1] << 24) | (data[2] << 16) | (data[3] << 8) | data[4];
            uint32_t key = pass(seed);
            nextFrame.data.u8[0] = (ISOTP_FT_CONSECUTIVE<<4) + 7;
            nextFrame.data.u8[1] = 0x02u;
            nextFrame.data.u8[2] = key >> 24u;
            nextFrame.data.u8[3] = key >> 16u;
            nextFrame.data.u8[4] = key >> 8u;
            nextFrame.data.u8[5] = key;
            nextFrame.FIR.B.DLC = 6u;
            known = true;
        }
        else if (*data == 0x02u)
        {
            // Seed accept, request TP
            SendTesterPresentTo(currentBus, bcmId);
        }
    }

    if (known)
    {
        if (currentBus->Write(&nextFrame) == ESP_FAIL) {
            ESP_LOGE(TAG, "Error writing BCM authentication frame");
        }
    }
}
