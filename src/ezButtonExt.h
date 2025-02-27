/*
 * Copyright (c) 2019, ArduinoGetStarted.com. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * - Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * - Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * - Neither the name of the ArduinoGetStarted.com nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ARDUINOGETSTARTED.COM "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ARDUINOGETSTARTED.COM BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef EZBUTTONEXT_H
#define EZBUTTONEXT_H

#include <Arduino.h>

#define COUNT_FALLING 0
#define COUNT_RISING 1
#define COUNT_BOTH 2

class ezButtonExt {
   private:
    // Config
    int btnPin;
    int countMode;
    bool pressedLevel;
    unsigned long debounceTime;
    unsigned long longClickTime;

    // Variable used for logic
    bool previousSteadyState;   // the previous steady state from the input pin,
                                // used to detect pressed and released event
    bool lastSteadyState;       // the last steady state from the input pin
    bool lastFlickerableState;  // the last flickerable state from the input pin
    unsigned long lastDebounceTime;  // the last time the output pin was toggled
    bool clickDetect;                // Logic for short and long click detect
    unsigned int sequentialShortClicks;  // Counter of sequential clicks
    bool resetSequentialCont;            // Logic for reset counter

    unsigned long count;          // Counter
    unsigned long lastClickTime;  // last time the button was press

   public:
    ezButtonExt(int pin);
    ezButtonExt(int pin, int mode);
    void setDebounceTime(unsigned long time);
    void setLevelMode(bool pressedLevel);
    void setLongClickTime(unsigned long time);

    bool getState(void);
    bool getStateRaw(void);
    bool getSequentialShortClicks(unsigned int Num_Clicks);

    bool isPressed(void);
    bool isReleased(void);
    bool isLongClick(void);
    bool isShortClick(void);

    void setCountMode(int mode);
    unsigned long getCount(void);
    void resetCount(void);

    void loop(void);
};

#endif
