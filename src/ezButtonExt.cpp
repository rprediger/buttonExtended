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

#include <ezButtonExt.h>

ezButtonExt::ezButtonExt(int pin) : ezButtonExt(pin, INPUT_PULLUP){};

ezButtonExt::ezButtonExt(int pin, int mode) {
    btnPin = pin;
    debounceTime = 0;
    count = 0;
    countMode = COUNT_FALLING;
    pressedLevel = LOW;

    pinMode(btnPin, mode);

    previousSteadyState = digitalRead(btnPin);
    lastSteadyState = previousSteadyState;
    lastFlickerableState = previousSteadyState;

    lastDebounceTime = 0;
    longClickTime = 350;
    clickDetect = true;           // Prevents long press detection
    sequentialShortClicks = 0;    // Counter of sequential clicks
    resetSequentialCont = false;  // Logic for reset counter
}

void ezButtonExt::setDebounceTime(unsigned long time) { debounceTime = time; }

void ezButtonExt::setLevelMode(bool pressedLevel) {
    this->pressedLevel = pressedLevel;
}

void ezButtonExt::setLongClickTime(unsigned long time) { longClickTime = time; }

bool ezButtonExt::getState(void) { return lastSteadyState; }

bool ezButtonExt::getStateRaw(void) { return digitalRead(btnPin); }

bool ezButtonExt::getSequentialShortClicks(unsigned int Num_Clicks) {
    // If the last click was a long time ago
    if ((millis() - lastDebounceTime) >= longClickTime) {
        resetSequentialCont = true;
        // check if it is the expected case
        if (sequentialShortClicks == Num_Clicks) {
            return true;
        }
    }
    return false;
}

bool ezButtonExt::isPressed(void) {
    if (previousSteadyState != pressedLevel && lastSteadyState == pressedLevel)
        return true;
    else
        return false;
}

bool ezButtonExt::isReleased(void) {
    if (previousSteadyState == pressedLevel && lastSteadyState != pressedLevel)
        return true;
    else
        return false;
}

bool ezButtonExt::isLongClick(void) {
    if (clickDetect == false) {
        if ((millis() - lastClickTime) >= longClickTime) {
            clickDetect = true;
            return true;
        }
    }
    return false;
}

bool ezButtonExt::isShortClick(void) {
    if (isReleased()) {
        if ((millis() - lastClickTime) < longClickTime) {
            clickDetect = true;
            return true;
        }
    }
    return false;
}

void ezButtonExt::setCountMode(int mode) { countMode = mode; }

unsigned long ezButtonExt::getCount(void) { return count; }

void ezButtonExt::resetCount(void) { count = 0; }

void ezButtonExt::loop(void) {
    // read the state of the switch/button:
    int currentState = digitalRead(btnPin);
    unsigned long currentTime = millis();

    // check to see if you just pressed the button
    // (i.e. the input went from LOW to HIGH), and you've waited long enough
    // since the last press to ignore any noise:

    // If the switch/button changed, due to noise or pressing:
    if (currentState != lastFlickerableState) {
        // reset the debouncing timer
        lastDebounceTime = currentTime;
        // save the the last flickerable state
        lastFlickerableState = currentState;
    }

    if ((currentTime - lastDebounceTime) >= debounceTime) {
        // whatever the reading is at, it's been there for longer than the
        // debounce delay, so take it as the actual current state:

        // save the the steady state
        previousSteadyState = lastSteadyState;
        lastSteadyState = currentState;
    }

    if (previousSteadyState != lastSteadyState) {
        // Counter
        if (countMode == COUNT_BOTH) {
            count++;
        } else if (countMode == COUNT_FALLING) {
            if (isPressed()) count++;
        } else if (countMode == COUNT_RISING) {
            if (isReleased()) count++;
        }
        // Short and Long click
        if (isPressed()) {
            lastClickTime = lastDebounceTime;
            // clickDetect = false;
        }

        // Sequential Clicks
        if (isShortClick()) {
            sequentialShortClicks++;
        }
    }

    // Sequential Clicks
    if (resetSequentialCont == true) {
        sequentialShortClicks = 0;
        resetSequentialCont = false;
    }
}