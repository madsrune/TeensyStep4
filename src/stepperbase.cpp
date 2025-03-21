#include "Arduino.h"

#pragma push_macro("abs")
#undef abs

#include "stepperbase.h"
#include <algorithm>

namespace TS4
{
    StepperBase::StepperBase(int _stepPin, int _dirPin)
        : s(0), v(0), v_sqr(0), stepPin(_stepPin), dirPin(_dirPin)
    {
        pinMode(stepPin, OUTPUT);
        pinMode(dirPin, OUTPUT);

        // setMaxSpeed(vMaxDefault);
    }

    void StepperBase::startRotate(int32_t _v_tgt, uint32_t a)
    {
        v_tgt      = _v_tgt;
        v_tgt_sqr  = (int64_t)signum(v_tgt) * v_tgt * v_tgt;
        vDir       = (int32_t)signum(v_tgt_sqr - v_sqr);
        twoA       = 2 * a;

        if (!isMoving)
        {
            stpTimer = TimerFactory::makeTimer();
            stpTimer->setPulseParams(8, stepPin);
            stpTimer->attachCallbacks([this] { rotISR(); }, [this] { resetISR(); });
            v_sqr = vDir * 200 * 200;
            
            // Only set mode to rotate if we're not in stopping mode
            if (mode != mmode_t::stopping) {
                mode = mmode_t::rotate;
            }
            
            stpTimer->start();
            isMoving = true;
        }
        // No else clause needed - we always update the motion parameters
    }

    void StepperBase::startMoveTo(int32_t _s_tgt, int32_t v_e, uint32_t v_tgt, uint32_t a)
    {
        s          = 0;
        int32_t ds = std::abs(_s_tgt - pos);
        s_tgt      = ds;

        dir = signum(_s_tgt - pos);
        digitalWriteFast(dirPin, dir > 0 ? HIGH : LOW);
        delayMicroseconds(5);

        twoA = 2 * a;
        // v_sqr      = (int64_t) v * v;
        v_sqr     = 0;
        v         = 0;
        v_tgt_sqr = (int64_t)v_tgt * v_tgt;

        int64_t accLength = (v_tgt_sqr - v_sqr) / twoA + 1;
        if (accLength >= ds / 2) accLength = ds / 2;

        accEnd   = accLength - 1;
        decStart = s_tgt - accLength;

        // SerialUSB1.printf("TimerAddr: %p\n", &stpTimer);
        // SerialUSB1.printf("a: %6d   twoA:  %6d\n", a, twoA);
        // SerialUSB1.printf("v0:%6d   v_tgt: %6d\n", v, v_tgt);
        // SerialUSB1.printf("s: %6d   s_tgt: %6d\n", s, s_tgt);
        // SerialUSB1.printf("aE:%6d   dS:    %6d %d\n\n", accEnd, decStart, accLength);

        if (!isMoving)
        {
            // Serial.println("ismoving");
            stpTimer = TimerFactory::makeTimer();

            stpTimer->attachCallbacks([this] { stepISR(); }, [this] { resetISR(); });
            stpTimer->setPulseParams(8, stepPin);
            isMoving = true;
            v_sqr    = 200 * 200;
            mode     = mmode_t::target;
            stpTimer->start();
        }
    }

    // void StepperBase::rotateAsync()
    // {
    //     rotateAsync(vMax);
    // }

    void StepperBase::startStopping(int32_t v_end, uint32_t a)
    {
        if (!isMoving) return;
        
        // Check current mode before changing it
        mmode_t original_mode = mode;
        
        // Set stopping mode
        mode = mmode_t::stopping;
        
        if (original_mode == mmode_t::rotate) {
            // For rotation mode, set target speed to zero for controlled deceleration
            startRotate(v_end, a);
            mode = mmode_t::stopping; // Ensure mode remains stopping after startRotate
        }
        
        // No need for additional code for target mode as stepISR will handle it
    }

    void StepperBase::emergencyStop()
    {
        stpTimer->stop();
        TimerFactory::returnTimer(stpTimer);
        stpTimer = nullptr;
        isMoving = false;
        v_sqr    = 0;
    }

    void StepperBase::overrideSpeed(int32_t newSpeed, uint32_t acceleration)
    {
        // If acceleration is provided, update twoA
        if (acceleration > 0) {
            twoA = 2 * acceleration;
        }

        noInterrupts(); // Critical section - avoid ISR conflicts
        
        if (mode == mmode_t::rotate)
        {
            // Update target velocity for rotation mode
            v_tgt = newSpeed;
            v_tgt_sqr = (int64_t)signum(v_tgt) * v_tgt * v_tgt;
            vDir = (int32_t)signum(v_tgt_sqr - v_sqr);
        }
        else if (mode == mmode_t::target && isMoving)
        {
            // Ensure we're using the absolute value for target mode
            int32_t v_tgt_abs = std::abs(newSpeed);
            
            // Calculate position within the movement profile
            int32_t remaining = s_tgt - s;
            
            // Only recalculate profile if we're not already decelerating
            if (s < decStart) {
                // Calculate distance needed to decelerate from current speed to zero
                int64_t currentStoppingDistance = v_sqr / twoA;
                
                // Maximum remaining distance available for acceleration and constant speed
                int64_t availableDistance = remaining - currentStoppingDistance;
                
                if (availableDistance > 0) {
                    // Calculate the maximum speed that can be safely reached and then decelerated from
                    // v_max^2 / (2*a) = availableDistance => v_max^2 = 2*a*availableDistance
                    int64_t max_v_sqr = twoA * availableDistance;
                    
                    // Constrain the new target speed
                    int64_t new_v_tgt_sqr = (int64_t)v_tgt_abs * v_tgt_abs;
                    if (new_v_tgt_sqr > max_v_sqr) {
                        // Reduce the target speed if it's too high
                        v_tgt_abs = sqrtf(max_v_sqr);
                    }
                    
                    v_tgt = v_tgt_abs;
                    v_tgt_sqr = (int64_t)v_tgt * v_tgt;
                    
                    // Recalculate motion profile
                    
                    // Calculate distance needed for deceleration from the target speed
                    int64_t decDistance = v_tgt_sqr / twoA;
                    
                    // If we're still in acceleration phase
                    if (s < accEnd) {
                        // Calculate distance needed to reach target speed from current speed
                        int64_t accDistance = (v_tgt_sqr - v_sqr) / twoA;
                        
                        // If we can reach target speed
                        if (accDistance + decDistance <= remaining) {
                            // Update acceleration end point
                            accEnd = s + accDistance;
                            
                            // Update deceleration start point
                            decStart = s_tgt - decDistance;
                        } else {
                            // Not enough distance for full acceleration/deceleration
                            // Calculate peak speed we can reach
                            int64_t peak_v_sqr = (remaining * twoA) / 2;
                            
                            // Calculate distance to reach this peak speed
                            int64_t peak_accDistance = (peak_v_sqr - v_sqr) / twoA;
                            
                            // Update motion profile
                            accEnd = s + peak_accDistance;
                            decStart = accEnd + 1; // Start decelerating immediately after acceleration
                        }
                    } 
                    // If we're in constant speed phase
                    else {
                        // Update deceleration start point based on new target speed
                        decStart = s_tgt - decDistance;
                        
                        // If we need to start decelerating immediately
                        if (s >= decStart) {
                            decStart = s;
                        }
                    }
                } else {
                    // Not enough distance to change speed safely, need to decelerate now
                    decStart = s;
                    v_tgt = 0;
                    v_tgt_sqr = 0;
                }
            }
            // If we're already decelerating, don't change the profile
        }
        
        interrupts(); // End critical section
    }
}

#pragma pop_macro("abs")