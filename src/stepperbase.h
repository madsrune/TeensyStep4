#pragma once

#include "Arduino.h"
#pragma push_macro("abs")
#undef abs

#include "timers/interfaces.h"
#include "timers/timerfactory.h"
#include <algorithm>
#include <cstdint>
#include <string>

namespace TS4
{
    class StepperBase
    {
     public:
        std::string name;
        bool isMoving = false;
        void emergencyStop();
        void overrideSpeed(int32_t newSpeed);

        // Add enum class definition outside of protected for Stepper access
        enum class mmode_t {
            target,
            rotate,
            stopping,
        };
        
        // Add a getter to access the current mode
        mmode_t getMode() const { return mode; }

     protected:
        StepperBase(const int stepPin, const int dirPin);

        void startMoveTo(int32_t s_tgt, int32_t v_e, uint32_t v_max, uint32_t a);
        void startRotate(int32_t v_max, uint32_t a);
        void startStopping(int32_t va_end, uint32_t a);


        inline void setDir(int d);
        int32_t dir;
        int32_t vDir;

        volatile int32_t pos = 0;
        volatile int32_t target;

        int32_t s_tgt;
        int32_t v_tgt;
        int64_t v_tgt_sqr;

        int32_t twoA;
        int32_t decStart, accEnd;

        volatile int32_t s;
        volatile int32_t v;
        volatile int64_t v_sqr;

        inline void doStep();

        const int stepPin, dirPin;

        ITimer* stpTimer;
        inline void stepISR();
        inline void rotISR();
        inline void resetISR();

        mmode_t mode = mmode_t::target;

        // Bresenham:
        StepperBase* next = nullptr; // linked list of steppers, maintained from outside
        int32_t A, B;                // Bresenham parameters (https://en.wikipedia.org/wiki/Bresenham)

        friend class StepperGroupBase;
        friend class Stepper; // Add Stepper as a friend class for direct access
    };
    
    //========================================================================================================
    // Inline implementation
    //========================================================================================================

    void StepperBase::doStep()
    {
        digitalWriteFast(stepPin, HIGH);
        s += 1;
        pos += dir;

        StepperBase* stepper = next;
        while (stepper != nullptr) // move slave motors if required
        {
            if (stepper->B >= 0)
            {
                digitalWriteFast(stepper->stepPin, HIGH);
                stepper->pos += stepper->dir;
                stepper->B -= this->A;
            }
            stepper->B += stepper->A;
            stepper = stepper->next;
        }
    }

    void StepperBase::stepISR()
    {
        // Setup phase - handle stopping mode at the start
        if (mode == mmode_t::stopping) {
            // When stopping, always target zero velocity
            v_tgt_sqr = 0;
            
            // Force immediate deceleration regardless of current phase
            if (s < decStart) {
                // If we're in acceleration or constant speed phase,
                // calculate distance needed to stop and begin deceleration
                int32_t stoppingDistance = v_sqr / twoA;  // twoA is already 2*a
                accEnd = s;       // End acceleration immediately
                decStart = s;     // Start deceleration immediately
                s_tgt = s + stoppingDistance;
            }
            // If already in deceleration phase, continue with current parameters
        }

        // Execution phase - use the parameters set above
        if (s < accEnd) { 
            // In acceleration phase - use twoA to adjust velocity
            v_sqr += twoA;
            v = signum(v_sqr) * sqrtf(std::abs(v_sqr));
            stpTimer->updateFrequency(std::abs(v));
            doStep();
        } 
        else if (s < decStart) { 
            // In constant speed phase
            v = std::min(sqrtf(v_sqr), sqrtf(v_tgt_sqr));
            stpTimer->updateFrequency(std::abs(v));
            doStep();
        } 
        else if (s < s_tgt) { 
            // In deceleration phase
            v_sqr -= twoA;
            
            // Check if we've decelerated to zero or below
            if (v_sqr <= 0) {
                v_sqr = 0;
                v = 0;
                
                // Update target to match actual position if in stopping mode
                if (mode == mmode_t::stopping) {
                    target = pos;
                }
                
                // Clean up and stop
                stpTimer->stop();
                TimerFactory::returnTimer(stpTimer);
                stpTimer = nullptr;
                
                auto *cur = this;
                while (cur != nullptr) {
                    auto *tmp = cur->next;
                    cur->next = nullptr;
                    cur = tmp;
                }
                
                isMoving = false;
                return;
            }
            
            v = signum(v_sqr) * sqrtf(std::abs(v_sqr));
            stpTimer->updateFrequency(std::abs(v));
            doStep();
        } 
        else { 
            // Target reached
            // Update target to match actual position if in stopping mode
            if (mode == mmode_t::stopping) {
                target = pos;
            }
            
            stpTimer->stop();
            TimerFactory::returnTimer(stpTimer);
            stpTimer = nullptr;
            
            auto *cur = this;
            while (cur != nullptr) {
                auto *tmp = cur->next;
                cur->next = nullptr;
                cur = tmp;
            }
            
            isMoving = false;
        }
    }

    void StepperBase::rotISR()
    {
        // Set to rotate mode unless we're stopping
        if (mode != mmode_t::stopping) {
            mode = mmode_t::rotate;
        }
        
        int32_t v_abs;

        if (std::abs(v_sqr - v_tgt_sqr) > twoA) // target speed not yet reached
        {
            // If we're stopping, decelerate regardless of target speed
            if (mode == mmode_t::stopping) {
                // Decelerate toward zero
                v_sqr -= vDir * twoA;
                
                // If we've decelerated to near zero or crossed zero, stop completely
                if ((vDir > 0 && v_sqr <= 0) || (vDir < 0 && v_sqr >= 0)) {
                    v_sqr = 0;
                    // Update target to current position since we're stopping here
                    target = pos;
                    
                    // Clean up and stop
                    stpTimer->stop();
                    TimerFactory::returnTimer(stpTimer);
                    stpTimer = nullptr;
                    
                    auto *cur = this;
                    while (cur != nullptr) {
                        auto *tmp = cur->next;
                        cur->next = nullptr;
                        cur = tmp;
                    }
                    
                    isMoving = false;
                    return;
                }
            } else {
                // Normal acceleration/deceleration toward target speed
                v_sqr += vDir * twoA;
            }

            dir = signum(v_sqr);
            digitalWriteFast(dirPin, dir > 0 ? HIGH : LOW);
            delayMicroseconds(5);

            v_abs = sqrtf(std::abs(v_sqr));
            stpTimer->updateFrequency(v_abs);
            doStep();
        } 
        else // At target speed
        {
            dir = signum(v_sqr);
            digitalWriteFast(dirPin, dir > 0 ? HIGH : LOW);
            delayMicroseconds(5);

            if (v_tgt != 0 || mode != mmode_t::stopping)
            {
                v_abs = sqrtf(std::abs(v_sqr));
                stpTimer->updateFrequency(v_abs);
                doStep();
            } 
            else // We're at target speed of 0 or stopping mode reached 0
            {
                // Update target to current position since we're stopping here
                target = pos;
                
                stpTimer->stop();
                TimerFactory::returnTimer(stpTimer);
                stpTimer = nullptr;
                v_sqr = 0;

                auto *cur = this;
                while (cur != nullptr) {
                    auto *tmp = cur->next;
                    cur->next = nullptr;
                    cur = tmp;
                }
                
                isMoving = false;
            }
        }
    }

    void StepperBase::resetISR()
    {
        // Serial.println("r");
        // Serial.flush();
        StepperBase* stepper = this;
        while (stepper != nullptr)
        {
            digitalWriteFast(stepper->stepPin, LOW);
            stepper = stepper->next;
        }
    }
}
#pragma pop_macro("abs")