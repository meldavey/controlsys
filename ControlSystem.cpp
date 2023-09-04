#include "ControlSystem.h"

#include <stdio.h>
#include <math.h>
#include <assert.h>
#include <thread>
#include <cmath>

#define DPLOG_INFO(...) {printf(__VA_ARGS__); printf("\n");}

ControlSystem::ControlSystem()
{
    // setting up defaults as an example, these should all be set in the sub-class's InitSystem method

    systemName = "Generic";

    target = 0; // our target measurement to hit
    targetK = 0.5; // adaptive learned linear adjustment rate
    deltaThresh = 0.02; // maintain within 2% of target
    maxStep = 10; // limit the physical control systems single adjustment to +/- this value
    curPos = 0; // current physical position
    waitMS = 0; // optional ms to wait after making an adjustment and before we can make a new value measurement
    direction = 1;
    directionChanges = 0;

    InitSystem();
}

ControlSystem::~ControlSystem()
{
}

void ControlSystem::Update()
{
    if (systemType == CSType_UpdateTarget)
        UpdateTarget();
    else if (systemType == CSType_UpdateMax)
        UpdateMax();
}

void ControlSystem::UpdateTarget()
{
    double currentValue = MeasureSystem(); // Measure is a pure virtual method implemented in subclasses
    double deltaValue = currentValue - target;

    // DPLOG_INFO("in UpdateTarget with current value: %lf\n", currentValue);

    // if delta is large enough, we need to correct
    if (fabs(deltaValue) > target*deltaThresh) { // 2%
        // calculate how much to adjust by (it's an integer because we ultimatly move a mechanical motor by a physical step)
        int32_t adjust = std::lround(deltaValue * targetK);

        // if the adjustment was small (it rounded to zero) move one step in the direction of the desired change
        // also limit to +/- maxStep to avoid very large changes (this should be a tuned param for your specific system)
        if (adjust == 0)
            adjust = (deltaValue > 0 ? 1 : -1);
        else if (adjust < -maxStep)
            adjust = -maxStep;
        else if (adjust > maxStep)
            adjust = maxStep;

        assert(adjust != 0); // delta should never be zero here

        // update to new position
        curPos += adjust;

        // log our input and adjustment to the system
        DPLOG_INFO("system: %s  target: %lf  current: %lf  delta: %lf  adjustment: %d", systemName, target, currentValue, deltaValue, adjust);

        // actually set our control system to new position
        SetSystem(curPos);

        // we will now take another measurement to see how we did.
        // Some systems require settling time after an adjustment before another measurement can occur, so we wait first
        if (waitMS > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(waitMS));

        // now, get the new system output value and update K
        double newValue = MeasureSystem();
        deltaValue = newValue - currentValue;

        if (deltaValue != 0) {
            double newK = fabs(adjust / deltaValue);
            // K (our adaptive linear system) is updated based on the learned result of our control system change
            // we also impose some reasonable adaptive change limits, so initial targetK setting is important
            if (newK > targetK*2.0) newK = targetK*2.0;
            if (newK < targetK*0.5) newK = targetK*0.5;
            double oldTargetK = targetK; // save so we can print it
            targetK = 0.8 * targetK + 0.2 * newK;
            DPLOG_INFO("system: %s  old value: %lf  new value: %lf  delta: %lf  old K: %lf  new K: %lf  adapted K: %lf",
                systemName, currentValue, newValue, deltaValue, oldTargetK, newK, targetK);
        }
    }
}

void ControlSystem::UpdateMax()
{
    double currentValue = MeasureSystem();
    double deltaValue = currentValue - target;

    // if we are below our last max by at least the threshold percentage, make a correction
    if (deltaValue < -deltaThresh * target) {
        // make a move (step) in the last known good direction, proportional to our delta value
        int32_t adjust = std::lround(fabs(deltaValue) * targetK) * direction;

        // if the adjustment was small (it rounded to zero) move one step in the direction of the desired change
        // also limit to +/- maxStep to avoid very large changes (this should be a tuned param for your specific system)
        if (adjust == 0)
            adjust = (deltaValue > 0 ? 1 : -1);
        else if (adjust < -maxStep)
            adjust = -maxStep;
        else if (adjust > maxStep)
            adjust = maxStep;

        assert(adjust != 0); // delta should never be zero here

        // update to new position
        curPos += adjust;

        // log our input and adjustment to the system
        DPLOG_INFO("system: %s  target: %lf  current: %lf  delta: %lf  adjustment: %d   newPos: %d", systemName, target, currentValue, deltaValue, adjust, curPos);

        // actually set our control system to new position
        SetSystem(curPos);

        // we will now take another measurement to see how we did.
        // Some systems require settling time after an adjustment before another measurement can occur, so we wait first
        if (waitMS > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(waitMS));

        // now, get the new system output value and update K
        double newValue = MeasureSystem();
        double deltaFromAdjust = newValue - currentValue;

        if (deltaFromAdjust != 0) {
            double newK = fabs(adjust / deltaFromAdjust);
            // K (our adaptive linear system) is updated based on the learned result of our control system change
            // we also impose some reasonable adaptive change limits, so initial targetK setting is important
            if (newK > targetK*2.0) newK = targetK*2.0;
            if (newK < targetK*0.5) newK = targetK*0.5;
            double oldTargetK = targetK; // save so we can print it
            targetK = 0.8 * targetK + 0.2 * newK;
            DPLOG_INFO("system: %s  old value: %lf  new value: %lf  delta: %lf  old K: %lf  new K: %lf  adapted K: %lf",
                systemName, currentValue, newValue, deltaFromAdjust, oldTargetK, newK, targetK);

            if (deltaFromAdjust < 0) {
                DPLOG_INFO("system: %s  moved in wrong direction, reversing and reverting", systemName);
                // if new value is worse than old value, revert the change and reverse direction
                curPos -= adjust; // TODO - tempting to move a little bit more on the opposite direction in this single step rather than wait until next time around
                SetSystem(curPos);
                direction = -direction;
                directionChanges++;
                if (directionChanges >= 3) {
                    DPLOG_INFO("system: %s  adjusting target down to: %lf", systemName, currentValue);
                    target = currentValue;
                    directionChanges = 0;
                }
            } else if (newValue > target) {
                // else if new value is better, keep it! (note this only happens if we were previously below our target)
                DPLOG_INFO("system: %s  setting new target to adjusted best: %lf", systemName, newValue);
                target = newValue;
            }

            if (deltaFromAdjust >= 0)
                directionChanges = 0;
        }

    // else if we are above our target by the threshold percentage, make this our new target value
    // using the threshold keep us from needing to make frequent adjustments, we only do so when we are below the target
    } else if (deltaValue > deltaThresh * target) {
        DPLOG_INFO("system: %s  setting new target to measured %lf", systemName, currentValue);
        target = currentValue;
    }
}

