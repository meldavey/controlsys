#pragma once

#include <stdint.h>

class ControlSystem {
    public:
        ControlSystem();
        virtual ~ControlSystem();

        virtual void Update();

    enum CSType {
        CSType_UpdateNone,
        CSType_UpdateTarget,
        CSType_UpdateMax,
    };

    protected:
        void UpdateTarget();
        void UpdateMax();

        void InitSystem() {}
        virtual double MeasureSystem() = 0;
        virtual void SetSystem(int32_t pos) = 0;
        virtual int32_t GetSystem() = 0;

        const char *systemName; // name used for debug prints
        CSType systemType;      // control system type for sub-classes
        double target;          // our target measurement to hit
        double targetK;         // adaptive learned linear adjustment rate
        double deltaThresh;     // maintain within this percent of target
        int32_t maxStep;        // limit the physical control systems single adjustment to +/- this value
        int32_t curPos;         // current physical position
        uint32_t waitMS;        // optional ms to wait after making an adjustment and before we can make a new value measurement
        int direction;          // for max (dither) algorithms, use this direction when adjusting
        int directionChanges;   // counts consecutive direction changes, used to adjust to new (lower) target
};

