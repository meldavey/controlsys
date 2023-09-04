#include <stdio.h>
#include <math.h>
#include "ControlSystem.h"

class LinearTest : public ControlSystem {
    public:
        LinearTest() {}
        ~LinearTest() {}

        void InitSystem() {
            systemName = "Linear Test";
            systemType = CSType_UpdateTarget;
        
            target = 100.0; // our target measurement to hit
            targetK = 0.5; // adaptive learned linear adjustment rate (actual K is 1.0)
            deltaThresh = 0.02; // maintain within 2% of target
            maxStep = 10; // limit the attenuator motor to +/- this many steps
            curPos = 90; // current attenuator motor position
            waitMS = 500; // system needs 500ms before a new value can be taken after an attenuator change
        }

        int testvar1 = 200;
    protected:

        void SetSystem(int32_t pos) {
            curPos = pos;
        }

        int32_t GetSystem() {
            return curPos;
        }

        double MeasureSystem() {
            double curValue = testvar1 - curPos; // note increase in position results in decrease in value to mimic attenuator motor
            return curValue;
        }
};

class MaxTest : public ControlSystem {
    public:
        MaxTest() {}
        ~MaxTest() {}

        void InitSystem() {
            systemName = "Max Test";
            systemType = CSType_UpdateMax;
        
            target = 100.0; // our target measurement to hit
            targetK = 2.0; // adaptive learned linear adjustment rate (actual K is 1.0)
            deltaThresh = 0.02; // maintain within 2% of target
            maxStep = 10; // limit the X motor to +/- this many steps
            curPos = 501; // current attenuator motor position
            waitMS = 500; // system needs 500ms before a new value can be taken after an attenuator change
        }

        double testvar1 = 0;
        double testvar2 = 5.0;
    protected:

        void SetSystem(int32_t pos) {
            curPos = pos;
        }

        int32_t GetSystem() {
            return curPos;
        }

        double MeasureSystem() {
            double curValue = 100.0 + (500 + testvar1*testvar2 - curPos);
            return curValue;
        }
};

class MaxTest2 : public MaxTest {
    protected:

        // measured value is always less than 125, and varies with sin, and curPos is used to counter the impact of the ever-changing sin term - so this is a challenge
        double MeasureSystem() {
            double curValue = 125.0 - fabs(500 - curPos + testvar1*testvar2);
            return curValue;
        }
};

class MaxTest3 : public MaxTest {
    public:
        void Update() {
            offset += 0.25;
            MaxTest::Update();
        }

    protected:

        // measured value is always less than 125, and varies with sin, and curPos is used to counter the impact of the ever-changing sin term - so this is a challenge
        double MeasureSystem() {
            double curValue = 125.0 - fabs(500 - curPos + testvar1*testvar2) - offset;
            return curValue;
        }

        double offset = 0.0;
};

////////

int main(int argc, char *argv[])
{
    printf("Starting linear test\n");
    LinearTest lt;
    lt.InitSystem();

    for(int i=0;i<100;i++) {
        lt.Update();
        if ((i%5) == 0)
            lt.testvar1++;
    }
    printf("End linear test\n\n");

    printf("Starting max test\n");
    MaxTest3 mt;
    mt.InitSystem();

    for(int i=0;i<100;i++) {
        mt.Update();
        mt.testvar1 = sin(i/100.0*3.141592654*2.0*3.0); // 3 periods of sin wave
    }
    printf("End max test\n\n");

}

