#ifndef ShapeMod_
#define ShapeMod_

#include "IKAROS.h"
#include <vector>

class ShapeMod: public Module
{
public:
    static Module * Create(Parameter * p) { return new ShapeMod(p); }

    float ** iminc;
    float ** imout;
    //float ** imine;

    int siX;
    int siY;
    int noR;
    int nodl;
    //int mix;
    //int max;
    //int miy;
    //int may;

    float pi;

    float * err;
    float * cin;
    float * cout;
    float * norou;
    float * shapeIn;
    float * shapeDir;

    //std::vector<std::vector<bool>> goDi;
    std::vector<std::vector<float>> shape;
    std::vector<std::vector<std::vector<std::vector<float>>>> proSh; 

    ShapeMod(Parameter * p) : Module(p) {}
    virtual ~ShapeMod() {}

    int eval(int je, int ke, int le, int ie);

    void Init();
    void Tick();
    void findShape();
    void makeShape();
    void printShape();
};

#endif

