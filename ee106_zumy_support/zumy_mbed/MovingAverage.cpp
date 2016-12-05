#include "mbed.h"

#include "MovingAverage.h"

MovingAverage::MovingAverage(int maxLength, float defaultValue){
    MaxLength = maxLength;

    Element = new float[MaxLength];

    for(int i = 0; i<MaxLength;i++)
    {
        Element[i] = defaultValue;
    }
    Average = defaultValue;

    NextElement = 0;
}

float MovingAverage::GetAverage(){
    return Average;
}

void MovingAverage::Insert(float value)
{

    Average = ( value / ((float) MaxLength)) + Average - (Element[NextElement]/ ((float) MaxLength));
    Element[NextElement] = value;

    NextElement = NextElement +1;

    if(NextElement>= (MaxLength) )
    {
        NextElement=0;
    }

}
