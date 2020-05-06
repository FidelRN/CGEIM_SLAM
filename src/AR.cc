#include "AR.h"

namespace ORB_SLAM2
{

AR::AR(long unsigned int pID, bool isOrigin)
{
    if (isOrigin){
        this->originID = pID;
        this->originValid = true;
        this->scaleValid = false;
    }
    else {
        this->scaleID = pID;
        this->scaleValid = true;
        this->originValid = false;    
    }
    
    this->valid = false;
}

bool AR::SetOriginID(long unsigned int pID)
{
    // No changes available if it is valid
    if (this->valid)
        return false;
    // No change if it is the same point as scale
    if (this->scaleValid && this->scaleID == pID)
        return false;

    this->originID = pID;
    this->originValid = true;
    return true;
}

bool AR::SetScaleID(long unsigned int pID)
{
    // No changes available if it is valid
    if (this->valid)
        return false;
    // No change if it is the same point as origin
    if (this->originValid && this->originID == pID)
        return false;

    this->scaleID = pID;
    this->scaleValid = true;
    return true;
}

bool AR::SetValid()
{
    if (this->originValid && this->scaleValid){
        this->valid = true;
        return true;
    }
    return false;
}

} //namespace ORB_SLAM
