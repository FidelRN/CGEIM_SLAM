#ifndef AR_H
#define AR_H

namespace ORB_SLAM2
{


class AR
{
public:
    AR(long unsigned int pID, bool isOrigin);

    bool SetOriginID(long unsigned int pID);
    bool SetScaleID(long unsigned int pID);

    bool SetValid();

    long unsigned int originID, scaleID;

    bool valid;
    bool originValid, scaleValid;

    // TODO: Algo para guardar la estructura de AR (tipo obj)
    

};

} //namespace ORB_SLAM

#endif // AR_H
