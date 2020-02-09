class MPULib{
public:
    static volatile bool isMPUReady;

private:
    MPU6050 mpu;
    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t gxPrev;
    unsigned long timeStamp;
    double accFx, accFy, accFz, gyFx, accP;
    double dT, alphaF, Tau, pitch, pitchRate, pitchPrev;

    void readMPUData(void);
    
}