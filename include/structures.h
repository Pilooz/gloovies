/*
    Structures de données.
*/

struct mpu6050_struct {
    int16_t AcX; 
    int16_t AcY; 
    int16_t AcZ; 
    int16_t Tmp; 
    int16_t GyX; 
    int16_t GyY; 
    int16_t GyZ;
};