#ifdef ACTUATOR_HAL_H
#define ACTUATOR_HAL_H

class ActuatorHAL {
    public:
    virtual void initialize() = 0;
    virtual void actuate(float value) = 0;
    virtual void set_speed(float speed) = 0;
    virtual void calibrate() = 0;
    virtual void stop() = 0;

};

#endif // ACTUATOR_HAL_H