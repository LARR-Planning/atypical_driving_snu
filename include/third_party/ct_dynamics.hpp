#pragma once
#include <ct/core/core.h>  // as usual, include CT
// create a class that derives from ct::core::System
class CarDynamics : public ct::core::ControlledSystem<4, 2, double>
{
    private:
    double invL_;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    static const size_t STATE_DIM = 4;
    static const size_t CONTROL_DIM = 2;
    // constructor
    CarDynamics(double invL) : invL_(invL) {}
    // copy constructor
    CarDynamics(const CarDynamics& other) : invL_(other.invL_) {}
    // destructor
    ~CarDynamics() = default;

    // clone method for deep copying
    CarDynamics* clone() const override
    {
        return new CarDynamics(*this);  // calls copy constructor
    }
    
    // The system dynamics. We override this method which gets called by e.g. the Integrator
    void computeControlledDynamics(const ct::core::StateVector<STATE_DIM>& state,
                         const ct::core::Time& t,
                         const ct::core::ControlVector<CONTROL_DIM>& control,
                         ct::core::StateVector<STATE_DIM>& derivative) override
    {   
        double x= state(0); double y = state(1);
        double v = state(2); double th = state(3);
        double acc = control(0); double del = control(1);

        derivative(0) = v * std::cos(th);
        derivative(1) = v * std::sin(th);
        derivative(2) = acc; 
        derivative(3) = v* std::tan(del)*invL_;
    }
};