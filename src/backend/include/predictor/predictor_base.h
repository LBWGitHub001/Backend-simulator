//
// Created by lbw on 25-3-3.
//

#ifndef PREDICTOR_BASE_H
#define PREDICTOR_BASE_H

#include "Eigen/Dense"

class PredictorBase {
public:
    PredictorBase();
    virtual ~PredictorBase();
    virtual void reset(Eigen::MatrixXd state) = 0;
    virtual void predict(Eigen::MatrixXd state,double dt) = 0;
  private:

};



#endif //PREDICTOR_BASE_H
