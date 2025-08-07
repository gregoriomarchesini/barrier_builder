
#ifndef BMSG_H
#define BMSG_H


#include <vector>

class bMsg {
    public:

    std::vector<double> slopes    ; //<  slopes of the gamma function as an array 
    std::vector<double> gamma0    ; //<  gamma0 
    double              r         ; //< robustness variable
    double              slack     ; //< slack violation for the constructed barrier (positive number : if it is not zero measn that the barrier can not be fully followed with the current maximum control input)
    std::vector<double> b_vector  ; //< b vector of the hyperrectangle of the task (Ax <= b) where b = b' + Ac with c: center of the cube and b : uniform vector size_/2 with size being the size of the hypercibe 
    std::vector<double> time_grid ; //< time grid over which the barrier function should be evaluated
    int                 task_id   ; //< task id (just for reference)
    int                 edge_i    ; //< Task is assigned on the edge e_{ij} = x_i - x_j
    int                 edge_j    ; //< Task is assigned on the edge e_{ij} = x_i - x_j
};

#endif // BMSG_H