#ifndef BARRIER_FUNCTION_H
#define BARRIER_FUNCTION_H

#include <vector>
#include <fstream>
#include <filesystem>

#include "hypercube.h"
#include "utils.h"
#include "task.h"
#include "fusion.h"
#include "nlohmann/json.hpp"

using namespace mosek::fusion;
using namespace monty;
using json = nlohmann::json;
namespace fs = std::filesystem;


template <int Dim>
class BarrierFunction {
    public:

    /**
     * @brief Simple barrier function constructor for a given task. The barrier function is more an holder of the different variables that are related 
     * to a given barrier to be designed.
     * 
     * @param task The task for which the barrier function is designed.
     * @param time_grid Vector of time points at which the barrier function is evaluated and it is active.
     * @param optimizer The optimizer model that will be used to optimize the barrier function (together with the other ones).
     */
    
    
    BarrierFunction(const Task<Dim>&                task, 
                    const std::vector<double>&      time_grid, 
                    Model::t                        optimizer ):
        
                    task(task), time_grid(time_grid), optimizer(optimizer.get()) {

            // Define delta times by taking the difference between consecutive time points.
            time_delta.resize(time_grid.size() - 1);
            for (size_t i = 0; i < time_grid.size() - 1; ++i) {
                time_delta[i] = time_grid[i + 1] - time_grid[i];
            }
            
            // defined slopes variables 
            if constexpr (Dim == 2) {
                num_hyperplanes = 4; 
            } else if constexpr (Dim == 3) {
                num_hyperplanes = 6; 
            } else {
                throw std::invalid_argument("Dim must be either 2 or 3.");
            }
            

            // define variables
            slopes_var = optimizer->variable(new_array_ptr<int,1>({num_hyperplanes, time_delta.size()}), Domain::lessThan(0.0));
            gamma0_var = optimizer->variable(new_array_ptr<int,1>({num_hyperplanes,1})                 , Domain::greaterThan(0.0));
            r_var      = optimizer->variable(Domain::greaterThan(0.0));
            slack_var =  optimizer->variable(Domain::greaterThan(0.0)); // slack variable for the max input constraint

            if (task.type == TaskType::ALWAYS) {
                flat_time = task.start_time;
            } else if (task.type == TaskType::EVENTUALLY) {
                flat_time = task.tau;
            } else {
                throw std::invalid_argument("Task type must be either ALWAYS or EVENTUALLY.");
            }


        };

    Variable::t e_vector(double t) {
        // extract the column vector corresponding to the time time t

        if (t < time_grid.front() || t > time_grid.back()) {
            throw std::out_of_range("Time t is out of the bounds of the time grid.");
        }
        else if (t == time_grid.front()) {
            return slopes_var->slice( new_array_ptr<int,1>({0,0}) , new_array_ptr<int,1>({num_hyperplanes,1})); // take the last column
        } 

        else {
            auto it   = std::lower_bound(time_grid.begin(), time_grid.end(), t);
            int index = std::distance(time_grid.begin(), it) - 1; // Get the index of the column corresponding to time    time_grid[index] < t <= time_grid[index + 1]
             
            // print the index
            std::cout << "Index of time " << t << " is " << index << std::endl;
            return slopes_var->slice( new_array_ptr<int,1>({0,index}) , new_array_ptr<int,1>({num_hyperplanes,index+1}) ); // take the column at index
        }
    };

    Expression::t g_vector(double t) {
        if (t < time_grid.front() || t > time_grid.back()) {
            throw std::out_of_range("The time t is not in the time grid. given time grid: " + std::to_string(time_grid.front()) + " to " + std::to_string(time_grid.back()) + ", but t is " + std::to_string(t));
        }
        
        std::vector<double> tt = std::vector<double>(time_delta.size(), 0.0);
        

        if (t == time_grid.front()) {
            // Fill t_vector with time_delta and last entry = -time_grid[N-1]
            
            tt[0] = -time_grid[0];
        } else {
            // Find the interval [time_grid[index], time_grid[index+1]) containing t
            auto it   = std::lower_bound(time_grid.begin(), time_grid.end(), t);
            int index = std::distance(time_grid.begin(), it) - 1;

            for (int j = 0; j < index; ++j) {
                tt[j] = time_delta[j];
            }
            tt[index] = -time_grid[index];
        }

        // Compute: gamma0_var + slopes_var * t_vector
        Matrix::t t_vector = Matrix::dense(time_delta.size(),1, new_array_ptr<double>(tt));

        // // print shape of slopes_var and t vector
        // std::cout << "Shape of slopes_var: " << (*slopes_var->getShape())[0] << ", " << (*slopes_var->getShape())[1] << std::endl;

        // std::cout << "Shape of t_vector: " << (t_vector->numRows()) << ", " << (t_vector->numColumns()) << std::endl;


        auto slopes_times_t = Expr::mul(slopes_var, t_vector);
        
        
        // //print shape
        // std::cout << "Shape of slopes_times_t: " << (*slopes_times_t->getShape())[0] << ", " << (*slopes_times_t->getShape())[1] << std::endl;

        auto g              = Expr::add(gamma0_var, slopes_times_t);
        return g;
    };

    Expression::t gamma_vector(double t) {

         if (t < time_grid.front() || t > time_grid.back()) {
            throw std::out_of_range("The time t is not in the time grid. given time grid: " + std::to_string(time_grid.front()) + " to " + std::to_string(time_grid.back()) + ", but t is " + std::to_string(t));
        }
        
        std::vector<double> tt = std::vector<double>(time_delta.size(), 0.0);
        

        if (t == time_grid.front()) {
            // Fill t_vector with time_delta and last entry = -time_grid[N-1]
            
            tt[0] = -time_grid[0];
        } else {
            // Find the interval [time_grid[index], time_grid[index+1]) containing t
            auto it   = std::lower_bound(time_grid.begin(), time_grid.end(), t);
            int index = std::distance(time_grid.begin(), it) - 1;

            for (int j = 0; j < index; ++j) {
                tt[j] = time_delta[j];
            }
            tt[index] = t-time_grid[index];
        }

        // Compute: gamma0_var + slopes_var * t_vector
        Matrix::t t_vector = Matrix::dense(time_delta.size(),1, new_array_ptr<double>(tt));

        // // print shape of slopes_var and t vector
        // std::cout << "Shape of slopes_var: " << (*slopes_var->getShape())[0] << ", " << (*slopes_var->getShape())[1] << std::endl;

        // std::cout << "Shape of t_vector: " << (t_vector->numRows()) << ", " << (t_vector->numColumns()) << std::endl;


        auto slopes_times_t = Expr::mul(slopes_var, t_vector);
        
        return Expr::add(slopes_times_t,gamma0_var); // gamma = slopes * t + gamma0

    };

    Matrix::t get_D_matrix() {

        return task.D;
    };

    Matrix::t get_c_vector() const {

        return Matrix::dense(task.hypercube.b->size(), 1, task.hypercube.b);    
    };


    void set_gamma_flat_constraint(){

        if (task.type == TaskType::ALWAYS) {
            // For ALWAYS tasks, the gamma function is flat after the start time
            auto gamma_flat = gamma_vector(flat_time);
            auto ones = Matrix::ones((*gamma_flat->getShape())[0], 1);
            this->optimizer->constraint(Expr::add(gamma_flat,  Expr::mul(ones,r_var)), Domain::equalsTo(0.)); // gamma == -r_var

        } else if (task.type == TaskType::EVENTUALLY) {
            // For EVENTUALLY tasks, the gamma function is flat after the tau time
            // Find first index j where time_grid[j] >= flat_time
           
            auto it = std::lower_bound(time_grid.begin(), time_grid.end(), flat_time);
            int flat_start_index = std::distance(time_grid.begin(), it);


            auto flat_slopes = slopes_var->slice(
            new_array_ptr<int, 1>({0              , flat_start_index}),
            new_array_ptr<int, 1>({num_hyperplanes, static_cast<int>(time_delta.size())}));
            
            this->optimizer->constraint(flat_slopes, Domain::equalsTo(0.0));    

            auto gamma_flat = gamma_vector(time_grid.back());
            auto ones = Matrix::ones((*gamma_flat->getShape())[0], 1);

            // print shape of gamma flat
            std::cout << "Shape of gamma_flat: " << (*gamma_flat->getShape())[0] << ", " << (*gamma_flat->getShape())[1] << std::endl;


            this->optimizer->constraint(Expr::add(gamma_flat,Expr::mul(ones,r_var)), Domain::equalsTo(0.0)); // gamma == -r_var

        }
        
    }

    void set_max_input_constraint(double max_input) {
        Matrix::t ones = Matrix::ones((*slopes_var->getShape())[0],(*slopes_var->getShape())[1]);
        
        this->optimizer->constraint( Expr::add(Expr::neg(slopes_var), Expr::neg(Expr::mul(ones,slack_var))), Domain::lessThan(max_input)); // slopes <= max_input
    }

    
    void save_json(std::string folder_name = "my_folder") {

        if (fs::create_directory(folder_name)) {
        std::cout << "Created folder: " << folder_name << "\n";
        } else {
            std::cerr << "Failed to create folder: " << folder_name << "\n";
        }

        
        std::string filename =  folder_name + "/barrier_" + std::to_string(task.task_id) + ".json";
       

        auto slopes_val = slopes_var->level();
        auto gamma0_val = gamma0_var->level();
        auto r_val      = r_var->level();
        auto slack_val  = slack_var->level(); 
    
        json j_array {{"Slopes"   ,new_vector_from_array_ptr(slopes_val)        },
                      {"Gamma0"   ,new_vector_from_array_ptr(gamma0_val)        },
                      {"R"        ,new_vector_from_array_ptr(r_val     )        },
                      {"Slack"    ,new_vector_from_array_ptr(slack_val )        },
                      {"b_vector" ,new_vector_from_array_ptr(task.hypercube.b)  },
                      {"TimeGrid" ,time_grid                                    },
                      {"TaskID"   , task.task_id                                },
                      {"edge_i"   , task.agent_i                                },
                      {"edge_j"   , task.agent_j                                }};
        
        std::ofstream out(filename);
        out <<  j_array.dump(4) << std::endl;
        // std::cout << std::setw(4) << j_array << std::endl;
    }

    
    // ====== Attributes ======


    const std::vector<double> time_grid;  // Time grid over which the barrier function is defined
    Task<Dim>                 task;       // task of the given barrier.
    double                    flat_time;  // Time after which the barrier function is flat
    std::vector<double>       time_delta;
    int                       num_hyperplanes;
    

    // mosek variables
    Variable::t         slopes_var;
    Variable::t         gamma0_var;
    Variable::t         r_var;
    Variable::t         slack_var; // slack variable for the max input constraint
    Model*              optimizer;  // non-owning raw pointer to optimzer. optimzer will be deleted by the TaskScheduler
    Matrix::t           D;

};

#endif