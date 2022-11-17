//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//

#pragma once

#include <fstream> 
#include <filesystem>
// Util
double cubic(double time,    ///< Current time
            double time_0,  ///< Start time
            double time_f,  ///< End time
            double x_0,     ///< Start state
            double x_f,     ///< End state
            double x_dot_0, ///< Start state dot
            double x_dot_f  ///< End state dot
)
{
  double x_t;

  if (time < time_0)
  {
    x_t = x_0;
  }
  else if (time > time_f)
  {
    x_t = x_f;
  }
  else
  {
    double elapsed_time = time - time_0;
    double total_time = time_f - time_0;
    double total_time2 = total_time * total_time;  // pow(t,2)
    double total_time3 = total_time2 * total_time; // pow(t,3)
    double total_x = x_f - x_0;

    x_t = x_0 + x_dot_0 * elapsed_time

          + (3 * total_x / total_time2 - 2 * x_dot_0 / total_time - x_dot_f / total_time) * elapsed_time * elapsed_time

          + (-2 * total_x / total_time3 +
              (x_dot_0 + x_dot_f) / total_time2) *
                elapsed_time * elapsed_time * elapsed_time;
  }

  return x_t;
}

void readTextFile(std::string file_path, Eigen::MatrixXd &output_matrix)
{
    Eigen::MatrixXd matrixData;

    std::ifstream file;
    file.open(file_path, std::ios::in);

    if(!file.is_open())
    {
        std::cout<<"Can not find the text file"<<std::endl;
    }

    float temp;
    int row = 0;
    int col = 0;

    while(!file.eof())
    {
        file >> temp;
        if(temp != '\n')
        {
            output_matrix(row, col) = temp;
            col ++;
            if (col == output_matrix.cols())
            {
                col = 0;
                row ++;
            }
        }
    }
}