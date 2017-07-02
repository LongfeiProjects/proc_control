/**
 * \file	filter.h
 * \author Francis Masse <francis.masse05@gmail.com>
 * \date	02/07/17
 *
 * \copyright Copyright (c) 2017 S.O.N.I.A. AUV All rights reserved.
 *
 * \section LICENSE
 *
 * This file is part of S.O.N.I.A. software.
 *
 * S.O.N.I.A. AUV software is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S.O.N.I.A. AUV software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S.O.N.I.A. AUV software. If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef PROC_CONTROL_FILTER_H
#define PROC_CONTROL_FILTER_H

#include <math.h>
#include <cstdlib>

class Filter {
 public:
  //==========================================================================
  // C O N S T  ,  T Y P E D E F   A N D   E N U M

  enum FilterType {LowPassFilter, HighPassFilter};

  //==========================================================================
  // P U B L I C   C / D T O R S

  Filter(FilterType filter_type, int number_of_taps, double fs, double fx);
  ~Filter();

  //==========================================================================
  // P U B L I C   M E T H O D S

  void Init();
  double Sample(double data_sample);
  int GetErrorFlag(){return error_flag;};
  void GetTaps( double *taps );

 private:
  //==========================================================================
  // P R I V A T E   M E T H O D S

  void DesignLowPassFilter();
  void DesignHighPassFilter();

  //==========================================================================
  // P R I V A T E   M E M B E R S

  FilterType type;
  int number_of_taps;
  int error_flag;
  double Fs;
  double Fx;
  double lambda;
  double *taps;
  double *sr;
};

#endif //PROC_CONTROL_FILTER_H