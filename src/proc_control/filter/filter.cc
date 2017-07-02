/**
 * \file	filter.cc
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

#include "filter.h"

#define ECODE(x) {error_flag = x; return;}

//==============================================================================
// C / D T O R S   S E C T I O N

//------------------------------------------------------------------------------
//
Filter::Filter(FilterType filter_type, int number_of_taps, double fs, double fx)
{
error_flag = 0;
type = filter_type;
this->number_of_taps = number_of_taps;
fs = Fs;
fx = Fx;
lambda = M_PI * fx / (fs/2);

if( fs <= 0 ) ECODE(-1);
if( fx <= 0 || fx >= fs/2 ) ECODE(-2);
if( this->number_of_taps <= 0 || this->number_of_taps > 100 ) ECODE(-3);

taps = sr = nullptr;
taps = (double*)malloc( this->number_of_taps * sizeof(double) );
sr = (double*)malloc( this->number_of_taps * sizeof(double) );
if( taps == nullptr || sr == nullptr ) ECODE(-4);

Init();

if( type == LowPassFilter ) DesignLowPassFilter();
else if( type == HighPassFilter ) DesignHighPassFilter();
else ECODE(-5);

return;
}

//------------------------------------------------------------------------------
//
Filter::~Filter() {
  if( taps != nullptr ) free( taps );
  if( sr != nullptr ) free( sr );
}

//==============================================================================
// M E T H O D   S E C T I O N

//-----------------------------------------------------------------------------
//
void Filter::DesignLowPassFilter() {
  int n;
  double mm;

  for(n = 0; n < number_of_taps; n++) {
    mm = n - (number_of_taps - 1.0) / 2.0;
    if( mm == 0.0 ) {
      taps[n] = lambda / M_PI;
    } else {
      taps[n] = sin( mm * lambda ) / (mm * M_PI);
    }
  }

  return;
}

//-----------------------------------------------------------------------------
//
void Filter::DesignHighPassFilter() {
  int n;
  double mm;

  for(n = 0; n < number_of_taps; n++){
    mm = n - (number_of_taps - 1.0) / 2.0;
    if( mm == 0.0 ) {
      taps[n] = 1.0 - lambda / M_PI;
    } else {
      taps[n] = -sin( mm * lambda ) / (mm * M_PI);
    }
  }

  return;
}

//-----------------------------------------------------------------------------
//
void Filter::GetTaps( double *taps )
{
  int i;

  if( error_flag != 0 ) return;

  for(i = 0; i < number_of_taps; i++) taps[i] = this->taps[i];

  return;
}

//-----------------------------------------------------------------------------
//
void Filter::Init()
{
  int i;

  if( error_flag != 0 ) return;

  for(i = 0; i < number_of_taps; i++) sr[i] = 0;

  return;
}

//-----------------------------------------------------------------------------
//
double Filter::Sample(double data_sample)
{
  int i;
  double result;

  if( error_flag != 0 ) {
    return(0);
  }

  for(i = number_of_taps - 1; i >= 1; i--){
    sr[i] = sr[i-1];
  }

  sr[0] = data_sample;

  result = 0;
  for(i = 0; i < number_of_taps; i++) {
    result += sr[i] * taps[i];
  }

  return result;
}

