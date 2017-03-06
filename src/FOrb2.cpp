/**
 * File: FOrb2.cpp
 * Date: Feburary 2013
 * Author: Lionel Heng
 * Description: functions for ORB descriptors
 *
 * This file is licensed under a Creative Commons 
 * Attribution-NonCommercial-ShareAlike 3.0 license. 
 * This file can be freely used and users can use, download and edit this file 
 * provided that credit is attributed to the original author. No users are 
 * permitted to use this file for commercial purposes unless explicit permission
 * is given by the original author. Derivative works must be licensed using the
 * same or similar license.
 * Check http://creativecommons.org/licenses/by-nc-sa/3.0/ to obtain further
 * details.
 *
 */
 
#include <vector>
#include <string>
#include <sstream>

#include "DVision.h"
#include "FOrb2.h"

using namespace std;

namespace DBoW2 {

// --------------------------------------------------------------------------

void FOrb2::meanValue(const std::vector<FOrb2::pDescriptor> &descriptors,
  FOrb2::TDescriptor &mean)
{
  mean.reset();
  
  if(descriptors.empty()) return;
  
  const int N2 = descriptors.size() / 2;
  const int L = descriptors[0]->size();
  
  vector<int> counters(L, 0);

  vector<FOrb2::pDescriptor>::const_iterator it;
  for(it = descriptors.begin(); it != descriptors.end(); ++it)
  {
    const FOrb2::TDescriptor &desc = **it;
    for(int i = 0; i < L; ++i)
    {
      if(desc[i]) counters[i]++;
    }
  }
  
  for(int i = 0; i < L; ++i)
  {
    if(counters[i] > N2) mean.set(i);
  }
  
}

// --------------------------------------------------------------------------
  
double FOrb2::distance(const FOrb2::TDescriptor &a,
  const FOrb2::TDescriptor &b)
{
  return (double)DVision::BRIEF::distance(a, b);
}

// --------------------------------------------------------------------------
  
std::string FOrb2::toString(const FOrb2::TDescriptor &a)
{
  // from boost::bitset
  string s;
  to_string(a, s); // reversed
  return s;
}

// --------------------------------------------------------------------------
  
void FOrb2::fromString(FOrb2::TDescriptor &a, const std::string &s)
{
  // from boost::bitset
  stringstream ss(s);
  ss >> a;
}

// --------------------------------------------------------------------------

void FOrb2::toMat32F(const std::vector<TDescriptor> &descriptors,
  cv::Mat &mat)
{
  if(descriptors.empty())
  {
    mat.release();
    return;
  }
  
  const int N = descriptors.size();
  const int L = descriptors[0].size();
  
  mat.create(N, L, CV_32F);
  
  for(int i = 0; i < N; ++i)
  {
    const TDescriptor& desc = descriptors[i];
    float *p = mat.ptr<float>(i);
    for(int j = 0; j < L; ++j, ++p)
    {
      *p = (desc[j] ? 1 : 0);
    }
  } 
}

// --------------------------------------------------------------------------

} // namespace DBoW2

