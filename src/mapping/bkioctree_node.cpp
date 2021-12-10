#include <algorithm>
#include <assert.h>
#include <math.h>
#include <iostream>

#include "bkioctree_node.h"

namespace semantic_bki {

    /// Default static values
    int Semantics::num_class = 2;
    float Semantics::prior = 0.5f;
    float Semantics::var_thresh = 1000.0f;
    float Semantics::free_thresh = 0.3f;
    float Semantics::occupied_thresh = 0.7f;
    float Semantics::gamma = 0.75;
    KernelParams Semantics::kp = {1.0, 1.0, 0.2, 0.2, 0.05} ;


    void Semantics::get_probs(std::vector<float>& probs) const {
      assert (probs.size() == num_class);
      float sum = 0;
      for (const auto &m : ms)
        sum += m;
      for (int i = 0; i < num_class; ++i)
        probs[i] = ms[i] / sum;
    }

    void Semantics::get_vars(std::vector<float>& vars) const {
      assert (vars.size() == num_class);
      float sum = 0;
      for (const auto &m : ms)
        sum += m;
      for (int i = 0; i < num_class; ++i)
        vars[i] = ((ms[i] / sum) - (ms[i] / sum) * (ms[i] / sum)) / (sum + 1);
    }

    void Semantics::update(const std::vector<float>& ybars,
                  const std::vector<float> &vbars, const std::vector<float> &pbars, 
                  bool spatiotemporal, bool free_sample) {
      //assert(pbars.size() == num_class && vbars.size() == num_class);
      classified = true;
      float removed_alpha = 0;
      if (!free_sample)
        flow[0] = 0;

      for (int i = 0; i < num_class; ++i){
        if (spatiotemporal){
            removed_alpha = exp(-flow[i] * flow[i]) * ms[i]; //decay prior (prediction step)
            //ms[i] = removed_alpha + ybars[i]; //add new observations (update step)
            ms[i] = removed_alpha + ybars[i] + prop[i];
            flow[i] = gamma * vbars[i] + (1 - gamma) * flow[i]; //update flow for the next time step
            prop[i] = pbars[i];
        }
        else
          ms[i] += ybars[i]; // static mapping: so just add new observations
      }

      std::vector<float> probs(num_class);
      get_probs(probs);

      semantics = std::distance(probs.begin(), std::max_element(probs.begin(), probs.end()));

      if (semantics == 0)
        state = State::FREE;
      else
        state = State::OCCUPIED;
    }
    


    void Semantics::update(const std::vector<float>& ybars,
                  const std::vector<float> &vbars,
                  const ScanStep time_diff) {
      assert(ybars.size() == num_class && vbars.size() == num_class);
      classified = true;

      
      for (int i = 0; i < num_class; ++i){
        //original
        //ms[i] += ybars[i];
        // // //current
        // flow[i] = vbars[i];
        // ms[i] =  exp( -flow[i] * flow[i]) /// (1 - probs[i])) 
        //     * ms[i] +  ybars[i];
        // // // //before
        ms[i] =  exp( -flow[i] * flow[i] * time_diff ) // / (1 - probs[i]))
                    * ms[i] +  ybars[i];
        flow[i] = vbars[i]; 
        //std::cout << exp( -flow[i] * flow[i]) << std::endl;
      }

      std::vector<float> probs(num_class);
      get_probs(probs);

      semantics = std::distance(probs.begin(), std::max_element(probs.begin(), probs.end()));

      if (semantics == 0)
        state = State::FREE;
      else
        state = State::OCCUPIED;
    }
}
