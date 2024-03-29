#pragma once

#include <vector>
namespace semantic_bki {


    //Kernel parameters helper struct
    struct KernelParams{

        double sf2;
        double ell;

        float flow_sf2;
        float flow_ell;

        float m_resol; 

    };

    /// Occupancy state: before pruning: FREE, OCCUPIED, UNKNOWN; after pruning: PRUNED
    enum class State : char {
        FREE, OCCUPIED, UNKNOWN, PRUNED
    };

    ///    // Discrete scan step when octreenode in Block is initialized
    typedef u_int64_t ScanStep;

    /*
     * @brief Inference ouputs and occupancy state.
     *
     * Occupancy has member variables: m_A and m_B (kernel densities of positive
     * and negative class, respectively) and State.
     * Before using this class, set the static member variables first.
     */
    class Semantics {
      
      friend class SemanticBKIOctoMap;

    public:
        /*
         * @brief Constructors and destructor.
         */
        Semantics() : ms(std::vector<float>(num_class, prior)), state(State::UNKNOWN), decayed(false),
                        flow(std::vector<float>(num_class, 0.0f)), prop(std::vector<float>(num_class, 0.0f))
        {   
            ms[0] = 0.1; //set free space prior to 0.1
        }

        Semantics(const Semantics &other) : ms(other.ms), state(other.state), decayed(other.decayed),
                            semantics(other.semantics), flow(other.flow), prop(other.prop) { }

        Semantics &operator=(const Semantics &other) {
          ms = other.ms;
          state = other.state;
          semantics = other.semantics;
          flow = other.flow;
          prop = other.prop;
          return *this;
        }

        ~Semantics() { }

        /*
         * @brief Exact updates for nonparametric Bayesian kernel inference
         * @param ybar kernel density estimate of positive class (occupied)
         * @param vbar kernel density estimate of per-class velocity
         * @param spatiotemporal bool about whether we are doing static (false) 
         *          or dynamic (true) mapping
         * @param free_sample bool to indicate whether the point cloud includes
         *          free space samples or does not.
         */
        void update(const std::vector<float>& ybars,
              const std::vector<float>& vbars, 
              const std::vector<float>& pbars,
              bool spatiotemporal,
              bool free_sample);

        /*
        * @brief A function to decay the cell with velocity stored in the previous
        *        time step
        */

        void decay(bool spatiotemporal, bool free_sample);

        /// Get probability of occupancy.
        void get_probs(std::vector<float>& probs) const;

        /// Get variance of occupancy (uncertainty)
	      void get_vars(std::vector<float>& vars) const;
        
        /*
         * @brief Get occupancy state of the node.
         * @return occupancy state (see State).
         */
        inline State get_state() const { return state; }

        inline float get_flow(const int class_id) const{
            return flow[class_id];
        }

        inline int get_semantics() const { return semantics; }


    private:
        std::vector<float> ms; // concentration parameters
        State state; // FREE, OCCUPIED OR UNKNOWN
        int semantics; // 0 if free
        std::vector<float> flow; // flow recorded for each class
        std::vector<float> prop; //propagated points from previous step
        static int num_class;      // number of classes

        static float prior;  // prior on each class

        static KernelParams kp; //kernel params - sf2, ell, flow_sf2, flow_ell

        static float var_thresh;
        static float free_thresh;     // FREE occupancy threshold
        static float occupied_thresh; // OCCUPIED occupancy threshold

        static float gamma; //parameter for how flow is smoothed with M.A.
        bool decayed;

    };

    typedef Semantics SemanticOcTreeNode;
}
