#pragma once

#include <vector>

namespace semantic_bki {

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
        Semantics() : ms(std::vector<float>(num_class, prior)), state(State::UNKNOWN) { classified = false; }

        Semantics(const Semantics &other) : ms(other.ms), state(other.state), semantics(other.semantics) { }

        Semantics &operator=(const Semantics &other) {
          ms = other.ms;
          state = other.state;
          semantics = other.semantics;
          return *this;
        }

        ~Semantics() { }

        /*
         * @brief Exact updates for nonparametric Bayesian kernel inference
         * @param ybar kernel density estimate of positive class (occupied)
         * @param kbar kernel density of negative class (unoccupied)
         */
        void update(const std::vector<float>& ybars);

        /// Get probability of occupancy.
        void get_probs(std::vector<float>& probs) const;

        /// Get variance of occupancy (uncertainty)
	      void get_vars(std::vector<float>& vars) const;
        
        /*
         * @brief Get occupancy state of the node.
         * @return occupancy state (see State).
         */
        inline State get_state() const { return state; }

        inline int get_semantics() const { return semantics; }

        bool classified;

        /*
        * @brief Add x number of observations per semantic category
        */
       void pred_post_update(const ScanStep update_number);

    private:
        std::vector<float> ms; // concentration parameters
        State state; // FREE, OCCUPIED OR UNKNOWN
        int semantics; // 0 if free
        static int num_class;      // number of classes
        
        static float sf2;
        static float ell;   // length-scale
        static float prior;  // prior on each class

        static float var_thresh;
        static float free_thresh;     // FREE occupancy threshold
        static float occupied_thresh; // OCCUPIED occupancy threshold
    };

    typedef Semantics SemanticOcTreeNode;
}
