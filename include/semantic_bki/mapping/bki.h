#pragma once

#include "bkioctomap.h"
#define PI 3.1415926f
namespace semantic_bki {

	/*
     * @brief Bayesian Generalized Kernel Inference on Bernoulli distribution
     * @param dim dimension of data (2, 3, etc.)
     * @param T data type (float, double, etc.)
     * @ref Nonparametric Bayesian inference on multivariate exponential families
     */
    template<int dim, typename T>
    class SemanticBKInference {
    public:
        /// Eigen matrix type for training and test data and kernel
        using MatrixXType = Eigen::Matrix<T, -1, dim, Eigen::RowMajor>;
        using MatrixKType = Eigen::Matrix<T, -1, -1, Eigen::RowMajor>;
        using MatrixDKType = Eigen::Matrix<T, -1, 1>;
        using MatrixYType = Eigen::Matrix<T, -1, 1>;

        SemanticBKInference(int nc, bool temporal_in, KernelParams &params, std::vector<bool> dynamic_in) : 
                            nc(nc), sf2(params.sf2), 
                            ell(params.ell), flow_ell(params.flow_ell),
                            flow_sf2(params.flow_sf2), m_resol(params.m_resol), temporal(temporal_in),
                            dynamic(dynamic_in), trained(false) { }

        /*
         * @brief Fit BGK Model
         * @param x input vector (3N, row major)
         * @param y target vector (N)
         */
        void train(const std::vector<T> &x, const std::vector<T> &y) {
            assert(x.size() % dim == 0 && (int) (x.size() / dim) == y.size());
            MatrixXType _x = Eigen::Map<const MatrixXType>(x.data(), x.size() / dim, dim);
            MatrixYType _y = Eigen::Map<const MatrixYType>(y.data(), y.size(), 1);
            this->y_vec = y;
            train(_x, _y);
        }

        /*
         * @brief Fit BGK Model
         * @param x input matrix (NX3)
         * @param y target matrix (NX1)
         */
        void train(const MatrixXType &x, const MatrixYType &y) {
            this->x = MatrixXType(x);
            this->y = MatrixYType(y);
            trained = true;
        }
        /*
        * \brief A function to store flow for each training point
        *  @param v: flow matrix N x 3)
        */

        void store_flow(const std::vector<T> &v){
            this->v = Eigen::Map<const MatrixXType> (v.data(), v.size() / dim, dim);
            //stored as Nx3 matrix
        }
       
      void predict(const std::vector<T> &xs, std::vector<std::vector<T>> &ybars,
                    std::vector<std::vector<T>> &vbars) {
          assert(xs.size() % dim == 0);
          MatrixXType _xs = Eigen::Map<const MatrixXType>(xs.data(), xs.size() / dim, dim);
          assert(trained == true);
          MatrixKType Ks, Kv, Ki;
          covSparse(_xs, x, Ks);
          
          if (temporal){
           //Shwarya : TODO you can do the same and compute a velocity covariance
            // here and simply multiply the velocities
            // voxel centroids vs training points
            covMaterniso3(_xs, x, Kv);
	          if (dynamic[0])
	            covSparse(_xs, x, Ki, 2*m_resol, 1000); // Apply sparse kernel to free space
            //covCountingSensorModel(_xs, x, Ki);
            //covGaussian(_xs, x, Kv);
          }
          vbars.resize(_xs.rows());

          ybars.resize(_xs.rows());

          for (int r = 0; r < _xs.rows(); ++r){
            ybars[r].resize(nc);
            vbars[r].resize(nc);
          }

          MatrixYType _y_vec = Eigen::Map<const MatrixYType>(y_vec.data(), y_vec.size(), 1);
          MatrixXType _v_vec = Eigen::Map<const MatrixXType>(v.data(), v.size() / dim, dim);
          /***
           * This block of code essentially copies over the velocities of each training point
           * to _v_vec
           * ***/

          
          for (int k = 0; k < nc; ++k) { //iterate through all classes
              for (int i = 0; i < y_vec.size(); ++i) { //iterate through all the labels 
              //std::cout << v.row(i).norm() << '\n';
                if (y_vec[i] == k){ //if the label class matches a valid class
                  _y_vec(i, 0) = 1; 

                  if (temporal)
                    _v_vec(i, 0) = v.row(i).norm(); //store the norm of velocity for that label
                }
                else{
                  _y_vec(i, 0) = 0; 
                  if (temporal){
                    if (k == 0 && dynamic[0] && dynamic[y_vec[i]]) //if free space is treated as moving
                      _v_vec(i, 0) = v.row(i).norm(); //save the norm of velocity for whichever label here
                      //a training point can only have one label
                    else
                      _v_vec.row(i).setZero();
                  }
                }
              }
              /***
               * This block of code will take the velocities saved from the training points
               * and use a Kernel to calculate the velocities for the test points
               * For non-free space semantic classes, we're using Kv
               * For free-space, we're using a sparse kernel Ki
               ***/
            
              MatrixYType _ybar;
              MatrixXType _vbar;

              _ybar = (Ks * _y_vec);
              // if (temporal)
              //   _vbar = (Kv * _v_vec);
              if (temporal && k != 0){
               _vbar = (Kv * _v_vec);

              }
              else if (temporal && dynamic[k] &&  k == 0)
                _vbar =  (Ki * _v_vec) ;
              
              /***
               * In this block of code, the velocities for query points have already been
               * computed, but we want to store it in a 2D matrix that has a component 
               * for each class as well.
               * The line vbars[r][k] is going to take the computed velocity for one query point
               * and add it on to the kth class for that query point.
               ***/ 
            
              for (int r = 0; r < _ybar.rows(); ++r){
                ybars[r][k] = _ybar(r, 0);
                if (temporal){
                  if (dynamic[k]){
                    vbars[r][k] = _vbar(r, 0) / _y_vec.size();
                    // std::cout << k << std::endl;
                  }
                   else
                     vbars[r][k] = vbars[r][0]; //Store the dynamic velocity observed to deplete static and free classes
                // if (k==2)
                //   std::cout << vbars[r][k] << '\n';
                  // if (vbars[r][k] > 0.5 && k != 0)
                  // std::cout << "Velocity is:" << vbars[r][k] << std::endl;
                }

              }

              
          }
      }

      void predict_csm(const std::vector<T> &xs, std::vector<std::vector<T>> &ybars,
                        std::vector<std::vector<T>> &vbars) {
          assert(xs.size() % dim == 0);
          MatrixXType _xs = Eigen::Map<const MatrixXType>(xs.data(), xs.size() / dim, dim);
          assert(trained == true);
          MatrixKType Ks;

          covCountingSensorModel(_xs, x, Ks); // gives a (No. of Voxels x Training Points)
          vbars.resize(_xs.rows());
          ybars.resize(_xs.rows());

          for (int r = 0; r < _xs.rows(); ++r){
            ybars[r].resize(nc);
            vbars[r].resize(nc);
          }

            MatrixYType _y_vec = Eigen::Map<const MatrixYType>(y_vec.data(), y_vec.size(), 1);
            MatrixYType _v_vec = Eigen::Map<const MatrixYType>(v.data(), v.size(), 1);

            for (int k = 0; k < nc; ++k) {
              for (int i = 0; i < y_vec.size(); ++i) {
                if (y_vec[i] == k){
                  _y_vec(i, 0) = 1;
                  _v_vec(i, 0) = v(i, 0); //what is this supposed to mean?!?!
                }
                else{
                  _y_vec(i, 0) = 0;
                  _v_vec(i, 0) = 0;
                }
              }
            
            MatrixYType _ybar, _vbar;
            _ybar = (Ks * _y_vec);
            _vbar = (Ks * _v_vec);
            
            for (int r = 0; r < _ybar.rows(); ++r){
              ybars[r][k] = _ybar(r, 0);
              vbars[r][k] = _vbar(r, 0);
            }
          }
      }

        
    private:
        /*
         * @brief Compute Euclid distances between two vectors.
         * @param x input vector
         * @param z input vecotr
         * @return d distance matrix
         */
        void dist(const MatrixXType &x, const MatrixXType &z, MatrixKType &d) const {
            d = MatrixKType::Zero(x.rows(), z.rows());
            for (int i = 0; i < x.rows(); ++i) {
                d.row(i) = (z.rowwise() - x.row(i)).rowwise().norm();
            }
        }

        /*
         * @brief Matern3 kernel.
         * @param x input vector
         * @param z input vector
         * @return Kxz covariance matrix
         */
        void covMaterniso3(const MatrixXType &x, const MatrixXType &z, MatrixKType &Kxz) const {
            dist(1.73205 / flow_ell * x, 1.73205 / flow_ell * z, Kxz);
            Kxz = ((1 + Kxz.array()) * exp(-Kxz.array())).matrix() * flow_sf2;
        }

        /*
         * @brief Custom Matern3 kernel.
         * @param x input vector
         * @param z input vector
         * @return Kxz covariance matrix
         */
        void covMaterniso3(const MatrixXType &x, const MatrixXType &z, MatrixKType &Kxz,
                           const float ell_in, const float sf2_in) const {
            dist(1.73205 / ell_in * x, 1.73205 / ell_in * z, Kxz);
            Kxz = ((1 + Kxz.array()) * exp(-Kxz.array())).matrix() * sf2_in;
        }

        /*
         * \brief Gaussian kernel.
         * @param x input vector
         * @param z input vector
         * @return Kxz covariance matrix
         */
        void covGaussian(const MatrixXType &x, const MatrixXType &z, MatrixKType &Kxz) const {
            dist(x, z, Kxz);
            Kxz = exp((1 / (sf2*sf2)) * -Kxz.array().pow(2)).matrix();
        }
        

        /*
         * @brief Custom Sparse kernel.
         * @param x input vector
         * @param z input vector
         * @param ell_in kernel length_scale
         * @param sf2_in kernel scale
         * @return Kxz covariance matrix
         * @ref A sparse covariance function for exact gaussian process inference in large datasets.
         */
        void covSparse(const MatrixXType &x, const MatrixXType &z, MatrixKType &Kxz,
                       const float ell_in, const float sf2_in) const {
            dist(x / ell_in, z / ell_in, Kxz);
            Kxz = (((2.0f + (Kxz * 2.0f * 3.1415926f).array().cos()) * (1.0f - Kxz.array()) / 3.0f) +
                  (Kxz * 2.0f * 3.1415926f).array().sin() / (2.0f * 3.1415926f)).matrix() * sf2_in;

            // Clean up for values with distance outside length scale
            // Possible because Kxz <= 0 when dist >= ell
            for (int i = 0; i < Kxz.rows(); ++i)
            {
                for (int j = 0; j < Kxz.cols(); ++j)
                    if (Kxz(i,j) < 0.0)
                        Kxz(i,j) = 0.0f;
            }
        }

        /*
         * @brief Sparse kernel.
         * @param x input vector
         * @param z input vector
         * @return Kxz covariance matrix
         * @ref A sparse covariance function for exact gaussian process inference in large datasets.
         */
        void covSparse(const MatrixXType &x, const MatrixXType &z, MatrixKType &Kxz) const {
            dist(x / ell, z / ell, Kxz);
            Kxz = (((2.0f + (Kxz * 2.0f * 3.1415926f).array().cos()) * (1.0f - Kxz.array()) / 3.0f) +
                  (Kxz * 2.0f * 3.1415926f).array().sin() / (2.0f * 3.1415926f)).matrix() * sf2;

            // Clean up for values with distance outside length scale
            // Possible because Kxz <= 0 when dist >= ell
            for (int i = 0; i < Kxz.rows(); ++i)
            {
                for (int j = 0; j < Kxz.cols(); ++j)
                    if (Kxz(i,j) < 0.0)
                        Kxz(i,j) = 0.0f;
            }
        }

        void covCountingSensorModel(const MatrixXType &x, const MatrixXType &z, MatrixKType &Kxz) const {
          Kxz = MatrixKType::Ones(x.rows(), z.rows());
        }

        T flow_sf2; //vel kernel signal variance
        T flow_ell; //vel kernel length-scale
        T m_resol; //map resolution
        bool temporal;


        T sf2;    // signal variance
        T ell;    // length-scale
        int nc;   // number of classes

        MatrixXType x;   // temporary storage of training data
        MatrixYType y;   // temporary storage of training labels
        //MatrixYType v; // temporary storage for flow of training data
        MatrixXType v;
        std::vector<T> y_vec;
        const std::vector<bool> dynamic; // a vector of the size nc that denotes whether a class is likely to be dynamic

        bool trained;    // true if bgkinference stored training data
    };

    typedef SemanticBKInference<3, float> SemanticBKI3f;

}
