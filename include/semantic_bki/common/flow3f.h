#include "point3f.h"

namespace semantic_bki{


    class Vector3fWithFlow : public Vector3{
        public:
        /*!
         * \brief Default constructor
         */
            Vector3fWithFlow() : Vector3(){
                data[0] = data[1] = data[2]; }
        
        /*!
         * \brief Copy constructor
         *
         * @param other a vector of dimension 6
        */
            Vector3fWithFlow(const Vector3fWithFlow &other) : Vector3(other){
                data[0] = other(3);
                data[1] = other(4);
                data[2] = other(5);
            }
        
        /*!
         * \brief Constructor
         *
         * Constructs a 6D vector from
         * 6 single values x, y, z, vx, vy, vz
         */
        Vector3fWithFlow(float x, float y, float z, float vx, float vy, float vz) : 
                Vector3(x, y, z) {
            data[0] = vx;
            data[1] = vy;
            data[2] = vz;
        }

        /*!
         * \brief Assignment operator
         *
         * @param other a vector of dimension 6
         */
        inline Vector3fWithFlow &operator=(const Vector3fWithFlow &other) {
            Vector3::operator=(other);
            data[0] = other(3);
            data[1] = other(4);
            data[2] = other(5);
            return *this;
        }
        inline const float &operator()(unsigned int i) const {
            if (i < 3)
                return Vector3::operator()(i);
            else
                return data[i - 3];
        }
        inline float &operator()(unsigned int i) {
            if (i < 3)
                return Vector3::operator()(i);
            else
                return data[i - 3];
        }
        inline float &vx() {
            return operator()(3);
        }

        inline float &vy() {
            return operator()(4);
        }

        inline float &vz() {
            return operator()(5);
        }

        inline const float &vx() const {
            return operator()(3);
        }

        inline const float &vy() const {
            return operator()(4);
        }

        inline const float &vz() const {
            return operator()(5);
        }
        
        inline double flow_norm() const{
            return sqrt( vx() * vx() + vy() * vy() + vz() * vz());
        }

        inline Vector3 point() const{
            return Vector3(x(), y(), z());
        } 

        

        private:
            float data[3];
        
    };

    typedef Vector3fWithFlow flow3f;
}