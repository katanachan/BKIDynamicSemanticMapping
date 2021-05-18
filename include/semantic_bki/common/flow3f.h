/**
*    Description: A library built on top of class Vector3 to
*    provide additional functionalities to store and 
*    access the velocity of a point in 3-dimensions
*    @file flow3f.h
*    @author Aishwarya Unnikrishnan
*    @version 1.1 08/04/20 
**/

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
        /*!
        *  \brief Function call operator that will invoke Vector3
        *          when the user wants x, y or z : indexed as 0, 1, 2
        *          If you want velocities vx, vym vz are indexed as 3-5
        * @param an int to index the 6D vector
        */
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
        
        inline Vector3fWithFlow operator-(const Vector3 &other) const {
            Vector3fWithFlow result(*this);
            result(3) -= other(0);
            result(4) -= other(1);
            result(5) -= other(2);
            return result;
        }

        inline Vector3fWithFlow operator+(const Vector3 &other) const {
            Vector3fWithFlow result(*this);
            result(3) += other(0);
            result(4) += other(1);
            result(5) += other(2);
            return result;
        }



        private:
            float data[3];
        
    };

    inline std::ostream &operator<<(std::ostream &out, semantic_bki::Vector3fWithFlow const &v){
        return out << '(' << v.x() << ' ' << v.y() << ' ' 
                << v.z() << ' ' << v.vx() << ' ' << v.vy() << ' ' << v.vz() <<')';

    }
    typedef Vector3fWithFlow flow3f;
}