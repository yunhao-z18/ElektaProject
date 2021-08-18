#pragma once

#include<Eigen\Dense>
using namespace Eigen;

// Generic functor
template<typename _Scalar, int NX = Dynamic, int NY = Dynamic>
struct Functor
{
    typedef _Scalar Scalar;
    enum {
        InputsAtCompileTime = NX,
        ValuesAtCompileTime = NY
    };
    typedef Matrix<Scalar, InputsAtCompileTime, 1> InputType;
    typedef Matrix<Scalar, ValuesAtCompileTime, 1> ValueType;
    typedef Matrix<Scalar, ValuesAtCompileTime, InputsAtCompileTime> JacobianType;

    const int m_inputs, m_values;

    Functor() : m_inputs(InputsAtCompileTime), m_values(ValuesAtCompileTime) {}
    Functor(int inputs, int values) : m_inputs(inputs), m_values(values) {}

    int inputs() const { return m_inputs; }
    int values() const { return m_values; }

    // you should define that in the subclass :
  //  void operator() (const InputType& x, ValueType* v, JacobianType* _j=0) const;
};
