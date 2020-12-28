#ifndef _DUALQUATERNION_H_
#define _DUALQUATERNION_H_

#include <iostream>
#include <math.h>

template<class Scalar>
struct Scalar3 {
        Scalar x;
        Scalar y;
        Scalar z;
        
        // Initializer
        Scalar3(): x((Scalar)0.0f), y((Scalar)0.0f), z((Scalar)0.0f) {}
        
        Scalar3(Scalar a): x(a), y(a), z(a) {}
        
        Scalar3(Scalar X, Scalar Y, Scalar Z): x(X), y(Y), z(Z) {}
        
        // Scalar Multiplication
        Scalar3 operator* (Scalar s) {
            return Scalar3(s*x, s*y, s*z);
        }
        
        // outer product
        Eigen::Matrix<Scalar, 3, 3> operator* (Scalar3 v) {
            Eigen::Matrix<Scalar, 3, 3> res;
            res <<  x*v.x, x*v.y, x*v.z,
                    y*v.x, y*v.y, y*v.z,
                    z*v.x, z*v.y, z*v.z;
            return res;
        }
        
        //matrix product
        Scalar3 operator* (Eigen::Matrix3f M) {
            return Scalar3(x*M(0,0) + y*M(0,1) + z*M(0,2),
                          x*M(1,0) + y*M(1,1) + z*M(1,2),
                          x*M(2,0) + y*M(2,1) + z*M(2,2));
        }
        
        // Addition
        Scalar3 operator+ (Scalar3 a) {
            return Scalar3(a.x + x, a.y + y, a.z + z);
        }
        
        // Subtraction
        Scalar3 operator- (Scalar3 a) {
            return Scalar3(x - a.x, y - a.y, z - a.z);
        }
        
    };

    template<class Scalar>
    inline static Scalar3<Scalar> make_Scalar3(Scalar X, Scalar Y, Scalar Z) {
        return Scalar3<Scalar>(X, Y, Z);
    }

    template<class Scalar>
    inline static Scalar3<Scalar> make_Scalar3(Scalar X) {
        return Scalar3<Scalar>(X);
    }

    template<class Scalar>
    static Scalar3<Scalar> cross (Scalar3<Scalar> a, Scalar3<Scalar> b) {
        return Scalar3<Scalar>(a.y*b.z - a.z*b.y,
                      -a.x*b.z + a.z*b.x,
                      a.x*b.y - a.y*b.x);
    }

    template<class Scalar>
    static Scalar dot (Scalar3<Scalar> a, Scalar3<Scalar> b) {
        return a.x*b.x + a.y*b.y + a.z*b.z;
    }


    template<class Scalar>
    static Eigen::Matrix<Scalar, 3, 3> outer_prod_tri_order(Scalar3<Scalar> a, Scalar3<Scalar> b) {
        Eigen::Matrix<Scalar, 3, 3> res;
        res << a.x*b.x, a.x*b.y, a.x*b.z,
               a.y*b.x, a.y*b.y, a.y*b.z,
               a.z*b.x, a.z*b.y, a.z*b.z;
        
        return res;
    }


    template<class Scalar>
    struct Scalar4 {
        Scalar x;
        Scalar y;
        Scalar z;
        Scalar w;
        
        Scalar4(): x((Scalar)0.0f), y((Scalar)0.0f), z((Scalar)0.0f), w((Scalar)0.0f) {}
        Scalar4(Scalar X, Scalar Y, Scalar Z, Scalar W): x(X), y(Y), z(Z), w(W) {}
                
        // Addition
        Scalar4 operator+ (Scalar4 a) {
            return Scalar4(a.x + x, a.y + y, a.z + z, a.w + w);
        }
    };

    template<class Scalar>
    inline static Scalar4<Scalar> make_Scalar4(Scalar X, Scalar Y, Scalar Z, Scalar W) {
        return Scalar4<Scalar>(X, Y, Z, W);
    }

    template<class Scalar>
    static Eigen::Matrix<Scalar, 4, 4> outer_prod(Scalar4<Scalar> a, Scalar4<Scalar> b) {
        Eigen::Matrix<Scalar, 4, 4> res;
        res << a.x*b.x, a.y*b.x, a.z*b.x, a.w*b.x,
               a.x*b.y, a.y*b.y, a.z*b.y, a.w*b.y,
               a.x*b.z, a.y*b.z, a.z*b.z, a.w*b.z,
               a.x*b.w, a.y*b.w, a.z*b.w, a.w*b.w;
        
        return res;
    }

    template<class Scalar>
    static Eigen::Matrix<Scalar, 4, 4> outer_prod_quat_order(Scalar4<Scalar> a, Scalar4<Scalar> b) {
        Eigen::Matrix<Scalar, 4, 4> res;
        res << a.w*b.w, a.w*b.x, a.w*b.y, a.w*b.z,
               a.x*b.w, a.x*b.x, a.x*b.y, a.x*b.z,
               a.y*b.w, a.y*b.x, a.y*b.y, a.y*b.z,
               a.z*b.w, a.z*b.x, a.z*b.y, a.z*b.z;
        
        return res;
    }


// template quaternion
template<class Scalar>
struct QuaternionScalar {
    Scalar4<Scalar> value;

    // Quaternion(double X, double Y, double Z, double W) {
    //     value.x = X;
    //     value.y = Y;
    //     value.z = Z;
    //     value.w = W;
    // }


    QuaternionScalar(Scalar X, Scalar Y, Scalar Z, Scalar W) {
        value.x = X;
        value.y = Y;
        value.z = Z;
        value.w = W;
    }
    
    QuaternionScalar (Scalar3<Scalar> t, Scalar W) {
        value.x = t.x;
        value.y = t.y;
        value.z = t.z;
        value.w = W;
    }
    
    QuaternionScalar (Scalar3<Scalar> r) {
        Scalar norm_r = sqrt(r.x*r.x + r.y*r.y + r.z*r.z);
        if (norm_r == (Scalar)0.0) {
            value.x = (Scalar)0.0;
            value.y = (Scalar)0.0;
            value.z = (Scalar)0.0;
            value.w = (Scalar)1.0;
            return;
        }
        Scalar f = sin(norm_r/(Scalar)2.0)/norm_r;
        value.x = f*r.x;
        value.y = f*r.y;
        value.z = f*r.z;
        value.w = cos(norm_r/(Scalar)2.0); // COMMENTED BY STEPHANE THE 22nd of AUGUST
        // float f = sin(norm_r)/norm_r;
        // value.x = f*r.x;
        // value.y = f*r.y;
        // value.z = f*r.z;
        // value.w = cos(norm_r);
    }
    
    QuaternionScalar(Eigen::Matrix<Scalar, 3, 3> rot) {
        Scalar trace = rot(0,0) + rot(1,1) + rot(2,2);
        if( trace > 0 ) {
            Scalar s = 0.5f / sqrt(trace + 1.0f);
            value.w = 0.25f / s;
            value.x = ( rot(2,1) - rot(1,2) ) * s;
            value.y = ( rot(0,2) - rot(2,0) ) * s;
            value.z = ( rot(1,0) - rot(0,1) ) * s;
        } else {
            if ( rot(0,0) > rot(1,1) && rot(0,0) > rot(2,2) ) {
                Scalar s = 2.0f * sqrt( 1.0f + rot(0,0) - rot(1,1) - rot(2,2));
                value.w = (rot(2,1) - rot(1,2) ) / s;
                value.x = 0.25f * s;
                value.y = (rot(0,1) + rot(1,0) ) / s;
                value.z = (rot(0,2) + rot(2,0) ) / s;
            } else if (rot(1,1) > rot(2,2)) {
                Scalar s = 2.0f * sqrt( 1.0f + rot(1,1) - rot(0,0) - rot(2,2));
                value.w = (rot(0,2)- rot(2,0) ) / s;
                value.x = (rot(0,1) + rot(1,0)) / s;
                value.y = 0.25f * s;
                value.z = (rot(1,2) + rot(2,1) ) / s;
            } else {
                Scalar s = 2.0f * sqrtf( 1.0f + rot(2,2) - rot(0,0) - rot(1,1) );
                value.w = (rot(1,0) - rot(0,1) ) / s;
                value.x = (rot(0,2) + rot(2,0) ) / s;
                value.y = (rot(1,2) + rot(2,1) ) / s;
                value.z = 0.25f * s;
            }
        }
    }
    
    Scalar3<Scalar> Vector () {
        Scalar3<Scalar> r;
        r.x = value.x;
        r.y = value.y;
        r.z = value.z;
        return r;
    }
    
    Scalar W () {
        return value.w;
    }
    
    // Scalar Multiplication
    QuaternionScalar<Scalar> operator* (Scalar s) {
        Scalar3<Scalar> v1;
        v1.x = s*value.x;
        v1.y = s*value.y;
        v1.z = s*value.z;
        return QuaternionScalar<Scalar>(v1, s*value.w);
    }
    
    // Multiplication
    QuaternionScalar<Scalar> operator* (QuaternionScalar<Scalar> b) {
        Scalar3<Scalar> v1;
        v1.x = value.x;
        v1.y = value.y;
        v1.z = value.z;
        Scalar w1 = value.w;
        Scalar3<Scalar> v2 = b.Vector();
        Scalar w2 = b.W();
        
        return QuaternionScalar<Scalar> (v2*w1 + v1*w2 + cross(v1,v2), w1*w2 - dot(v1,v2));
    }
    
    // Addition
    QuaternionScalar<Scalar> operator+ (QuaternionScalar<Scalar> b) {
        Scalar3<Scalar> v1;
        v1.x = value.x;
        v1.y = value.y;
        v1.z = value.z;
        Scalar w1 = value.w;
        Scalar3<Scalar> v2 = b.Vector();
        Scalar w2 = b.W();
        
        return QuaternionScalar<Scalar> (v1 + v2, w1 + w2);
    }
    
    // Conjugate
    QuaternionScalar Conjugate () {
        return QuaternionScalar<Scalar>(-value.x, -value.y, -value.z, value.w);
    }
    
    // Magnitude
    Scalar Magnitude () {
        return sqrt(value.x*value.x + value.y*value.y + value.z*value.z + value.w*value.w);
    }
    
    Scalar Dot(QuaternionScalar<Scalar> b) {
        Scalar3<Scalar> v1;
        v1.x = value.x;
        v1.y = value.y;
        v1.z = value.z;
        Scalar w1 = value.w;
        Scalar3<Scalar> v2 = b.Vector();
        Scalar w2 = b.W();
        
        return w1*w2 + dot(v1,v2);
    }
    
    //Normalize
    QuaternionScalar Normalize ( ) {
        Scalar norm = sqrt(value.x*value.x + value.y*value.y + value.z*value.z + value.w*value.w);
        return QuaternionScalar<Scalar>(value.x/norm, value.y/norm, value.z/norm, value.w/norm);
    }

    void Print()
    {
        cout << value.x << ", " << value.y << ", " << value.z << ", " << value.w << endl;
    }
    //norm
    Scalar Norm(){
        Scalar norm = sqrt(value.x*value.x + value.y*value.y + value.z*value.z + value.w*value.w);
        return norm;
    }
};


template<class Scalar>
static Scalar3<Scalar> logOfQuaternion(Quaternion qi){
  Scalar3<Scalar> thetai;  
  const Scalar3<Scalar> q123 = qi.Vector();
  const Scalar sin_squared_theta = q123.x*q123.x + q123.y*q123.y + q123.z*q123.z;

  if (sin_squared_theta > (Scalar)(0.0f)) {
  	const Scalar sin_theta = sqrt(sin_squared_theta);
  	const Scalar cos_theta = qi.Scalar();

    // If cos_theta is negative, theta is greater than pi/2, which
    // means that angle for the angle_axis vector which is 2 * theta
    // would be greater than pi.
    //
    // While this will result in the correct rotation, it does not
    // result in a normalized angle-axis vector.
    //
    // In that case we observe that 2 * theta ~ 2 * theta - 2 * pi,
    // which is equivalent saying
    //
    //   theta - pi = atan(sin(theta - pi), cos(theta - pi))
    //              = atan(-sin(theta), -cos(theta))
    //
    const Scalar two_theta =
        (Scalar)2.0f * ((cos_theta < (0.0)) ? atan2(-sin_theta, -cos_theta)
                                       : atan2(sin_theta, cos_theta));
    const Scalar k = two_theta / sin_theta;

    thetai.x = q123.x * k;
    thetai.y = q123.y * k;
    thetai.z = q123.z * k;
  }else{
  	    // For zero rotation, sqrt() will produce NaN in the derivative since
    // the argument is zero.  By approximating with a Taylor series,
    // and truncating at one term, the value and first derivatives will be
    // computed correctly when Jets are used.
    const Scalar k = (2.0f);
    thetai.x = q123.x * k;
    thetai.y = q123.y * k;
    thetai.z = q123.z * k;
  }




  return thetai;

}

template<class Scalar>
struct DualQuaternionScalar {
    QuaternionScalar<Scalar> m_real = QuaternionScalar<Scalar>((Scalar)0.0,(Scalar)0.0,(Scalar)0.0,(Scalar)1.0);
    QuaternionScalar<Scalar> m_dual = QuaternionScalar<Scalar>((Scalar)0.0,(Scalar)0.0,(Scalar)0.0,(Scalar)0.0);

    DualQuaternionScalar() {
        m_real = QuaternionScalar<Scalar>((Scalar)0.0,(Scalar)0.0,(Scalar)0.0,(Scalar)1.0);
        m_dual = QuaternionScalar<Scalar>((Scalar)0.0,(Scalar)0.0,(Scalar)0.0,(Scalar)0.0);
    }
        
    DualQuaternionScalar(QuaternionScalar<Scalar> r, QuaternionScalar<Scalar> d) {
        //m_real = r.Normalize();
        m_real = r;
        m_dual = d;
    }
    
    DualQuaternionScalar(QuaternionScalar<Scalar> r, Scalar3<Scalar> t) {
        m_real = r.Normalize();
        m_dual = (QuaternionScalar<Scalar>(t,(Scalar)0.0) * m_real) * (Scalar)0.5;
    }
    
    DualQuaternionScalar(float3 r, float3 t) {
        m_real = QuaternionScalar<Scalar>(r);
        m_dual = (QuaternionScalar<Scalar>(t,0.0) * m_real) * 0.5;
    }
    
    DualQuaternionScalar(Eigen::Matrix4f transfo) {
        Eigen::Matrix<Scalar, 3, 3> rotation;
        rotation << transfo(0,0), transfo(0,1), transfo(0,2),
                    transfo(1,0), transfo(1,1), transfo(1,2),
                    transfo(2,0), transfo(2,1), transfo(2,2);
        m_real = QuaternionScalar<Scalar>(rotation);

        Scalar3<Scalar> t = make_Scalar3(transfo(0,3), transfo(1,3), transfo(2,3));

        m_dual = (QuaternionScalar<Scalar>(t,0.0) * m_real) * 0.5;
    }
    
    // Inverse
    DualQuaternionScalar Inv (DualQuaternion a) {
        if(a.m_real.Magnitude() == 0)
            return DualQuaternion();

        QuaternionScalar<Scalar> p_1 = InvQ(a.m_real);
        QuaternionScalar<Scalar> p_2 = a.m_dual * p_1;
        DualQuaternionScalar<Scalar> q_1 = DualQuaternionScalar<Scalar>(p_1, QuaternionScalar<Scalar>(0.0,0.0,0.0,0.0));
        DualQuaternionScalar<Scalar> q_2 = DualQuaternionScalar<Scalar>(QuaternionScalar<Scalar>(0.0,0.0,0.0,1.0), p_2 * (Scalar)(-1.0));

        return q_1 * q_2;
    }
    
    QuaternionScalar<Scalar> Real() {
        return m_real;
    }
    
    QuaternionScalar<Scalar> Dual () {
        return m_dual;
    }
    
    DualQuaternionScalar<Scalar> Identity() {
        return DualQuaternionScalar<Scalar>(QuaternionScalar<Scalar>(make_Scalar3<Scalar>(0.0f), (Scalar)1.0), QuaternionScalar<Scalar>(make_Scalar3(0.0f), 0.0f));
    }
    
    //Addition
    DualQuaternionScalar<Scalar> operator+ (DualQuaternionScalar<Scalar> b) {
        return DualQuaternionScalar<Scalar>(m_real + b.Real(), m_dual + b.Dual());
    }
    
    // Scalar multiplication
    DualQuaternionScalar<Scalar> operator* (Scalar s) {
        return DualQuaternionScalar<Scalar>(m_real*s, m_dual*s);
    }
    
    // Multiplication
    DualQuaternionScalar<Scalar> operator* (DualQuaternionScalar<Scalar> b) {
        return DualQuaternionScalar<Scalar>(m_real*b.Real(), m_real*b.Dual() + m_dual*b.Real());
    }
    
    // Division
    DualQuaternionScalar<Scalar> operator/ (DualQuaternionScalar<Scalar> b) {
        if(m_real.Magnitude() == 0.0f)
            return DualQuaternionScalar<Scalar>();
        DualQuaternion c = Inv(b);
        return DualQuaternionScalar<Scalar>(m_real*c.Real(), m_real*c.Dual() + m_dual*c.Real());
    }
    
    // Conjugate
    DualQuaternionScalar<Scalar> Conjugate () {
        return DualQuaternionScalar<Scalar> (m_real.Conjugate(), m_dual.Conjugate());
    }
    
    // Conjugate
    DualQuaternionScalar<Scalar> DualConjugate1 () {
        return DualQuaternionScalar<Scalar> (m_real, m_dual * ((Scalar)-1.0));
    }
    
    // Conjugate
    DualQuaternionScalar<Scalar> DualConjugate2 () {
        return DualQuaternionScalar<Scalar> (m_real.Conjugate(), m_dual.Conjugate() * ((Scalar)-1.0));
    }
    
    Scalar Dot (DualQuaternionScalar<Scalar> b) {
        return m_real.Dot(b.Real());
    }
    
    // Magnitude
    Scalar Magnitude () {
        return m_real.Dot(m_real);
    }
    
    Scalar Norm () {
        return m_real.Magnitude();
    }
    
    DualQuaternionScalar<Scalar> Normalize () {
        Scalar norm_a = m_real.Norm();//if one uses m_real.Magnitude()) it is the squared norm
        if (norm_a == (Scalar)0.0)
            return DualQuaternionScalar<Scalar>();
        QuaternionScalar<Scalar> real_part = m_real * ((Scalar)1.0/norm_a);
        QuaternionScalar<Scalar> dual_factor = (m_dual * m_real.Conjugate() ) * ((Scalar)1.0/(norm_a*norm_a));//(m_dual * m_real.Conjugate()+ Quaternion(0.0,0.0,0.0,-1.0f*m_real.Dot(m_dual)) ) * (1.0f/(norm_a*norm_a));
        dual_factor.value.w = (Scalar)0.0;
        return DualQuaternionScalar<Scalar> (real_part, dual_factor*real_part);
    }
    
    QuaternionScalar<Scalar> GetRotation () {
        return m_real;
    }
    
    Scalar3<Scalar> GetTranslation () {
        QuaternionScalar<Scalar> t = (m_dual * (Scalar)2.0) * m_real.Conjugate();
        return t.Vector();
    }
    
    /*float4x4 DualQuaternionToMatrix () {
        float4x4 M;*/
    Eigen::Matrix<Scalar, 4, 4> DualQuaternionToMatrix () {
        Eigen::Matrix<Scalar, 4, 4> M;
        
        Scalar mag = m_real.Dot(m_real);
        if (mag < 0.000001f)
            return M;
        DualQuaternionScalar<Scalar> q = DualQuaternionScalar<Scalar>(m_real*(1.0f/mag), m_dual*(1.0f/mag));
        
        Scalar w = q.m_real.W();
        Scalar3<Scalar> v = q.m_real.Vector();
        
        M(0,0) = w*w + v.x*v.x - v.y*v.y - v.z*v.z;
        M(1,0) = 2.0f*v.x*v.y + 2.0f*w*v.z;
        M(2,0) = 2.0f*v.x*v.z - 2.0f*w*v.y;
        M(3,0) = 0.0f;
        
        M(0,1) = 2.0f*v.x*v.y - 2.0f*w*v.z;
        M(1,1) = w*w + v.y*v.y - v.x*v.x - v.z*v.z;
        M(2,1) = 2.0f*v.y*v.z + 2.0f*w*v.x;
        M(3,1) = 0.0f;
        
        M(0,2) = 2.0f*v.x*v.z + 2.0f*w*v.y;
        M(1,2) = 2.0f*v.y*v.z - 2.0f*w*v.x;
        M(2,2) = w*w + v.z*v.z - v.x*v.x - v.y*v.y;
        M(3,2) = 0.0f;
        
        QuaternionScalar<Scalar> t = (m_dual * (Scalar)2.0) * m_real.Conjugate();
        Scalar3<Scalar> t_v = t.Vector();
        M(0,3) = t_v.x;
        M(1,3) = t_v.y;
        M(2,3) = t_v.z;
        M(3,3) = 1.0f;
        
        return M;
    }
    void Print()
    {
        cout << "Real: "; m_real.Print(); 
        cout << "Dual: "; m_dual.Print();
    }
};




#endif