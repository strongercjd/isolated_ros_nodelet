#include "unity/transform.h"

namespace unity
{
    /**
     * @brief Construct a new Rotation:: Rotation object
     *
     * @param angle 传参是角度
     */
    Rotation::Rotation(float angle)
    {
        /*
        旋转方程为
        |std::cos(angle) -std::sin(angle)|
        |std::sin(angle) std::cos(angle) |
        相当于X轴旋转到
        |std::cos(angle)|
        |std::sin(angle)|
        Y轴旋转到
        |-std::sin(angle)|
        |std::cos(angle) |

        _rot*p; 就坐标系经过_rot旋转方程，P点在新的坐标系下的坐标
        */
        _rot << std::cos(angle), -std::sin(angle),
            std::sin(angle), std::cos(angle);
    }
    /**
     * @brief Construct a new Rotation:: Rotation object
     *
     * @param rot 传参就是旋转方程
     */
    Rotation::Rotation(const ROT2D &rot)
        : _rot(rot)
    {
    }
    /**
     * @brief *运算符重载 两个旋转方程相乘
     * 
     * @param rot 
     * @return Rotation 
     */
    Rotation Rotation::operator*(const Rotation &rot) const
    {
        Rotation tmp(_rot * rot._rot);
        return Rotation(tmp.angle());
    }
    /**
     * @brief 旋转方程和向量相乘
     * 
     * @param p 
     * @return Eigen::Vector2f 
     */
    Eigen::Vector2f Rotation::operator*(const Eigen::Vector2f &p) const
    {
        return _rot * p;
    }
    /**
     * @brief 获取旋转方程的角度
     * 
     * @return float 
     */
    float Rotation::angle() const
    {
        return std::atan2(_rot(1, 0), _rot(0, 0));
    }

    Rigid2f::Rigid2f()
        : _pose(0, 0), _rot(0)
    {
    }

    Rigid2f::Rigid2f(float ang, float x, float y)
        : _pose(x, y), _rot(ang)
    {
    }

    Rigid2f::Rigid2f(const Rotation &rot, const Eigen::Vector2f &pose)
        : _pose(pose), _rot(rot)
    {
    }
    /**
     * @brief 两个位姿相乘
     * 
     * @param right 
     * @return Rigid2f 
     */
    Rigid2f Rigid2f::operator*(const Rigid2f &right) const
    {
        return Rigid2f(_rot * right._rot, _rot * right._pose + _pose);
    }
    /**
     * @brief 位姿和向量相乘，也就是坐标系变换
     * 
     * @param right 
     * @return Rigid2f 
     */
    Eigen::Vector2f Rigid2f::operator*(const Eigen::Vector2f &p) const
    {
        return _rot * p + _pose;
    }
    
    /**
    * 计算并返回当前Rigid2f对象的逆变换。
    * 
    * 该函数不接受任何参数。
    * 
    * @return 返回一个新的Rigid2f对象，代表当前对象的逆变换。该对象包含一个逆旋转角度和逆平移向量。
    */
    Rigid2f Rigid2f::inverse() const
    {
        // 获取旋转矩阵的转置，即逆旋转矩阵
        const Rotation transposed_rot(_rot.transpose());//这里只是旋转矩阵的逆矩阵

        /* 计算逆变换后的位置，得到角度和坐标 */
        // 计算逆变换后的位置，此处用了旋转矩阵的转置和原位置的乘积，并取负值
        const Eigen::Vector2f new_pos(-(transposed_rot * _pose));//这里算平移向量的逆变换
        // 返回包含逆旋转角度和新位置的新Rigid2f对象
        return Rigid2f(transposed_rot.angle(), new_pos(0), new_pos(1));
        // 注：原工程误用 new_pos(-(_pose))（平移未随旋转一起求逆，仅角度为 0 时
        // 碰巧正确），初始朝向非 0 时 odom_delta 会整体转错方向（实测位姿从
        // (6,-4,90°) 瞬间跳到 (8,-14)），此处按注释的正确公式修复。
    }

    Rigid2f Rigid2f::Indentity()
    {
        return Rigid2f();
    }

}
