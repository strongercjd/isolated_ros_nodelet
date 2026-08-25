#pragma once

#include <iostream>
#include <eigen3/Eigen/Core>

namespace unity
{
    /*
    指定了一个 2x2 的浮点数矩阵，也就是说，这个矩阵有 2 行和 2 列。这是 Eigen 库中一种方便创建和操作二维数组的方式。
    Eigen::Matrix<float, 2, 2> m;  // 创建一个2x2的浮点数矩阵
    m(0,0) = 1.0;  // 设置第一行第一列的值为1.0
    m(1,1) = 2.0;  // 设置第二行第二列的值为2.0
    用于表示旋转方程
    */
    typedef Eigen::Matrix<float, 2, 2> ROT2D;
    /**
     * @brief Rotation 旋转，用于表示旋转方程
     *
     */
    class Rotation
    {
    public:
        Rotation(float angle);
        Rotation(const ROT2D &rot);

        Rotation operator*(const Rotation &rot) const;
        Eigen::Vector2f operator*(const Eigen::Vector2f &p) const;

        const ROT2D &rotation() const
        {
            return _rot;
        }

        ROT2D &rotation()
        {
            return _rot;
        }

        ROT2D transpose() const
        {
            /*
            将这个矩阵进行转置。在矩阵运算中，转置操作将矩阵的行和列互换。也就是说，矩阵的第一个行变成第一个列，第二个行变成第二个列
            1 2  
            3 4
            上述矩阵转置后为
            1 3  
            2 4
            这里个人理解：
            _rot旋转矩阵表示逆时针旋转30度，转置后表示顺时针旋转30度
            
            逆矩阵是矩阵的一种重要概念，指的是如果存在另一个矩阵，使得这两个矩阵相乘的结果为单位矩阵，则这个矩阵就是原矩阵的逆矩阵。
            单位矩阵是一种特殊的方阵，其主对角线上的元素都为1，其他位置的元素都为0。

            正交矩阵是一种特殊的方阵，其实质在于其转置矩阵等于其逆矩阵

            旋转矩阵为
            |std::cos(angle) -std::sin(angle)|
            |std::sin(angle) std::cos(angle) |
            因为旋转矩阵是正交矩阵，所以旋转矩阵的置等于旋转矩阵的逆，这里是为了求旋转矩阵的逆
            */
            return _rot.transpose();
        }

        float angle() const;

    private:
        ROT2D _rot; // 旋转方程
    };
    /**
     * @brief Rigid2f是Rigid body 2D float的缩写，用于描述物理学中的刚体（即没有弹性、不能变形的物体）在二维空间中的运动
     * 
     * 这个类就是用来表示刚体运动的，包括旋转和平移，从原点到当前位置的旋转和平移，其实也就可以表示为位姿(位置和姿态)
     *
     */
    class Rigid2f
    {
    public:
        typedef Eigen::Vector2f Vector;

        Rigid2f();
        Rigid2f(float ang, float x, float y);
        Rigid2f(const Rotation &rot, const Eigen::Vector2f &pose);
        Rigid2f operator*(const Rigid2f &right) const;
        Eigen::Vector2f operator*(const Eigen::Vector2f &p) const;
        const Eigen::Vector2f &pose() const
        {
            return _pose;
        }

        Eigen::Vector2f &pose()
        {
            return _pose;
        }

        const float angle() const
        {
            return _rot.angle();
        }

        void printf_myself() const
        {
            std::cout << "angle: " << angle() << " pose: " << pose().x() << " " << pose().y() << std::endl;
        }

        Rigid2f inverse() const;

        static Rigid2f Indentity();

    private:
        /* 这里可以理解为位姿，也就是_pose表示坐标，_rot表示角度
           也可以理解为平移向量和旋转方程，_pose表示平移向量，_rot表示旋转方程(相对与原点坐标的) */
        Eigen::Vector2f _pose;//坐标 /* Eigen库中的2维浮点向量 */
        Rotation _rot;//旋转矩阵
    };

    inline std::ostream &operator<<(std::ostream &os, const Rigid2f &rigid)
    {
        os << rigid.angle() << '\t' << rigid.pose().x() << '\t' << rigid.pose().y();
        return os;
    }

}