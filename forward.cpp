#include <iostream>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl_parser/kdl_parser.hpp>

int main() {
    // 解析URDF，构建KDL::Tree
    KDL::Tree tree;
    kdl_parser::treeFromFile("../ur5.urdf", tree);

    // 获取机械臂的KDL::Chain
    KDL::Chain chain;
    tree.getChain("base", "wrist3", chain);

    std::cout << chain.getNrOfJoints() << std::endl;

    // 正解：已知关节角度，求解末端坐标位置
    KDL::ChainFkSolverPos_recursive fk(chain);

    // 关节角度
    KDL::JntArray q_in(chain.getNrOfJoints());

    // 赋值关节角度（30度转换为弧度）
    q_in(0) = KDL::deg2rad * 30;
    q_in(1) = KDL::deg2rad * 30;
    q_in(2) = KDL::deg2rad * 30;
    q_in(3) = KDL::deg2rad * 30;
    q_in(4) = KDL::deg2rad * 30;
    q_in(5) = KDL::deg2rad * 30;


    // 末端坐标位置和姿态
    KDL::Frame p_out;
    int state = fk.JntToCart(q_in, p_out);

    std::cout << state << std::endl;

    if (state >= 0) {
        std::cout << p_out << std::endl;

        KDL::Vector p = p_out.p;
        std::cout << "xyz: " << p.x() << ", " << p.y() << ", " << p.z() << std::endl;

        double roll, pitch, yaw;
        p_out.M.GetRPY(roll, pitch, yaw);
        std::cout << "rpy: " << roll * KDL::rad2deg << ", "
                  << pitch * KDL::rad2deg << ", "
                  << yaw * KDL::rad2deg << std::endl;
    }

    return 0;
}

/*
phoenix@ubuntu:~/fz/robot/build$ ./02_forward 
6
0
[[   -0.216506,      -0.875,    0.433013;
       -0.625,   -0.216506,       -0.75;
         0.75,   -0.433013,        -0.5]
[   -0.316418,    -0.39102,   -0.504189]]
xyz: -0.316418, -0.39102, -0.504189
rpy: -139.107, -48.5904, -109.107
*/