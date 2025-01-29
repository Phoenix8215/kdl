#include <iostream>
#include <kdl/tree.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames_io.hpp>
#include <kdl/kinfam_io.hpp>
#include <kdl/chainiksolverpos_nr.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl_parser/kdl_parser.hpp>

// 通过kdl库，去加载urdf的模型，模型文件变为可操作的对象
// kdl_parser：将urdf这个XML文件，转换为tree

int main() {
    KDL::Tree tree;
    kdl_parser::treeFromFile("../ur5.urdf", tree);

    // 获取机械臂结构
    KDL::Chain chain;
    tree.getChain("base", "wrist3", chain);

    std::cout << chain.getNrOfJoints() << std::endl;

    // 反解：已知末端位姿，求解关节角度
    KDL::ChainFkSolverPos_recursive fk(chain);
    KDL::ChainIkSolverVel_pinv ikVel(chain);
    int max_iter = 500;      // 增加最大迭代次数
    double eps = 1e-3;       // 降低精度要求
    KDL::ChainIkSolverPos_NR ik(chain, fk, ikVel, max_iter, eps);

    // 关节角
    KDL::JntArray q_init(6);
    q_init(0) = KDL::deg2rad * 30;
    q_init(1) = KDL::deg2rad * -10;
    q_init(2) = KDL::deg2rad * 10;
    q_init(3) = KDL::deg2rad * 15;
    q_init(4) = KDL::deg2rad * 10;
    q_init(5) = KDL::deg2rad * 10;

    // 目标末端的位置信息
    // KDL::Frame p_in(KDL::Rotation::EulerZYX(
    //     KDL::deg2rad * -109,
    //     KDL::deg2rad * -10,
    //     KDL::deg2rad * -124),
    //     KDL::Vector(-0.10784, -0.31527, -0.56881));

    // 目标位姿 (调整到合理范围)
    KDL::Frame p_in(KDL::Rotation::EulerZYX(
        KDL::deg2rad * 178.57, 
        KDL::deg2rad * 1.43,  
        KDL::deg2rad * -179.97),
        KDL::Vector(-0.12009, -0.431804, 0.14606));

    // 计算逆解
    KDL::JntArray q_out;
    int state = ik.CartToJnt(q_init, p_in, q_out);
    std::cout << "state :" << state << std::endl;
    if (state >= 0) {
        std::cout << q_out << std::endl;

        unsigned int rows = q_out.rows();
        unsigned int columns = q_out.columns();

        for (int i = 0; i < columns; ++i) {
            for (int j = 0; j < rows; ++j) {
                std::cout << q_out.data.coeffRef(j, i) << " ";
            }
            std::cout << std::endl;
        }
    } else {
        std::cout << "无解" << std::endl;
    }

    return 0;
}
