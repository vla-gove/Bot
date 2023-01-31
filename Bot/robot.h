//
// Created by v on 1/26/2023.
//
//ROBOT_H
#ifndef ROBOT_H
#define ROBOT_H

#include <cmath>
#include <vector>

class Link {
public:
    double length;
    double mass;
    double inertia;
    // other link parameters
};

class Joint {
public:
    int type; // 0 for revolute, 1 for prismatic
    double q; // joint variable
    double dq; // joint velocity
    double ddq; // joint acceleration
    double torque; // joint torque
    // other joint parameters
};

// missing end effector parameters for calculating inverse kinematics

class Manipulator {
public:
    int dof; // number of degrees of freedom
    std::vector<Link> links;
    std::vector<Joint> joints;
    // other manipulator parameters

    void setDof(int dof) {
        this->dof = dof;
        links.resize(dof);
        joints.resize(dof);
    }

    void setJointType(int index, int type) {
        joints[index].type = type;
    }

    void setJointValue(int index, double q) {
        joints[index].q = q;
    }

    void setJointVelocity(int index, double dq) {
        joints[index].dq = dq;
    }

    void setJointAcceleration(int index, double ddq) {
    joints[index].ddq = ddq;
}

    void setJointTorque(int index, double torque) {
    joints[index].torque = torque;
}

    void setLinkLength(int index, double length) {
    links[index].length = length;
}

    void setLinkMass(int index, double mass) {
    links[index].mass = mass;
}

    void setLinkInertia(int index, double inertia) {
    links[index].inertia = inertia;
}
};

#endif //ROBOT_H