include <swivel_joint_bearing.scad>
include <swivel_joint_knuckle.scad>
include <pin_joint_bearing.scad>
include <pin_joint_knuckle.scad>
include <cube_link.scad>

module assembly_1(h_j1b = 50, r_j1b = 100, joint_chamfer = 5){
    //  joint 1 bearing
    translate(v = [0, 0, 0]){
        swivel_joint_bearing(h_j1b, r_j1b, joint_chamfer);
    }
}

//  assembly_1();