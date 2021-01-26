include <swivel_joint_bearing.scad>
include <swivel_joint_knuckle.scad>
include <pin_joint_bearing.scad>
include <pin_joint_knuckle.scad>
include <cube_link.scad>

module assembly_2(h_j1k = 50, r_j1k = 100, l_l1 = 10, b_l1 = 100, d_l1 = 100, joint_chamfer = 5){
    union(){
        //  joint 1 knuckle
        swivel_joint_knuckle(h_j1k, r_j1k, joint_chamfer);
        //  link 1
        translate(v = [0, 0, h_j1k + l_l1/2]){
            rotate([0, 90, 0]){
                cube_link(b_l1, d_l1, l_l1, 0);
            }
        }
        //  joint 2 bearing
        translate(v = [0, 0, h_j1k + l_l1 + d_l1/2]){
            rotate([90, -90, 0]){
                pin_joint_bearing(b_l1, d_l1/2);
            }
        }
    }
}

//  assembly_2();