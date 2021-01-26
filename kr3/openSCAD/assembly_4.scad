include <swivel_joint_bearing.scad>
include <swivel_joint_knuckle.scad>
include <pin_joint_bearing.scad>
include <pin_joint_knuckle.scad>
include <cube_link.scad>

module assembly_4(w_j3k = 100, r_j3k = 50, l_l3 = 20, h_j4b = 20, r_j4b = 50, joint_chamfer = 2){
    rotate([90, -90, 0]){
        union(){
            //  joint 3 knuckle
            pin_joint_knuckle(w_j3k, r_j3k);
            //  link 3
            translate(v = [r_j3k + l_l3/2, 0, 0]){
                cube_link(2*r_j3k, 2*r_j3k, l_l3, 10);
            }
            //  joint 4 bearing
            translate(v = [r_j3k + l_l3 + h_j4b/2, 0, 0]){
                rotate([0, 90, 0]){
                    swivel_joint_bearing(h_j4b, r_j4b, joint_chamfer);
                }
            }
        }
    }
}

//  assembly_4();