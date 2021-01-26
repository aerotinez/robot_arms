include <swivel_joint_bearing.scad>
include <swivel_joint_knuckle.scad>
include <pin_joint_bearing.scad>
include <pin_joint_knuckle.scad>
include <cube_link.scad>

module assembly_6(w_j5k = 100, r_j5k = 50, l_l5 = 20, h_j6b = 20, r_j6b = 50, joint_chamfer = 2){
    rotate([90, -90, 0]){
        union(){
            //  joint 5 knuckle
            pin_joint_knuckle(w_j5k, r_j5k);
            //  link 5
            translate(v = [r_j5k + l_l5/2, 0, 0]){
                cube_link(2*r_j5k, 2*r_j5k, l_l5, 10);
            }
            //  joint 6 bearing
            translate(v = [r_j5k + l_l5 + h_j6b/2, 0, 0]){
                rotate([0, 90, 0]){
                    swivel_joint_bearing(h_j6b, r_j6b, joint_chamfer);
                }
            }
        }
    }
}
//  assembly_6();